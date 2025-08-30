#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <functional>
#include <future>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

// ==== Project headers ====
#include "config.h"
// Include the header where ImuSample is defined:
#include "control_loop.h"     // only for interface parity; not strictly needed

// If g_stop is defined elsewhere, remove this extern and include the header.
// Otherwise, declare it here to satisfy the test build.
extern std::atomic<bool> g_stop;

// ================== A minimal fake MotorRunner ==================
struct FakeMotorRunner {
  void setTarget(double sps) {
    last_target.store(sps, std::memory_order_relaxed);
    calls.fetch_add(1, std::memory_order_relaxed);
  }
  void stop() {}
  double last() const { return last_target.load(std::memory_order_relaxed); }
  int callCount() const { return calls.load(std::memory_order_relaxed); }
private:
  std::atomic<double> last_target{0.0};
  std::atomic<int>    calls{0};
};

// ================== CSV loader for ImuSample ==================
/*
Expected CSV columns per line (header optional):
time_s, angle_rad, gyro_rad_s, yaw_rate_z

- If a header is present, it will be skipped automatically.
- Extra columns are ignored. Whitespace is allowed.
*/
static std::vector<ImuSample> loadImuCsv(const std::string& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("Couldn't open CSV: " + path);

  using clk = std::chrono::steady_clock;
  const auto t0 = clk::now();

  std::vector<ImuSample> out;
  out.reserve(10000);

  std::string line;

  // Read header and verify (optional but helpful)
  if (!std::getline(f, line)) return out;
  // Accept exact header; ignore whitespace
  {
    std::istringstream iss(line);
    std::string h0,h1,h2,h3;
    auto trim=[](std::string s){
      size_t b=s.find_first_not_of(" \t\r\n"), e=s.find_last_not_of(" \t\r\n");
      return (b==std::string::npos) ? std::string() : s.substr(b, e-b+1);
    };
    std::getline(iss,h0,','); std::getline(iss,h1,','); std::getline(iss,h2,','); std::getline(iss,h3,',');
    if (trim(h0)!="pitch" || trim(h1)!="dpitch" || trim(h2)!="yaw" || trim(h3)!="time") {
      throw std::runtime_error("Unexpected CSV header; expected: pitch,dpitch,yaw,time");
    }
  }

  // Parse all rows
  std::vector<double> times;
  struct Row { double pitch_deg, dpitch_dps, yaw_dps, t_s; };
  std::vector<Row> rows; rows.reserve(10000);

  while (std::getline(f, line)) {
    if (line.empty()) continue;
    std::istringstream iss(line);
    std::string c0,c1,c2,c3;
    if (!std::getline(iss,c0,',')) continue;
    if (!std::getline(iss,c1,',')) continue;
    if (!std::getline(iss,c2,',')) continue;
    if (!std::getline(iss,c3,',')) continue;
    try {
      rows.push_back(Row{
        std::stod(c0), std::stod(c1), std::stod(c2), std::stod(c3)
      });
    } catch (...) { /* skip malformed line */ }
  }
  if (rows.empty()) return out;

  const double t0_s = rows.front().t_s;

  for (const auto& r : rows) {
    ImuSample s{};
    s.angle_rad  = r.pitch_deg;
    s.gyro_rad_s = r.dpitch_dps;
    s.yaw_rate_z = r.yaw_dps;
    const double dt = std::max(0.0, r.t_s - t0_s);
    s.t = t0 + std::chrono::duration_cast<clk::duration>(std::chrono::duration<double>(dt));
    out.push_back(s);
  }
  return out;
}

// ================== The actual test ==================
TEST(CascadedControllerCsvReplayTest, ReplaysCsvAndPrintsTelemetry) {
  // ---- Arrange ----
  g_stop.store(false, std::memory_order_relaxed);

  // Set where your CSV lives. You can switch to an env var for convenience.
  const std::string csv_path = "../data/imu_data.csv";

  std::vector<ImuSample> samples = loadImuCsv(csv_path);
  ASSERT_FALSE(samples.empty()) << "CSV is empty: " << csv_path;

  FakeMotorRunner L, R;

  CascadedController<FakeMotorRunner> ctrl(L, R);

  // Zero joystick for a pure balance/yaw response; tweak if you want motion.
  ctrl.setJoystick(JoyCmd{0.0f, 0.0f});

  // Telemetry printing — throttle to every Nth callback if you want less spam.
  std::atomic<int> tel_count{0};
  constexpr int kPrintEvery = 10; // ~10 Hz with 400 Hz balance loop
  ctrl.setTelemetrySink([&](const Telemetry& t) {
    int n = ++tel_count;
    if (kPrintEvery > 0 && (n % kPrintEvery) != 0) return;

    const double deg = 180.0 / M_PI;
    std::printf(
    "time=%8.3fs  pitch=%8.2f°  target_pitch=%8.2f°  "
    "balance_output=%7.0f  left_command=%7.0f  right_command=%7.0f\n",
    std::chrono::duration<double>(t.ts.time_since_epoch()).count(),
    t.tilt_rad * deg,
    t.tilt_target_rad * deg,
    t.u_balance_sps,
    t.left_cmd_sps,
    t.right_cmd_sps
    );
  });

  // ---- Act ----
  // Feed all IMU samples. We don't need to match real-time pacing; the controller
  // runs in its own thread and will consume the latest IMU state at its balance rate.
  for (const auto& s : samples) {
    ctrl.pushImu(s);
    // A tiny pause gives the controller chances to run between bursts.
    std::this_thread::sleep_for(std::chrono::microseconds(200));
  }

  // Let it run a little longer to drain telemetry after the last sample.
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // ---- Assert (soft) ----
  // We at least expect the controller to have produced some motor targets.
  EXPECT_GT(L.callCount(), 0);
  EXPECT_GT(R.callCount(), 0);
  EXPECT_GT(tel_count.load(), 0);

  // ---- Cleanup ----
  g_stop.store(true, std::memory_order_relaxed);
  // ctrl destructor joins; Fake runners have no threads.
}
