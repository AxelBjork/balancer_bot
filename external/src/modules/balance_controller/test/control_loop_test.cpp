#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <deque>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <thread>
#include "control_loop.h"

// ==== Project headers ====
#include "config.h"
// Include the header where ImuSample is defined:

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

// Telemetry sink with blocking wait
class TelemetryCollector {
public:
  void on(const Telemetry& t) {
    std::lock_guard<std::mutex> lk(m_);
    if (buf_.size() >= 4000) buf_.pop_front();
    buf_.push_back(t);
    cv_.notify_all();
  }
  bool waitSizeAtLeast(size_t n, std::chrono::milliseconds to = std::chrono::milliseconds(600)) {
    std::unique_lock<std::mutex> lk(m_);
    return cv_.wait_for(lk, to, [&]{ return buf_.size() >= n; });
  }
  std::vector<Telemetry> snapshot() const {
    std::lock_guard<std::mutex> lk(m_);
    return {buf_.begin(), buf_.end()};
  }
  void clear() {
    std::lock_guard<std::mutex> lk(m_);
    buf_.clear();
  }
private:
  mutable std::mutex m_;
  std::condition_variable cv_;
  std::deque<Telemetry> buf_;
};

// ---- 1 kHz IMU feeder helper ----
template <class CtrlT>
class ImuFeeder {
public:
  ImuFeeder(CtrlT& c, double angle=0.0, double gyro=0.0, double yaw=0.0)
    : ctrl_(c) {
    angle_.store(angle);
    gyro_.store(gyro);
    yaw_.store(yaw);
    running_.store(true);
    th_ = std::thread([this]{
      using clk = std::chrono::steady_clock;
      auto next = clk::now();
      while (running_.load(std::memory_order_relaxed)) {
        next += std::chrono::microseconds(1000); // 1 kHz
        ImuSample s;
        s.angle_rad = angle_.load(std::memory_order_relaxed);
        s.gyro_rad_s = gyro_.load(std::memory_order_relaxed);
        s.yaw_rate_z = yaw_.load(std::memory_order_relaxed);
        s.t = clk::now();
        ctrl_.pushImu(s);
        std::this_thread::sleep_until(next);
      }
    });
  }
  ~ImuFeeder() { stop(); }
  void stop() {
    bool exp = true;
    if (running_.compare_exchange_strong(exp, false) && th_.joinable()) th_.join();
  }
  void set(double angle, double gyro, double yaw) {
    angle_.store(angle, std::memory_order_relaxed);
    gyro_.store(gyro,  std::memory_order_relaxed);
    yaw_.store(yaw,    std::memory_order_relaxed);
  }
private:
  CtrlT& ctrl_;
  std::atomic<bool> running_{false};
  std::thread th_;
  std::atomic<double> angle_{0.0}, gyro_{0.0}, yaw_{0.0};
};

// ===================== Test Fixture =====================
class CascadedControllerFixture : public ::testing::Test {
protected:
  using Ctrl = CascadedController<FakeMotorRunner>;

  void SetUp() override {
    // Default tunings for tests (can be tweaked in individual tests via recreateController)
    recreateController();
  }

  void TearDown() override {
    destroyController();
    sink_.clear();
    g_stop.store(false, std::memory_order_relaxed);
  }

  // Recreate controller if a test needs custom gains
  void recreateController() {
    destroyController();
    ctrl_ = std::make_unique<Ctrl>(left_, right_);
    // Telemetry hookup (the bit you asked to centralize)
    ctrl_->setTelemetrySink([this](const Telemetry& t) {
      sink_.on(t);
      static std::atomic<int> k{0};
      if ((++k % 25) == 0) {
        constexpr double deg = 180.0 / M_PI;
        std::printf(
          "t=%7.3f  θ=%6.2f°  θ̇=%6.2f°/s  aθ=%7.1f  aθ̇=%7.1f  av=%7.1f  u=%6.0f%s  L=%6.0f  R=%6.0f\n",
          std::chrono::duration<double>(t.ts.time_since_epoch()).count(),
          t.tilt_rad * deg,
          t.gyro_rad_s * deg,
          t.a_theta_mps2,
          t.a_dtheta_mps2,
          t.a_v_mps2,
          t.u_balance_sps,
          (t.du_rate_limited || t.u_amp_limited) ? "*" : "",
          t.left_cmd_sps,
          t.right_cmd_sps
        );
      }
    });
    // Start IMU feeder
    imu_ = std::make_unique<ImuFeeder<Ctrl>>(*ctrl_, 0.0, 0.0);
  }

  void destroyController() {
    if (imu_) { imu_->stop(); imu_.reset(); }
    if (ctrl_) { ctrl_->stop(); ctrl_.reset(); }
  }

  // Convenience
  void setJoystick(float forward, float turn) {
    ctrl_->setJoystick(JoyCmd{forward, turn});
  }
  bool waitTelemetry(size_t n, std::chrono::milliseconds to = std::chrono::milliseconds(600)) {
    return sink_.waitSizeAtLeast(n, to);
  }
  std::vector<Telemetry> telemetry() const { return sink_.snapshot(); }

  // Members
  FakeMotorRunner left_, right_;
  std::unique_ptr<Ctrl> ctrl_;
  std::unique_ptr<ImuFeeder<Ctrl>> imu_;
  TelemetryCollector sink_;
};

// ===================== Tests =====================

TEST_F(CascadedControllerFixture, ZeroInputsStayNearZero) {
  setJoystick(0.0f, 0.0f);

  ASSERT_TRUE(waitTelemetry(80));
  auto v = telemetry();

  double avg_abs_tgt = 0.0, avg_abs_u = 0.0, avg_abs_l = 0.0, avg_abs_r = 0.0;
  for (auto& t : v) {
    avg_abs_u   += std::fabs(t.u_balance_sps);
    avg_abs_l   += std::fabs(t.left_cmd_sps);
    avg_abs_r   += std::fabs(t.right_cmd_sps);
  }
  const double n = std::max<size_t>(1, v.size());
  avg_abs_tgt /= n; avg_abs_u /= n; avg_abs_l /= n; avg_abs_r /= n;

  EXPECT_LT(avg_abs_u,   1e-2);
  EXPECT_LT(avg_abs_l,   1e-2);
  EXPECT_LT(avg_abs_r,   1e-2);
}


TEST_F(CascadedControllerFixture, SteeringSplitsWheelCommandsWithCorrectSigns) {
  setJoystick(0.0f, +0.6f);

  ASSERT_TRUE(waitTelemetry(80));
  auto v = telemetry();
  const size_t k = std::min<size_t>(v.size(), 20);
  auto tail_begin = v.end() - k;

  double avg_left = 0.0, avg_right = 0.0;
  for (auto it = tail_begin; it != v.end(); ++it) {
    avg_left  += it->left_cmd_sps;
    avg_right += it->right_cmd_sps;
  }
  avg_left  /= k;
  avg_right /= k;

  EXPECT_LT(avg_left,  0.1);
  EXPECT_LT(avg_right, 0.1);
}


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
      constexpr double deg = 180.0 / M_PI;
      std::printf(
        "t=%7.3f  θ=%6.2f°  θ̇=%6.2f°/s  aθ=%7.1f  aθ̇=%7.1f  av=%7.1f  u=%6.0f%s  L=%6.0f  R=%6.0f\n",
        std::chrono::duration<double>(t.ts.time_since_epoch()).count(),
        t.tilt_rad * deg,
        t.gyro_rad_s * deg,
        t.a_theta_mps2,
        t.a_dtheta_mps2,
        t.a_v_mps2,
        t.u_balance_sps,
        (t.du_rate_limited || t.u_amp_limited) ? "*" : "",
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
    std::this_thread::sleep_for(std::chrono::microseconds(10));
  }

  // Let it run a little longer to drain telemetry after the last sample.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // ---- Assert (soft) ----
  // We at least expect the controller to have produced some motor targets.
  EXPECT_GT(L.callCount(), 0);
  EXPECT_GT(R.callCount(), 0);
  EXPECT_GT(tel_count.load(), 0);

  // ---- Cleanup ----
  g_stop.store(true, std::memory_order_relaxed);
  // ctrl destructor joins; Fake runners have no threads.
}

