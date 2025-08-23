#include <gtest/gtest.h>
#include <deque>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <thread>
#include "control_loop.h"


// Stub motor runner (matches interface: setTarget(double))
struct FakeMotorRunner {
  void setTarget(double sps) noexcept { last.store(sps, std::memory_order_relaxed); }
  std::atomic<double> last{0.0};
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
  ImuFeeder(CtrlT& c, double angle=0.0, double gyro=0.0)
    : ctrl_(c) {
    angle_.store(angle);
    gyro_.store(gyro);
    running_.store(true);
    th_ = std::thread([this]{
      using clk = std::chrono::steady_clock;
      auto next = clk::now();
      while (running_.load(std::memory_order_relaxed)) {
        next += std::chrono::microseconds(1000); // 1 kHz
        ImuSample s;
        s.angle_rad = angle_.load(std::memory_order_relaxed);
        s.gyro_rad_s = gyro_.load(std::memory_order_relaxed);
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
  void set(double angle, double gyro) {
    angle_.store(angle, std::memory_order_relaxed);
    gyro_.store(gyro, std::memory_order_relaxed);
  }
private:
  CtrlT& ctrl_;
  std::atomic<bool> running_{false};
  std::thread th_;
  std::atomic<double> angle_{0.0}, gyro_{0.0};
};

// ===================== Test Fixture =====================
class CascadedControllerFixture : public ::testing::Test {
protected:
  using Ctrl = CascadedController<FakeMotorRunner>;

  void SetUp() override {
    // Default tunings for tests (can be tweaked in individual tests via recreateController)
    gains_.hz_balance = 300;
    gains_.hz_outer   = 100;

    recreateController(gains_);
  }

  void TearDown() override {
    destroyController();
    sink_.clear();
    g_stop.store(false, std::memory_order_relaxed);
  }

  // Recreate controller if a test needs custom gains
  void recreateController(const ControlTunings& g) {
    destroyController();
    gains_ = g;
    ctrl_ = std::make_unique<Ctrl>(left_, right_, gains_);
    // Telemetry hookup (the bit you asked to centralize)
    ctrl_->setTelemetrySink([this](const Telemetry& t) {
      sink_.on(t);
      static std::atomic<int> k{0};
      if ((++k % 25) == 0) {
        std::printf("tgt=%.2f° tilt=%.2f° u=%.0f L=%.0f R=%.0f velErr=%.0f sat=%d\n",
          t.tilt_target_rad * 180.0/M_PI, t.tilt_rad * 180.0/M_PI,
          t.u_balance_sps, t.left_cmd_sps, t.right_cmd_sps,
          t.vel_err_sps, int(t.tilt_saturated));
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
  ControlTunings gains_{};
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
    avg_abs_tgt += std::fabs(t.tilt_target_rad);
    avg_abs_u   += std::fabs(t.u_balance_sps);
    avg_abs_l   += std::fabs(t.left_cmd_sps);
    avg_abs_r   += std::fabs(t.right_cmd_sps);
  }
  const double n = std::max<size_t>(1, v.size());
  avg_abs_tgt /= n; avg_abs_u /= n; avg_abs_l /= n; avg_abs_r /= n;

  EXPECT_LT(avg_abs_tgt, 1e-3);
  EXPECT_LT(avg_abs_u,   1e-2);
  EXPECT_LT(avg_abs_l,   1e-2);
  EXPECT_LT(avg_abs_r,   1e-2);
}

TEST_F(CascadedControllerFixture, ForwardCommandCreatesPositiveTiltTargetAndForwardSps) {
  setJoystick(0.4f, 0.0f);

  ASSERT_TRUE(waitTelemetry(120));
  auto v = telemetry();
  const size_t k = std::min<size_t>(v.size(), 30);
  auto tail_begin = v.end() - k;

  double avg_tilt_target = 0.0;
  double avg_base = 0.0;
  for (auto it = tail_begin; it != v.end(); ++it) {
    avg_tilt_target += it->tilt_target_rad;
    avg_base += 0.5 * (it->left_cmd_sps + it->right_cmd_sps);
  }
  avg_tilt_target /= k;
  avg_base /= k;

  EXPECT_GT(avg_tilt_target, 0.0);
  EXPECT_GT(avg_base, 0.0);
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

  EXPECT_GT(avg_left,  0.0);
  EXPECT_LT(avg_right, 0.0);
}

TEST_F(CascadedControllerFixture, TiltTargetIsClamped) {
  // Recreate with tighter tilt clamp & stronger outer gains to force saturation
  ControlTunings g = gains_;
  g.max_tilt_rad = 4.0 * (M_PI / 180.0);
  g.kp_vel = 5.0; g.ki_vel = 2.0;
  recreateController(g);

  setJoystick(+1.0f, 0.0f);

  ASSERT_TRUE(waitTelemetry(120));
  auto v = telemetry();

  bool saw_sat = false;
  for (auto& t : v) {
    EXPECT_LE(std::fabs(t.tilt_target_rad), g.max_tilt_rad + 1e-9);
    if (t.tilt_saturated) saw_sat = true;
  }
  EXPECT_TRUE(saw_sat);
}
