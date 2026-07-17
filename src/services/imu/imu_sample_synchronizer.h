#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <limits>
#include <optional>

struct ImuTimedVector {
  std::array<double, 3> value{};
  int64_t timestamp_ns{0};
};

struct ImuSynchronizedPair {
  ImuTimedVector accel;
  ImuTimedVector gyro;

  int64_t timestamp_ns() const {
    return (accel.timestamp_ns > gyro.timestamp_ns) ? accel.timestamp_ns : gyro.timestamp_ns;
  }

  int64_t skew_ns() const {
    const int64_t delta = accel.timestamp_ns - gyro.timestamp_ns;
    return (delta < 0) ? -delta : delta;
  }
};

class ImuSampleSynchronizer {
 public:
  static constexpr int64_t kDefaultMaxSkewNs = 2'000'000;

  explicit ImuSampleSynchronizer(int64_t max_skew_ns = kDefaultMaxSkewNs)
      : max_skew_ns_(max_skew_ns > 0 ? max_skew_ns : kDefaultMaxSkewNs) {
  }

  std::optional<ImuSynchronizedPair> push_accel(ImuTimedVector sample) {
    accel_.push_back(sample);
    trim(accel_);
    return try_pair();
  }

  std::optional<ImuSynchronizedPair> push_gyro(ImuTimedVector sample) {
    gyro_.push_back(sample);
    trim(gyro_);
    return try_pair();
  }

 private:
  static constexpr size_t kMaxBufferedSamples = 32;

  int64_t max_skew_ns_;
  std::deque<ImuTimedVector> accel_;
  std::deque<ImuTimedVector> gyro_;

  static void trim(std::deque<ImuTimedVector>& samples) {
    while (samples.size() > kMaxBufferedSamples) {
      samples.pop_front();
    }
  }

  std::optional<ImuSynchronizedPair> try_pair() {
    while (!accel_.empty() && !gyro_.empty()) {
      size_t best_accel = 0;
      size_t best_gyro = 0;
      int64_t best_skew = std::numeric_limits<int64_t>::max();
      int64_t best_pair_time = std::numeric_limits<int64_t>::max();

      for (size_t ai = 0; ai < accel_.size(); ++ai) {
        for (size_t gi = 0; gi < gyro_.size(); ++gi) {
          const int64_t delta = accel_[ai].timestamp_ns - gyro_[gi].timestamp_ns;
          const int64_t skew = (delta < 0) ? -delta : delta;
          const int64_t pair_time = (accel_[ai].timestamp_ns > gyro_[gi].timestamp_ns)
                                        ? accel_[ai].timestamp_ns
                                        : gyro_[gi].timestamp_ns;
          if (skew < best_skew || (skew == best_skew && pair_time < best_pair_time)) {
            best_accel = ai;
            best_gyro = gi;
            best_skew = skew;
            best_pair_time = pair_time;
          }
        }
      }

      if (best_skew <= max_skew_ns_) {
        ImuSynchronizedPair pair{accel_[best_accel], gyro_[best_gyro]};
        accel_.erase(accel_.begin(), accel_.begin() + static_cast<std::ptrdiff_t>(best_accel + 1));
        gyro_.erase(gyro_.begin(), gyro_.begin() + static_cast<std::ptrdiff_t>(best_gyro + 1));
        return pair;
      }

      if (accel_.front().timestamp_ns < gyro_.front().timestamp_ns) {
        accel_.pop_front();
      } else {
        gyro_.pop_front();
      }
    }
    return std::nullopt;
  }
};
