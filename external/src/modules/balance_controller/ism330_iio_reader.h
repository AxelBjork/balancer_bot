// ism330_iio_reader.h
#pragma once
#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

class Ism330IioReader {
public:
  using TimePoint = std::chrono::steady_clock::time_point;

  struct IMUConfig {
    std::function<void(double               /*pitch*/,
                       std::array<double,3> /*acc*/,
                       std::array<double,3> /*gyr*/,
                       TimePoint            /*ts*/)> on_sample;
  };

  explicit Ism330IioReader(IMUConfig cfg);
  ~Ism330IioReader();

  Ism330IioReader(Ism330IioReader&&) noexcept;
  Ism330IioReader& operator=(Ism330IioReader&&) noexcept;

  void stop();
  std::string devnode() const;

private:
  struct Impl;
  std::unique_ptr<Impl> p_;
};
