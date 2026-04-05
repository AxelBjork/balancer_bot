// XboxController.h  (header-only includes kept tiny)
#pragma once
#include <memory>

class XboxController {
 public:
  XboxController();
  ~XboxController();

  XboxController(XboxController&&) noexcept;
  XboxController& operator=(XboxController&&) noexcept;

  void update();
  void setDeadzone(float dz);
  void setAxisMap(int leftY_axis, int rightY_axis);

  float leftX() const;
  float rightX() const;
  float leftY() const;
  float rightY() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
