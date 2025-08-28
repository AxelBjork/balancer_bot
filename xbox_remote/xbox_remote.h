/****************************************************************************
 *
 *   Copyright (c) 2025 Your Team. All rights reserved.
 *
 *   Based on PX4 heater driver skeleton (c) 2018-2020 PX4 Development Team
 *
 ****************************************************************************/

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/topics/manual_control_setpoint.h>

#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <linux/joystick.h>
#include <linux/input.h>
#include <string.h>

using namespace time_literals;

#ifndef MODULE_NAME
#define MODULE_NAME "xbox_remote"
#endif

class XboxRemote : public ModuleBase<XboxRemote>, public px4::ScheduledWorkItem
{
public:
  XboxRemote();
  ~XboxRemote() override;

  static int task_spawn(int argc, char *argv[]);
  static int custom_command(int argc, char *argv[]);
  static int print_usage(const char *reason = nullptr);

  int start();

private:
  void Run() override;
  bool open_device();
  void close_device();
  void read_all_events();
  float apply_deadzone(float v) const;

  // joystick state
  int _fd{-1};
  char _dev_path[64]{"/dev/input/js0"};
  int _controller_period_usec{20000}; // 50 Hz
  float _deadzone{0.08f};

  // Axis values normalized to [-1, 1]. Indexed by ABS_* code where applicable
  static constexpr int kMaxAxes = 16;
  float _axes[kMaxAxes]{}; // zero-initialized
  uint8_t _axmap[ABS_CNT]{}; // mapping from js axis index -> ABS_* code
  int _num_axes{0};
  int _num_buttons{0};
  bool _buttons[32]{};

  // Publication
  uORB::Publication<manual_control_setpoint_s> _msp_pub{ORB_ID(manual_control_setpoint)};

  // mapping helpers
  int find_axis_index_by_code(uint8_t abs_code) const;
};