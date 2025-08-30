/****************************************************************************
 *
 *   Copyright (c) 2025 Your Team
 *
 ****************************************************************************/

#include "xbox_remote.h"

#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>

#include <errno.h>
#include <cmath>
#include <cstdlib>

XboxRemote::XboxRemote()
	: ModuleBase{}, ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	// zero arrays
	memset(_axes, 0, sizeof(_axes));
	memset(_buttons, 0, sizeof(_buttons));
	memset(_axmap, 0, sizeof(_axmap));
}

XboxRemote::~XboxRemote()
{
	close_device();
}

int XboxRemote::print_usage(const char *reason)
{
	if (reason)
	{
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR(
### Description
Reads a Linux joystick (tested with Xbox controllers via joydev `/dev/input/js*`) and
publishes `manual_control_setpoint` for manual flight.

Only builds/runs on POSIX (Linux/SITL).
)DESCR");

	PRINT_MODULE_USAGE_NAME("xbox_remote", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/input/js0", nullptr, "Joystick device path", true);
	PRINT_MODULE_USAGE_PARAM_INT('r', 50, 1, 200, "Publish rate (Hz)", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('z', 0.08f, 0.0f, 0.5f, "Deadzone (0..0.5)", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int XboxRemote::custom_command(int argc, char *argv[])
{
	return print_usage("Unrecognized command");
}

int XboxRemote::task_spawn(int argc, char *argv[])
{
	XboxRemote *inst = new XboxRemote();

	if (!inst)
	{
		PX4_ERR("allocation failed");
		return PX4_ERROR;
	}

	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	while ((ch = px4_getopt(argc, argv, "d:r:z:", &myoptind, &myoptarg)) != EOF)
	{
		switch (ch)
		{
		case 'd':
			strncpy(inst->_dev_path, myoptarg, sizeof(inst->_dev_path) - 1);
			inst->_dev_path[sizeof(inst->_dev_path) - 1] = '\0';
			break;
		case 'r':
		{
			int hz = atoi(myoptarg);
			hz = hz < 1 ? 1 : (hz > 200 ? 200 : hz);
			inst->_controller_period_usec = 1000000 / hz;
			break;
		}
		case 'z':
			inst->_deadzone = fmaxf(0.f, fminf(0.5f, static_cast<float>(atof(myoptarg))));
			break;
		default:
			delete inst;
			return PX4_ERROR;
		}
	}

	_object.store(inst);
	_task_id = task_id_is_work_queue;
	return inst->start();
}

int XboxRemote::start()
{
	if (!open_device())
	{
		PX4_WARN("%s not available yet, will retry", _dev_path);
	}
	ScheduleNow();
	return PX4_OK;
}

bool XboxRemote::open_device()
{
	if (_fd >= 0)
	{
		return true;
	}

	_fd = ::open(_dev_path, O_RDONLY | O_NONBLOCK);
	if (_fd < 0)
	{
		return false;
	}

	// Query counts
	if (ioctl(_fd, JSIOCGAXES, &_num_axes) < 0)
	{
		_num_axes = 0;
	}
	if (ioctl(_fd, JSIOCGBUTTONS, &_num_buttons) < 0)
	{
		_num_buttons = 0;
	}

	// Map joystick axes -> ABS_* codes so we can pick sticks by name
	if (ioctl(_fd, JSIOCGAXMAP, _axmap) < 0)
	{
		memset(_axmap, 0, sizeof(_axmap));
	}

	PX4_INFO("Opened %s (axes=%d buttons=%d)", _dev_path, _num_axes, _num_buttons);
	return true;
}

void XboxRemote::close_device()
{
	if (_fd >= 0)
	{
		::close(_fd);
		_fd = -1;
	}
}

int XboxRemote::find_axis_index_by_code(uint8_t abs_code) const
{
	for (int i = 0; i < _num_axes && i < ABS_CNT; i++)
	{
		if (_axmap[i] == abs_code)
			return i;
	}
	return -1;
}

float XboxRemote::apply_deadzone(float v) const
{
	const float s = fabsf(v);
	if (s <= _deadzone)
		return 0.f;
	// rescale so that just outside the deadzone maps to 0
	const float sign = (v >= 0.f) ? 1.f : -1.f;
	const float t = (s - _deadzone) / (1.f - _deadzone);
	return sign * t;
}

void XboxRemote::read_all_events()
{
	if (_fd < 0)
		return;

	for (;;)
	{
		struct js_event e{};
		ssize_t n = ::read(_fd, &e, sizeof(e));
		if (n != (ssize_t)sizeof(e))
			break; // no more

		const uint8_t type = e.type & ~JS_EVENT_INIT;

		if (type == JS_EVENT_AXIS && e.number < kMaxAxes)
		{
			// normalize [-32767, 32767] -> [-1, 1]
			float v = 0.f;
			if (e.value >= 0)
				v = (float)e.value / 32767.f;
			else
				v = (float)e.value / 32768.f; // keep symmetry
			_axes[e.number] = v;
			unsigned code = (e.number < ABS_CNT) ? _axmap[e.number] : 0u;
			PX4_INFO("axis[%u] (ABS=%u) = %.3f", e.number, code, (double)_axes[e.number]);
		}
		else if (type == JS_EVENT_BUTTON && e.number < (int)(sizeof(_buttons)))
		{
			_buttons[e.number] = (e.value != 0);
			PX4_INFO("button[%u] = %d", e.number, _buttons[e.number] ? 1 : 0);
		}
	}
}

void XboxRemote::Run()
{
	if (should_exit())
	{
		close_device();
		exit_and_cleanup();
		return;
	}

	if (_fd < 0)
	{
		(void)open_device();
	}

	read_all_events();

	// Map to RC Mode 2 by default:
	//  left stick: yaw (x), throttle (y)
	//  right stick: roll (x), pitch (y)
	const int idx_lx = find_axis_index_by_code(ABS_X);	// left stick X
	const int idx_ly = find_axis_index_by_code(ABS_Y);	// left stick Y
	const int idx_rx = find_axis_index_by_code(ABS_RX); // right stick X
	const int idx_ry = find_axis_index_by_code(ABS_RY); // right stick Y

	float lx = (idx_lx >= 0) ? _axes[idx_lx] : 0.f;
	float ly = (idx_ly >= 0) ? _axes[idx_ly] : 0.f;
	float rx = (idx_rx >= 0) ? _axes[idx_rx] : 0.f;
	float ry = (idx_ry >= 0) ? _axes[idx_ry] : 0.f;

	// Apply deadzones
	lx = apply_deadzone(lx);
	ly = apply_deadzone(ly);
	rx = apply_deadzone(rx);
	ry = apply_deadzone(ry);

	manual_control_setpoint_s m{};
	m.timestamp = hrt_absolute_time();
	m.timestamp_sample = m.timestamp;

	// mark valid + origin
	m.valid = true;
	m.data_source = manual_control_setpoint_s::SOURCE_RC;

	// RC Mode 2 mapping (Xbox):
	// roll  = right stick X (right = +1)
	// pitch = right stick Y inverted (forward = +1)
	// yaw   = left  stick X (right = +1)
	// throttle = left stick Y mapped to [0..1] (up = 1)
	m.roll = rx;
	m.pitch = -ry;
	m.yaw = lx;
	m.throttle = (-ly + 1.f) * 0.5f;

	// Aux channels from buttons (keep if you like)
	m.aux1 = _buttons[4] ? 1.f : 0.f; // LB
	m.aux2 = _buttons[5] ? 1.f : 0.f; // RB
	m.aux3 = _buttons[0] ? 1.f : 0.f; // A
	m.aux4 = _buttons[1] ? 1.f : 0.f; // B
	m.aux5 = _buttons[2] ? 1.f : 0.f; // X
	m.aux6 = _buttons[3] ? 1.f : 0.f; // Y

	// Pack common Xbox buttons into bitmask A..RS as bits 0..10
	m.buttons = 0;
	for (int i = 0; i <= 10; i++)
	{
		if (i < (int)(sizeof(_buttons)) && _buttons[i])
		{
			m.buttons |= (1u << i);
		}
	}
	// sticks moving flag
	const float maxabs = fmaxf(
		fmaxf(fabsf(m.roll), fabsf(m.pitch)),
		fmaxf(fabsf(m.yaw), fabsf(2.f * m.throttle - 1.f)));
	m.sticks_moving = (maxabs > 0.02f);
	_msp_pub.publish(m);

	ScheduleDelayed(_controller_period_usec);
}

extern "C" __EXPORT int xbox_remote_main(int argc, char *argv[])
{
	return XboxRemote::main(argc, argv);
}