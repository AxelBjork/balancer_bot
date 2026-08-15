function(add_ipc_hub_target)
  add_library(ipc_hub STATIC
    ${CMAKE_SOURCE_DIR}/src/ipc/message_bus.cpp
    ${CMAKE_SOURCE_DIR}/src/ipc/udp_bridge.cpp
  )
  target_include_directories(ipc_hub PUBLIC
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/ipc
    ${CMAKE_SOURCE_DIR}/src/messages
  )
  enable_project_warnings(ipc_hub)
endfunction()

function(add_px4_ratecontrol_target)
  set(PX4_DIR ${CMAKE_SOURCE_DIR}/src/px4_stub)

  add_library(px4_ratecontrol STATIC
    ${PX4_DIR}/src/lib/rate_control/rate_control.cpp
    ${PX4_DIR}/src/lib/pid/PID.cpp
    ${PX4_DIR}/src/modules/attitude_estimator_q/AttitudeEstimatorQ.cpp
  )

  target_include_directories(px4_ratecontrol PUBLIC
    ${PX4_DIR}/src/lib/rate_control
    ${PX4_DIR}/src/lib/matrix
    ${PX4_DIR}/src/lib
    ${PX4_DIR}/src/lib/mathlib
    ${PX4_DIR}/src/lib/mathlib/math
    ${PX4_DIR}/src/modules/attitude_estimator_q
    ${PX4_DIR}/src
    ${PX4_DIR}
  )

  enable_project_warnings(px4_ratecontrol)
endfunction()

function(add_balancer_common_target)
  set(options USE_PIGPIO_STUB PI_OPTIMIZED)
  set(oneValueArgs SDL_LIBS PIGPIOD_LIB)
  cmake_parse_arguments(BAL "${options}" "${oneValueArgs}" "" ${ARGN})

  set(control_dir ${CMAKE_SOURCE_DIR}/src/services/control)
  set(imu_dir ${CMAKE_SOURCE_DIR}/src/services/imu)
  set(input_dir ${CMAKE_SOURCE_DIR}/src/services/input)
  set(main_dir ${CMAKE_SOURCE_DIR}/src/services/main)
  set(balancer_sources
    ${control_dir}/rate_controller_core.cpp
    ${control_dir}/control_service.cpp
    ${imu_dir}/ism330_iio_reader.cpp
    ${imu_dir}/imu_pitch_estimator.cpp
    ${imu_dir}/imu_service.cpp
    ${input_dir}/input_service.cpp
    ${input_dir}/xbox_controller.cpp
    ${main_dir}/config_pid_io.cpp
    ${CMAKE_SOURCE_DIR}/src/services/motor/motor_service.cpp
    ${CMAKE_SOURCE_DIR}/src/services/time/time_service.cpp
  )

  add_library(balancer_common STATIC ${balancer_sources})

  target_include_directories(balancer_common PUBLIC
    ${CMAKE_SOURCE_DIR}/src
    ${control_dir}
    ${imu_dir}
    ${input_dir}
    ${main_dir}
    ${BAL_INCLUDE_DIRS}
  )

  target_link_libraries(balancer_common PUBLIC
    px4_ratecontrol
    Threads::Threads
    ipc_hub
  )

  if (BAL_SDL_LIBS)
    target_link_libraries(balancer_common PUBLIC ${BAL_SDL_LIBS})
  endif()

  if (BAL_USE_PIGPIO_STUB)
    target_include_directories(balancer_common PUBLIC
      ${CMAKE_SOURCE_DIR}/tests/stubs
      ${CMAKE_SOURCE_DIR}/PX4-Autopilot/src/lib/rate_control
      ${CMAKE_SOURCE_DIR}/PX4-Autopilot/src/lib/matrix
      ${CMAKE_SOURCE_DIR}/PX4-Autopilot/src/lib
      ${CMAKE_SOURCE_DIR}/PX4-Autopilot/platforms/common/include
      ${CMAKE_SOURCE_DIR}/src/px4_stub/src/modules
      ${CMAKE_SOURCE_DIR}/src/px4_stub
    )
    target_compile_definitions(balancer_common PUBLIC PIGPIOD_STUB_IMPL)
  endif()

  if (BAL_PIGPIOD_LIB)
    target_link_libraries(balancer_common PUBLIC ${BAL_PIGPIOD_LIB})
  endif()

  if (BAL_PI_OPTIMIZED)
    target_precompile_headers(balancer_common PRIVATE
      <atomic> <thread> <chrono> <vector> <string> <unordered_map> <array> <algorithm> <cstdio> <cmath>
    )
    enable_pi_optimization(balancer_common)
    enable_pi_optimization(px4_ratecontrol)
  endif()

  enable_project_warnings(balancer_common)
endfunction()
