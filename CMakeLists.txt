cmake_minimum_required(VERSION 3.18)
project(MotorQuickBuild LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Options: build the quick test by default; app is opt-in and *hard* requires libs.
option(BUILD_TESTS "Build unit tests" ON)

find_package(Threads REQUIRED)

# ---------- SDL2 finder (module first, then pkg-config) ----------
set(HAVE_SDL2 FALSE)
set(SDL2_INCS "")
set(SDL2_LIBS "")
find_package(SDL2 QUIET)
if (SDL2_FOUND)
  set(HAVE_SDL2 TRUE)
  if (TARGET SDL2::SDL2)
    set(SDL2_LIBS SDL2::SDL2)
  else()
    # Some FindSDL2.cmake modules only set variables
    set(SDL2_INCS ${SDL2_INCLUDE_DIRS})
    set(SDL2_LIBS ${SDL2_LIBRARIES})
  endif()
else()
  find_package(PkgConfig QUIET)
  if (PkgConfig_FOUND)
    pkg_check_modules(SDL2 QUIET sdl2)
    if (SDL2_FOUND OR SDL2_LIBRARIES)
      set(HAVE_SDL2 TRUE)
      set(SDL2_INCS ${SDL2_INCLUDE_DIRS})
      set(SDL2_LIBS ${SDL2_LIBRARIES})
    endif()
  endif()
endif()
# ================================================================
#                       Source Definition
# ================================================================
set(SRC_DIR ${CMAKE_SOURCE_DIR}/external/src/modules/balance_controller)
set(BALANCER_SOURCES
    ${SRC_DIR}/xbox_controller.cpp
    ${SRC_DIR}/control_loop.cpp
    ${SRC_DIR}/ism330_iio_reader.cpp
    ${SRC_DIR}/pitch_lpf.cpp
    ${SRC_DIR}/config_pid_io.cpp
)

# ================================================================
#                       Tests (default ON)
# ================================================================
if (BUILD_TESTS)
  include(CTest)
  include(GoogleTest)

  enable_testing()
  # GoogleTest via FetchContent
  include(FetchContent)

  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
  )
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  message(STATUS "Building tests")
  add_library(balancer_common STATIC ${BALANCER_SOURCES})
  target_include_directories(balancer_common PUBLIC
    ${SRC_DIR}
    ${SRC_DIR}/stubs
    ${SDL2_INCS}
    ${CMAKE_SOURCE_DIR}/PX4-Autopilot/src/lib/rate_control
    ${CMAKE_SOURCE_DIR}/PX4-Autopilot/src/lib/matrix
    ${CMAKE_SOURCE_DIR}/PX4-Autopilot/src/lib
    ${CMAKE_SOURCE_DIR}/PX4-Autopilot/platforms/common/include
    ${CMAKE_SOURCE_DIR}/PX4-Autopilot/src/modules
    ${CMAKE_SOURCE_DIR}/px4_stub
  )

  target_compile_definitions(balancer_common PUBLIC
    PIGPIOD_STUB_IMPL
  )
  target_link_libraries(balancer_common PUBLIC
    GTest::gmock_main GTest::gtest
    Threads::Threads
    ${SDL2_LIBS}
  )

  set(PX4_DIR ${CMAKE_SOURCE_DIR}/PX4-Autopilot)
  add_library(px4_ratecontrol STATIC
    ${PX4_DIR}/src/lib/rate_control/rate_control.cpp
    ${PX4_DIR}/src/lib/pid/PID.cpp
  )
  target_include_directories(px4_ratecontrol PUBLIC
    ${PX4_DIR}/src/lib/rate_control
    ${PX4_DIR}/src/lib/matrix
    ${PX4_DIR}/src/lib
    ${PX4_DIR}/src/lib/mathlib
    ${PX4_DIR}/src/lib/mathlib/math
    ${PX4_DIR}/src
    ${PX4_DIR}/platforms
    ${PX4_DIR}/platforms/common/include
    ${CMAKE_SOURCE_DIR}/px4_stub
  )
  target_link_libraries(balancer_common PUBLIC px4_ratecontrol)

  set(TEST_DIR ${CMAKE_SOURCE_DIR}/external/src/modules/balance_controller/test)
  # 2) motor_controller_test (existing)
  add_executable(balancer_tests
    ${TEST_DIR}/motor_runner_test.cpp
    ${TEST_DIR}/control_loop_test.cpp
    ${TEST_DIR}/latency_test.cpp
    ${CMAKE_SOURCE_DIR}/external/src/modules/balance_controller/stubs/pigpiod_stub.cpp
  )

  target_link_libraries(balancer_tests PRIVATE balancer_common)
  ## add_test(NAME motor_controller_test COMMAND motor_controller_test)

  # 3) control_loop_test (new)
  #add_executable(control_loop_test
  #  ${CMAKE_SOURCE_DIR}/external/src/modules/balance_controller/control_loop_test.cpp
  #)
  #target_link_libraries(control_loop_test PRIVATE
  #  balancer_common
  #)

  #add_test(NAME control_loop_test COMMAND control_loop_test)
  gtest_discover_tests(balancer_tests
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    DISCOVERY_TIMEOUT 60
  )

  # 4) balancer_simulator (standalone sim)
  add_executable(balancer_simulator
    ${CMAKE_SOURCE_DIR}/external/src/modules/balance_controller/test/simulator_main.cpp
    ${CMAKE_SOURCE_DIR}/external/src/modules/balance_controller/test/simulator/balancer_simulator.cpp
    ${CMAKE_SOURCE_DIR}/external/src/modules/balance_controller/stubs/pigpiod_stub.cpp
  )
  target_link_libraries(balancer_simulator PRIVATE balancer_common)
# ================================================================
#                       App (opt-in, hard checks)
# ================================================================
else()
  set(CMAKE_CXX_COMPILER_LAUNCHER ccache)           # ccache

  if (NOT HAVE_SDL2)
    message(FATAL_ERROR "SDL2 not found. Install libsdl2-dev.")
  endif()

  # Require libpigpiod_if2 for the app
  find_library(PIGPIOD_LIB pigpiod_if2)
  if (NOT PIGPIOD_LIB)
    message(FATAL_ERROR "libpigpiod_if2 not found. Install pigpio (daemon client).")
  endif()

  set(PX4_DIR ${CMAKE_SOURCE_DIR}/PX4-Autopilot)
  add_library(px4_ratecontrol STATIC
    ${PX4_DIR}/src/lib/rate_control/rate_control.cpp
    ${PX4_DIR}/src/lib/pid/PID.cpp
  )

  target_include_directories(px4_ratecontrol PUBLIC
    ${PX4_DIR}/src/lib/rate_control   # RateControl.hpp
    ${PX4_DIR}/src/lib/matrix                            # PX4 matrix headers
    ${PX4_DIR}/src/lib                                   # mathlib lives here
    ${PX4_DIR}/src
    ${PX4_DIR}/platforms                # px4_platform_common/defines.h
    ${PX4_DIR}/platforms/common/include                  # common px4 includes (harmless)
    ${CMAKE_SOURCE_DIR}/px4_stub                         # px4_boardconfig.h stub
)


  # Define balancer_common for App (if NOT building tests)
  # But we also used it above. Wait, if we are in else(), we are NOT building tests.
  # So we define it here for the App case.
  add_library(balancer_common STATIC ${BALANCER_SOURCES})

  target_precompile_headers(balancer_common PRIVATE
    <atomic> <thread> <chrono> <vector> <string> <unordered_map> <array> <algorithm> <cstdio> <cmath>
  )

  target_link_libraries(balancer_common
  PUBLIC px4_ratecontrol
  )

  target_include_directories(balancer_common PUBLIC
    ${CMAKE_SOURCE_DIR}/src
    ${SDL2_INCS}
  )
  target_link_libraries(balancer_common PUBLIC
    Threads::Threads
    ${SDL2_LIBS}
    ${PIGPIOD_LIB}
  )
  add_executable(imu_demo
    ${CMAKE_SOURCE_DIR}/external/src/modules/balance_controller/extras/imu_demo.cpp
  )

  target_link_libraries(imu_demo
  PRIVATE balancer_common
  )

endif()

# ============================================================
# Production executable for Raspberry Pi (when not building tests)
# ============================================================
if(NOT BUILD_TESTS)
  message(STATUS "Building production executable for Raspberry Pi")
  
  # Main balancer executable
  add_executable(balancer_pi
    ${CMAKE_SOURCE_DIR}/external/src/modules/balance_controller/main.cpp
  )
  
  target_link_libraries(balancer_pi PRIVATE
    balancer_common
    pigpiod_if2
    ${SDL2_LIBS}
    Threads::Threads
  )
  
  target_include_directories(balancer_pi PRIVATE
    ${CMAKE_SOURCE_DIR}/external/src/modules/balance_controller
    ${SDL2_INCS}
  )
  
  # Install to system (optional)
  install(TARGETS balancer_pi DESTINATION bin)
endif()
