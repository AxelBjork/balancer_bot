cmake_minimum_required(VERSION 3.18)
project(MotorQuickBuild LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Options: build the quick test by default; app is opt-in and *hard* requires libs.
option(BUILD_TESTS "Build unit tests" ON)
option(
  BUILD_REFLECTION_ARTIFACTS
  "Build reflection-generated docs and Python bindings as part of the default host build"
  ${BUILD_TESTS}
)

find_package(Threads REQUIRED)
include(cmake/balancer_common.cmake)

add_library(project_warnings INTERFACE)
target_compile_options(project_warnings INTERFACE
  $<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang>:-Wall;-Wextra;-Werror>
)

add_library(project_pi_optimization INTERFACE)
target_compile_options(project_pi_optimization INTERFACE
  $<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang>:-O2>
)

function(enable_project_warnings target_name)
  target_link_libraries(${target_name} PUBLIC project_warnings)
endfunction()

function(enable_pi_optimization target_name)
  target_link_libraries(${target_name} PUBLIC project_pi_optimization)
endfunction()

add_ipc_hub_target()

# ── C++26 Reflection (host-side only) ─────────────────────────────────────────
if (BUILD_REFLECTION_ARTIFACTS AND NOT CMAKE_CROSSCOMPILING)
  include(cmake/reflection.cmake)
elseif (BUILD_REFLECTION_ARTIFACTS AND CMAKE_CROSSCOMPILING)
  message(STATUS "Skipping reflection artifacts for cross-compiling build")
endif()

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
set(SRC_DIR ${CMAKE_SOURCE_DIR}/src/balance_controller)

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
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  )
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  message(STATUS "Building tests")
  add_px4_ratecontrol_target()
  add_balancer_common_target(
    USE_PIGPIO_STUB
    SDL_LIBS "${SDL2_LIBS}"
  )

  set(TEST_DIR ${CMAKE_SOURCE_DIR}/src/balance_controller/test)
  add_library(balancer_sim_support STATIC
    ${TEST_DIR}/simulator/balancer_simulator.cpp
    ${TEST_DIR}/simulator/simulator_runner.cpp
  )
  target_link_libraries(balancer_sim_support PRIVATE balancer_common)
  target_include_directories(balancer_sim_support PUBLIC
    ${TEST_DIR}
    ${TEST_DIR}/simulator
  )
  enable_project_warnings(balancer_sim_support)

  # 2) motor_controller_test (existing)
  add_executable(balancer_tests
    ${TEST_DIR}/motor_runner_test.cpp
    ${TEST_DIR}/control_loop_test.cpp
    ${TEST_DIR}/simulator_runner_test.cpp
    ${TEST_DIR}/time_service_test.cpp
    ${TEST_DIR}/latency_test.cpp
    ${CMAKE_SOURCE_DIR}/src/balance_controller/stubs/pigpiod_stub.cpp
  )

  target_compile_definitions(balancer_tests PRIVATE
    BALANCER_REPO_ROOT="${CMAKE_SOURCE_DIR}"
  )
  target_link_libraries(balancer_tests PRIVATE balancer_common balancer_sim_support)
  ## add_test(NAME motor_controller_test COMMAND motor_controller_test)

  gtest_discover_tests(balancer_tests
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    DISCOVERY_TIMEOUT 60
  )

  # 4) balancer_simulator (standalone sim)
  add_executable(balancer_simulator
    ${CMAKE_SOURCE_DIR}/src/balance_controller/test/simulator_main.cpp
    ${CMAKE_SOURCE_DIR}/src/balance_controller/stubs/pigpiod_stub.cpp
  )
  target_compile_definitions(balancer_simulator PRIVATE
    BALANCER_REPO_ROOT="${CMAKE_SOURCE_DIR}"
  )
  target_link_libraries(balancer_simulator PRIVATE balancer_common balancer_sim_support)

  # 5) sil_app (Hardware-in-the-loop simulation/testing)
  add_executable(sil_app
    ${CMAKE_SOURCE_DIR}/src/sil_app.cpp
    ${CMAKE_SOURCE_DIR}/src/balance_controller/stubs/pigpiod_stub.cpp # Stub pigpiod for SIL
  )
  target_link_libraries(sil_app PRIVATE balancer_common)
  target_include_directories(sil_app PRIVATE ${CMAKE_SOURCE_DIR}/src)
# ================================================================
#                       App (opt-in, hard checks)
# ================================================================
else()
  set(CMAKE_CXX_COMPILER_LAUNCHER ccache)           # ccache

  if (NOT HAVE_SDL2)
    message(FATAL_ERROR "SDL2 not found. Install libsdl2-dev.")
  endif()

  # Require libpigpiod_if2 for the app
  find_library(PIGPIOD_LIB pigpiod_if2 HINTS /usr/lib/aarch64-linux-gnu)
  if (NOT PIGPIOD_LIB)
    message(FATAL_ERROR "libpigpiod_if2 not found. Install pigpio (daemon client).")
  endif()

  add_px4_ratecontrol_target()
  add_balancer_common_target(
    PI_OPTIMIZED
    SDL_LIBS "${SDL2_LIBS}"
    PIGPIOD_LIB "${PIGPIOD_LIB}"
  )

  add_executable(imu_demo
    ${CMAKE_SOURCE_DIR}/src/balance_controller/extras/imu_demo.cpp
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
    ${CMAKE_SOURCE_DIR}/src/balance_controller/main.cpp
  )
  
  target_link_libraries(balancer_pi PRIVATE
    balancer_common
    pigpiod_if2
    ${SDL2_LIBS}
    Threads::Threads
  )
  enable_project_warnings(balancer_pi)
  enable_pi_optimization(balancer_pi)
  
  target_include_directories(balancer_pi PRIVATE
    ${CMAKE_SOURCE_DIR}/src/balance_controller
    ${SDL2_INCS}
  )
  
  # Install to system (optional)
  install(TARGETS balancer_pi DESTINATION bin)
endif()
