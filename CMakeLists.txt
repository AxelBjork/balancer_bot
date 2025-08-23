cmake_minimum_required(VERSION 3.18)
project(MotorQuickBuild LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Options: build the quick test by default; app is opt-in and *hard* requires libs.
option(BUILD_TESTS "Build motor_controller_test with stubbed pigpio + SDL2" ON)

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
#                       Tests (default ON)
# ================================================================
if (BUILD_TESTS)
  include(CTest)
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
  add_executable(motor_controller_test
    ${CMAKE_SOURCE_DIR}/external/src/modules/balance_controller/motor_controller_test.cpp
  )

  # Use pigpio stub (inline impl) and include your src for headers under test
  target_compile_definitions(motor_controller_test PRIVATE PIGPIOD_STUB_IMPL)
  target_include_directories(motor_controller_test PRIVATE
    ${CMAKE_SOURCE_DIR}/external/src/modules/balance_controller/stubs
    ${SDL2_INCS}
  )
  target_link_libraries(motor_controller_test PRIVATE
    GTest::gmock_main GTest::gtest
    Threads::Threads
    ${SDL2_LIBS}
  )

  add_test(NAME motor_controller_test COMMAND motor_controller_test)

# ================================================================
#                       App (opt-in, hard checks)
# ================================================================
else()
  if (NOT HAVE_SDL2)
    message(FATAL_ERROR "SDL2 not found. Install libsdl2-dev.")
  endif()

  # Require libpigpiod_if2 for the app
  find_library(PIGPIOD_LIB pigpiod_if2)
  if (NOT PIGPIOD_LIB)
    message(FATAL_ERROR "libpigpiod_if2 not found. Install pigpio (daemon client).")
  endif()

  add_executable(motor_controller
    ${CMAKE_SOURCE_DIR}/src/motor_controller.cpp
  )
  target_include_directories(motor_controller PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${SDL2_INCS}
  )
  target_link_libraries(motor_controller PRIVATE
    Threads::Threads
    ${SDL2_LIBS}
    ${PIGPIOD_LIB}
  )
  # Optional librt if present (Linux)
  find_library(RT_LIB rt)
  if (RT_LIB)
    target_link_libraries(motor_controller PRIVATE ${RT_LIB})
  endif()
endif()
