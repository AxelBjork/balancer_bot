# CMake toolchain file for Raspberry Pi 4 (ARM64 / aarch64)

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Cross compiler provided by the devcontainer / host system.
set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

# Where to find libraries and headers for the target system.
# If you have a Pi sysroot, prefer pointing CMAKE_SYSROOT at it.
# Without a sysroot we fall back to the host's multiarch cross packages.
set(CMAKE_LIBRARY_ARCHITECTURE aarch64-linux-gnu)
set(CMAKE_FIND_ROOT_PATH /usr/aarch64-linux-gnu /usr)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# The host cross compiler in this environment is GCC 15, which emits binaries
# requiring newer GLIBCXX symbols than a typical Raspberry Pi runtime provides.
# Statically link libstdc++ and libgcc into executables so the deployed binary
# does not depend on the Pi's system libstdc++.so.6 version.
set(BALANCER_RPI_PORTABLE_STDLIB ON CACHE BOOL
    "Statically link libstdc++/libgcc for Raspberry Pi cross builds")

if (BALANCER_RPI_PORTABLE_STDLIB)
  set(_balancer_portable_stdlib_flags "-static-libstdc++ -static-libgcc")
  if (NOT CMAKE_EXE_LINKER_FLAGS_INIT)
    set(CMAKE_EXE_LINKER_FLAGS_INIT "${_balancer_portable_stdlib_flags}")
  elseif (NOT CMAKE_EXE_LINKER_FLAGS_INIT MATCHES "(^| )-static-libstdc\\+\\+( |$)")
    string(APPEND CMAKE_EXE_LINKER_FLAGS_INIT " ${_balancer_portable_stdlib_flags}")
  endif()
endif()
