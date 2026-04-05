# cmake/reflection.cmake — Isolates C++26 reflection targets.
#
# Only targets in this file use gcc-trunk with -freflection.
# The rest of the project compiles with the standard C++20 compiler.

set(REFLECT_CXX /usr/local/gcc-trunk/bin/g++)

# ── generate_balancer_bindings (compile-time reflection) ─────────────────────
# We use add_custom_command to invoke gcc-trunk DIRECTLY instead of
# add_executable, because CMake does not support mixing compilers in one project.

set(GEN_SRC ${CMAKE_SOURCE_DIR}/generator/generate_balancer_bindings.cpp)
set(GEN_BIN ${CMAKE_BINARY_DIR}/generate_balancer_bindings)
set(GEN_OUT ${CMAKE_SOURCE_DIR}/tests/python/generated_balancer.py)

add_custom_command(
    OUTPUT ${GEN_BIN}
    COMMAND ${REFLECT_CXX}
        -std=c++26 -freflection
        -I${CMAKE_SOURCE_DIR}/reflect_pytest/messages
        -I${CMAKE_SOURCE_DIR}/messages
        -L/usr/local/gcc-trunk/lib64
        -Wl,-rpath,/usr/local/gcc-trunk/lib64
        -o ${GEN_BIN}
        ${GEN_SRC}
    DEPENDS ${GEN_SRC}
            ${CMAKE_SOURCE_DIR}/messages/balancer_msgs.h
    COMMENT "Compiling balancer bindings generator with C++26 reflection..."
    VERBATIM
)

add_custom_command(
    OUTPUT ${GEN_OUT}
    COMMAND ${GEN_BIN} > ${GEN_OUT}
    DEPENDS ${GEN_BIN}
    COMMENT "Generating Python bindings for balancer_bot payloads..."
    VERBATIM
)

add_custom_target(balancer_bindings
    DEPENDS ${GEN_OUT}
)
