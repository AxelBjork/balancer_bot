# cmake/reflection.cmake — Isolates C++26 reflection targets.
#
# Only targets in this file use -freflection for C++26 features.
# The rest of the project compiles with GCC 16 in C++23 mode.

if (EXISTS "/usr/local/gcc-16.1.0/bin/g++")
  set(REFLECT_CXX "/usr/local/gcc-16.1.0/bin/g++")
elseif (CMAKE_CXX_COMPILER)
  set(REFLECT_CXX ${CMAKE_CXX_COMPILER})
else()
  set(REFLECT_CXX g++)
endif()

find_program(DOT_EXECUTABLE dot)
if (NOT DOT_EXECUTABLE)
  message(FATAL_ERROR "Graphviz 'dot' is required to build balancer_docs")
endif()

# ── generate_balancer_bindings (compile-time reflection) ─────────────────────
# We use add_custom_command to invoke gcc-trunk DIRECTLY instead of
# add_executable, because CMake does not support mixing compilers in one project.

set(REFLECTION_SRC_DIR ${CMAKE_SOURCE_DIR}/src/reflection)
set(REFLECTION_COMMON_HDR ${REFLECTION_SRC_DIR}/reflection_common.h)
set(REFLECTION_REGISTRY_HDR ${REFLECTION_SRC_DIR}/balancer_message_registry.h)

set(GEN_SRC ${REFLECTION_SRC_DIR}/generate_balancer_bindings.cpp)
set(GEN_BIN ${CMAKE_BINARY_DIR}/generate_balancer_bindings)
set(GEN_OUT ${CMAKE_SOURCE_DIR}/tests/python/generated_balancer.py)
set(DOC_SRC ${REFLECTION_SRC_DIR}/generate_balancer_docs.cpp)
set(DOC_BIN ${CMAKE_BINARY_DIR}/generate_balancer_docs)
set(DOC_OUT ${CMAKE_SOURCE_DIR}/doc/ipc/protocol.md)
set(DOC_DOT ${CMAKE_SOURCE_DIR}/doc/ipc/ipc_flow.dot)
set(DOC_SVG ${CMAKE_SOURCE_DIR}/doc/ipc/ipc_flow.svg)
set(DOC_PNG ${CMAKE_SOURCE_DIR}/doc/ipc/ipc_flow.png)

add_custom_command(
    OUTPUT ${GEN_BIN}
    COMMAND ${REFLECT_CXX}
        -std=c++26 -freflection
        -I${CMAKE_SOURCE_DIR}/src/messages
        -I${CMAKE_SOURCE_DIR}/src
        -I${CMAKE_SOURCE_DIR}/src/ipc
        -L/usr/local/gcc-16.1.0/lib64
        -Wl,-rpath,/usr/local/gcc-16.1.0/lib64
        -o ${GEN_BIN}
        ${GEN_SRC}
    DEPENDS ${GEN_SRC}
            ${REFLECTION_COMMON_HDR}
            ${REFLECTION_REGISTRY_HDR}
            ${CMAKE_SOURCE_DIR}/src/messages/msg_base.h
            ${CMAKE_SOURCE_DIR}/src/messages/core_msgs.h
            ${CMAKE_SOURCE_DIR}/src/messages/balancer_msgs.h
            ${CMAKE_SOURCE_DIR}/src/ipc/udp_bridge.h
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

add_custom_command(
    OUTPUT ${DOC_BIN}
    COMMAND ${REFLECT_CXX}
        -std=c++26 -freflection -DREFLECT_DOCS
        -I${CMAKE_SOURCE_DIR}/src/messages
        -I${CMAKE_SOURCE_DIR}/src
        -I${CMAKE_SOURCE_DIR}/src/ipc
        -L/usr/local/gcc-16.1.0/lib64
        -Wl,-rpath,/usr/local/gcc-16.1.0/lib64
        -o ${DOC_BIN}
        ${DOC_SRC}
    DEPENDS ${DOC_SRC}
            ${REFLECTION_COMMON_HDR}
            ${REFLECTION_REGISTRY_HDR}
            ${CMAKE_SOURCE_DIR}/src/messages/msg_base.h
            ${CMAKE_SOURCE_DIR}/src/messages/core_msgs.h
            ${CMAKE_SOURCE_DIR}/src/messages/balancer_msgs.h
            ${CMAKE_SOURCE_DIR}/src/ipc/udp_bridge.h
            ${CMAKE_SOURCE_DIR}/src/services/imu/imu_service.h
            ${CMAKE_SOURCE_DIR}/src/services/control/control_service.h
            ${CMAKE_SOURCE_DIR}/src/services/motor/motor_service.h
            ${CMAKE_SOURCE_DIR}/src/services/time/time_service.h
    COMMENT "Compiling balancer docs generator with C++26 reflection..."
    VERBATIM
)

add_custom_command(
    OUTPUT ${DOC_OUT} ${DOC_DOT} ${DOC_SVG} ${DOC_PNG}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_SOURCE_DIR}/doc/ipc
    COMMAND ${DOC_BIN} ${DOC_DOT} > ${DOC_OUT}
    COMMAND ${DOT_EXECUTABLE} -Tsvg ${DOC_DOT} -o ${DOC_SVG}
    COMMAND ${DOT_EXECUTABLE} -Tpng ${DOC_DOT} -o ${DOC_PNG}
    DEPENDS ${DOC_BIN}
    COMMENT "Generating Markdown IPC docs for balancer_bot..."
    VERBATIM
)

add_custom_target(balancer_docs
    DEPENDS ${DOC_OUT} ${DOC_DOT} ${DOC_SVG} ${DOC_PNG}
)

add_custom_target(balancer_reflection ALL
    DEPENDS balancer_bindings balancer_docs
)
