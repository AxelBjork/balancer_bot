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
set(DOC_SRC ${CMAKE_SOURCE_DIR}/generator/generate_balancer_docs.cpp)
set(DOC_BIN ${CMAKE_BINARY_DIR}/generate_balancer_docs)
set(DOC_OUT ${CMAKE_SOURCE_DIR}/doc/ipc/protocol.md)
set(DOC_DOT ${CMAKE_SOURCE_DIR}/doc/ipc/ipc_flow.dot)
set(DOC_SVG ${CMAKE_SOURCE_DIR}/doc/ipc/ipc_flow.svg)
set(DOC_PNG ${CMAKE_SOURCE_DIR}/doc/ipc/ipc_flow.png)

add_custom_command(
    OUTPUT ${GEN_BIN}
    COMMAND ${REFLECT_CXX}
        -std=c++26 -freflection
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

add_custom_command(
    OUTPUT ${DOC_BIN}
    COMMAND ${REFLECT_CXX}
        -std=c++26 -freflection -DREFLECT_DOCS
        -I${CMAKE_SOURCE_DIR}/messages
        -I${CMAKE_SOURCE_DIR}/src/ipc
        -L/usr/local/gcc-trunk/lib64
        -Wl,-rpath,/usr/local/gcc-trunk/lib64
        -o ${DOC_BIN}
        ${DOC_SRC}
    DEPENDS ${DOC_SRC}
            ${CMAKE_SOURCE_DIR}/messages/balancer_msgs.h
    COMMENT "Compiling balancer docs generator with C++26 reflection..."
    VERBATIM
)

add_custom_command(
    OUTPUT ${DOC_OUT} ${DOC_DOT} ${DOC_SVG} ${DOC_PNG}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_SOURCE_DIR}/doc/ipc
    COMMAND ${DOC_BIN} ${DOC_DOT} > ${DOC_OUT}
    COMMAND dot -Tsvg ${DOC_DOT} -o ${DOC_SVG}
    COMMAND dot -Tpng ${DOC_DOT} -o ${DOC_PNG}
    DEPENDS ${DOC_BIN}
    COMMENT "Generating Markdown IPC docs for balancer_bot..."
    VERBATIM
)

add_custom_target(balancer_docs
    DEPENDS ${DOC_OUT} ${DOC_DOT} ${DOC_SVG} ${DOC_PNG}
)
