cmake_minimum_required(VERSION 3.22.1)

project("starsos")

add_library(
    starsjni
    SHARED
    bridge.c
)

# 链接 Rust 编译出来的 .so
add_library(quantum_core SHARED IMPORTED)
set_target_properties(quantum_core PROPERTIES
    IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/../jniLibs/${ANDROID_ABI}/libquantum_core.so"
)

target_link_libraries(starsjni
    quantum_core
    log
)
