cmake_minimum_required(VERSION 3.18)
project(actuator_control VERSION 0.2.0 LANGUAGES CXX)

include(CTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)

add_library(actuator_control_native
    src/actuator_bus.cpp
    src/can_socket.cpp
    src/erob_bus.cpp
    src/robstride_bus.cpp
    src/robstride_table.cpp
    src/sito_bus.cpp
)
target_include_directories(actuator_control_native PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(actuator_control_native PRIVATE Threads::Threads)
set_target_properties(actuator_control_native PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(_actuator_control python/bindings.cpp)
target_link_libraries(_actuator_control PRIVATE actuator_control_native Threads::Threads)

install(
    TARGETS _actuator_control
    LIBRARY DESTINATION actuator_control
    RUNTIME DESTINATION actuator_control
)

if(BUILD_TESTING)
    add_subdirectory(tests/cpp)
endif()
