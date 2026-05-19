cmake_minimum_required(VERSION 3.15)

project(trossen_vr VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Find packages
find_package(libtrossen_arm REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(nlohmann_json 3.2 REQUIRED)
find_package(Threads REQUIRED)

# Static library
add_library(trossen_vr_lib STATIC
  src/network_manager.cpp
  src/teleop.cpp
)
target_include_directories(trossen_vr_lib PUBLIC include)
target_link_libraries(trossen_vr_lib PUBLIC
  Eigen3::Eigen
  nlohmann_json::nlohmann_json
  Threads::Threads
)

# Dual-arm teleop (event-driven)
add_executable(dual_arm_teleop demos/cpp/dual_arm_teleop.cpp)
target_link_libraries(dual_arm_teleop PRIVATE
  trossen_vr_lib
  libtrossen_arm
)

# Manual polling teleop
add_executable(manual_polling_teleop demos/cpp/manual_polling_teleop.cpp)
target_link_libraries(manual_polling_teleop PRIVATE
  trossen_vr_lib
  libtrossen_arm
)

# Python bindings
find_package(Python REQUIRED COMPONENTS Interpreter Development)
if(NOT pybind11_DIR)
  execute_process(
    COMMAND ${Python_EXECUTABLE} -c "import pybind11; print(pybind11.get_cmake_dir())"
    OUTPUT_VARIABLE pybind11_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
endif()
find_package(pybind11 CONFIG REQUIRED)
pybind11_add_module(trossen_vr MODULE src/bindings.cpp)
target_link_libraries(trossen_vr PRIVATE trossen_vr_lib)

install(TARGETS trossen_vr LIBRARY DESTINATION ${Python_SITEARCH})