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
find_package(nlohmann_json 3.2 REQUIRED)
find_package(Threads REQUIRED)

# Static library
add_library(trossen_vr_lib STATIC
  src/network_manager.cpp
  src/teleop.cpp
  src/vr_conversions.cpp
)
target_include_directories(trossen_vr_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(trossen_vr_lib PUBLIC
  nlohmann_json::nlohmann_json
  Threads::Threads
)

# Demo scripts
option(BUILD_DEMOS "Build C++ demo scripts" OFF)

if(BUILD_DEMOS)
  find_package(libtrossen_arm)
  if(libtrossen_arm_FOUND)
    message(STATUS "Building demo scripts...")

    # Event-driven teleop demo
    add_executable(event_driven_teleop demos/cpp/event_driven_teleop.cpp)
    target_link_libraries(event_driven_teleop PRIVATE
      trossen_vr_lib
      libtrossen_arm
    )

    # Manual polling teleop demo
    add_executable(manual_polling_teleop demos/cpp/manual_polling_teleop.cpp)
    target_link_libraries(manual_polling_teleop PRIVATE
      trossen_vr_lib
      libtrossen_arm
    )
  else()
    message(FATAL_ERROR "BUILD_DEMOS=ON but libtrossen_arm not found. Install libtrossen_arm or set BUILD_DEMOS=OFF.")
  endif()
endif()


# Python bindings
option(BUILD_PYTHON "Build Python bindings" OFF)

if(BUILD_PYTHON)
  find_package(Python REQUIRED COMPONENTS Interpreter Development)
  execute_process(
    COMMAND ${Python_EXECUTABLE} -c "import pybind11; print(pybind11.get_cmake_dir())"
    OUTPUT_VARIABLE pybind11_DIR OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE pybind11_RESULT
  )
  if(NOT pybind11_RESULT EQUAL 0)
    message(FATAL_ERROR "Failed to find pybind11. Make sure dependencies are installed: uv sync")
  endif()
  find_package(pybind11 CONFIG REQUIRED PATHS ${pybind11_DIR} NO_DEFAULT_PATH)
  pybind11_add_module(trossen_vr MODULE src/bindings.cpp)
  target_link_libraries(trossen_vr PRIVATE trossen_vr_lib)

  install(TARGETS trossen_vr LIBRARY DESTINATION ${Python_SITEARCH})
endif()

# library install & export
install(TARGETS trossen_vr_lib
  EXPORT trossen_vrTargets
  ARCHIVE DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)
install(EXPORT trossen_vrTargets
  FILE trossen_vrConfig.cmake
  NAMESPACE trossen_vr::
  DESTINATION lib/cmake/trossen_vr
)
