cmake_minimum_required(VERSION 3.15)

project(trossen_vr VERSION 1.0.0 LANGUAGES CXX)

set(LIBRARY_NAME trossen_vr)

option(BUILD_DEMOS "Build example executables" OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(ARCH "x86_64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" OR
       CMAKE_SYSTEM_PROCESSOR MATCHES "arm64")
    set(ARCH "aarch64")
else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
    set(OS "macos")
elseif(CMAKE_SYSTEM_NAME MATCHES "Linux")
    set(OS "linux")
else()
    message(FATAL_ERROR "Unsupported OS: ${CMAKE_SYSTEM_NAME}")
endif()

message(STATUS "Building ${PROJECT_NAME} for ${OS}/${ARCH}")

file(GLOB_RECURSE HEADER_FILES ${PROJECT_SOURCE_DIR}/include/trossen_vr/*.hpp)
file(GLOB_RECURSE SOURCE_FILES ${PROJECT_SOURCE_DIR}/src/trossen_vr/*.cpp)
# Exclude bindings.cpp from the static library
list(FILTER SOURCE_FILES EXCLUDE REGEX ".*bindings\\.cpp$")
file(GLOB_RECURSE TEST_FILES ${PROJECT_SOURCE_DIR}/scripts/cpp/*.cpp)

# Set Position Independent Code flag before defining targets
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_library(${LIBRARY_NAME} STATIC
    ${SOURCE_FILES}
    ${HEADER_FILES}
)

target_include_directories(${LIBRARY_NAME}
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)

find_package(Threads REQUIRED)
target_link_libraries(${LIBRARY_NAME} PUBLIC Threads::Threads)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries(${LIBRARY_NAME} PUBLIC Eigen3::Eigen)

find_package(nlohmann_json REQUIRED)
target_link_libraries(${LIBRARY_NAME} PUBLIC nlohmann_json::nlohmann_json)

find_package(websocketpp REQUIRED)
target_link_libraries(${LIBRARY_NAME} PUBLIC websocketpp::websocketpp)

# Find the libtrossen_arm package
find_package(libtrossen_arm REQUIRED)
target_link_libraries(${LIBRARY_NAME} PRIVATE libtrossen_arm)

# Find pybind11
find_package(pybind11 CONFIG REQUIRED)
# Find Python with development headers
find_package(Python REQUIRED COMPONENTS Interpreter Development)
pybind11_add_module(pytrossen_vr MODULE src/trossen_vr/bindings.cpp)
target_link_libraries(pytrossen_vr PRIVATE ${LIBRARY_NAME} libtrossen_arm Python::Python)

if(BUILD_DEMOS)
    add_executable(vr_demo examples/main.cpp)
    target_include_directories(vr_demo PUBLIC ${PROJECT_SOURCE_DIR}/include)
    target_link_libraries(vr_demo PRIVATE ${LIBRARY_NAME})
endif()

add_executable(event_driven_teleop demos/cpp/event_driven_teleop.cpp)
target_link_libraries(event_driven_teleop PRIVATE ${LIBRARY_NAME} libtrossen_arm)

add_executable(manual_polling_teleop demos/cpp/manual_polling_teleop.cpp)
target_link_libraries(manual_polling_teleop PRIVATE ${LIBRARY_NAME} libtrossen_arm)


install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
        DESTINATION include)

install(TARGETS ${LIBRARY_NAME}
        ARCHIVE DESTINATION lib)

