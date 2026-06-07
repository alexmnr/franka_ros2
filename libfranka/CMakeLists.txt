cmake_minimum_required(VERSION 3.8)
project(libfranka
  VERSION 0.7.1
  LANGUAGES CXX
)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(MSVC)
  add_compile_options(/W0)
else()
  add_compile_options(-Wall -Wextra)
endif()

## Dependencies
find_package(Poco REQUIRED COMPONENTS Net Foundation)
find_package(Eigen3 REQUIRED)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

## Library
add_library(franka SHARED
  src/control_loop.cpp
  src/control_tools.cpp
  src/control_types.cpp
  src/duration.cpp
  src/errors.cpp
  src/exception.cpp
  src/gripper.cpp
  src/gripper_state.cpp
  src/library_downloader.cpp
  src/library_loader.cpp
  src/load_calculations.cpp
  src/log.cpp
  src/logger.cpp
  src/lowpass_filter.cpp
  src/model.cpp
  src/model_library.cpp
  src/network.cpp
  src/rate_limiting.cpp
  src/robot.cpp
  src/robot_impl.cpp
  src/robot_state.cpp
)
add_library(Franka::Franka ALIAS franka)

set_target_properties(franka PROPERTIES
  WINDOWS_EXPORT_ALL_SYMBOLS ON
  VERSION ${libfranka_VERSION}
  SOVERSION ${libfranka_VERSION_MAJOR}.${libfranka_VERSION_MINOR}
  EXPORT_NAME Franka
)

target_compile_features(franka INTERFACE
  cxx_attribute_deprecated
  cxx_constexpr
  cxx_defaulted_functions
  cxx_deleted_functions
  cxx_generalized_initializers
  cxx_noexcept
  cxx_uniform_initialization
)

if(MSVC)
  target_compile_definitions(franka PUBLIC
    _USE_MATH_DEFINES # for M_PI in cmath
    NOMINMAX # avoid conflict with std::min
  )
endif()

target_include_directories(franka PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(franka PRIVATE
  Poco::Foundation
  Poco::Net
  Eigen3::Eigen3
  Threads::Threads
)

## Installation (Required for colcon downstream linking)
include(GNUInstallDirs)
set(INSTALL_CMAKE_CONFIG_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/Franka)

install(TARGETS franka
  EXPORT FrankaTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

export(EXPORT FrankaTargets
  NAMESPACE Franka::
  FILE ${CMAKE_CURRENT_BINARY_DIR}/FrankaTargets.cmake
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/FrankaConfigVersion.cmake
  COMPATIBILITY SameMajorVersion
)
configure_package_config_file(cmake/FrankaConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/FrankaConfig.cmake
  INSTALL_DESTINATION ${INSTALL_CMAKE_CONFIG_DIR}
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/FrankaConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/FrankaConfigVersion.cmake
  DESTINATION ${INSTALL_CMAKE_CONFIG_DIR}
)
install(EXPORT FrankaTargets
  NAMESPACE Franka::
  DESTINATION ${INSTALL_CMAKE_CONFIG_DIR}
)

## Subprojects (Optional Examples)
option(BUILD_EXAMPLES "Build example code" OFF)
if(BUILD_EXAMPLES)
  # Ignore find_package(Franka) in subprojects so they link to this local build.
  macro(find_package)
    if(NOT "${ARGV0}" STREQUAL Franka AND NOT DEFINED "${ARGV0}_FOUND")
      _find_package(${ARGV})
    endif()
  endmacro()
  
  add_subdirectory(examples)
endif()
