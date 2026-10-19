cmake_minimum_required(VERSION 3.16)
project(ird_ethercat_hardware_interface LANGUAGES C CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
foreach(dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

# ---------------------------------------------------------------------------
# SOEM (Simple Open EtherCAT Master), version 2.x
#
# 1. If SOEM is installed (e.g. built from source and installed to /usr/local,
#    see README.md), its exported CMake package provides the `soem` target.
# 2. Otherwise, and unless IRD_FETCH_SOEM is OFF, SOEM is downloaded at
#    configure time (pinned tag below) and built as part of this package.
# ---------------------------------------------------------------------------
set(IRD_SOEM_GIT_TAG "v2.0.0" CACHE STRING "SOEM tag downloaded when SOEM is not installed")
option(IRD_FETCH_SOEM "Download and build SOEM if it is not installed" ON)

find_package(soem CONFIG QUIET)
if(TARGET soem)
  message(STATUS "Using installed SOEM: ${soem_DIR}")
elseif(IRD_FETCH_SOEM)
  message(STATUS "SOEM not found: downloading SOEM ${IRD_SOEM_GIT_TAG}")
  include(FetchContent)
  # SOEM is a static library linked into the shared plugin below.
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)
  set(SOEM_BUILD_SAMPLES OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(soem
    GIT_REPOSITORY https://github.com/OpenEtherCATsociety/SOEM.git
    GIT_TAG ${IRD_SOEM_GIT_TAG}
  )
  FetchContent_MakeAvailable(soem)
else()
  message(FATAL_ERROR "SOEM 2.x not found. Install it (see README.md) or configure with -DIRD_FETCH_SOEM=ON.")
endif()

# ---------------------------------------------------------------------------
# Core library: configuration, PDO layout, startup SDO plan and the per-cycle
# enable/fault-recovery logic. No ROS or SOEM dependency, so it is unit-tested
# on its own (see test/).
# ---------------------------------------------------------------------------
add_library(ird_ethercat_core STATIC
  src/config.cpp
  src/config_loader.cpp
  src/cyclic_controller.cpp
  src/pdo.cpp
  src/startup_plan.cpp
)
set_target_properties(ird_ethercat_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(ird_ethercat_core PUBLIC cxx_std_17)
target_include_directories(ird_ethercat_core PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)
target_link_libraries(ird_ethercat_core PRIVATE yaml-cpp)

# ---------------------------------------------------------------------------
# Hardware interface plugin
# ---------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/ethercat_master.cpp
  src/ird_ethercat_system.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PRIVATE ird_ethercat_core soem)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(${PROJECT_NAME} PRIVATE "IRD_ETHERCAT_HARDWARE_INTERFACE_BUILDING_DLL")

pluginlib_export_plugin_description_file(hardware_interface ird_ethercat_hardware_interface.xml)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ---------------------------------------------------------------------------
# Unit tests (no hardware needed): colcon test --packages-select ird_ethercat_hardware_interface
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ird_ethercat_core
    test/test_config.cpp
    test/test_config_loader.cpp
    test/test_pdo.cpp
    test/test_startup_plan.cpp
    test/test_cia402.cpp
    test/test_dc_sync.cpp
    test/test_cyclic_controller.cpp
  )
  target_link_libraries(test_ird_ethercat_core ird_ethercat_core)
  target_compile_definitions(test_ird_ethercat_core PRIVATE
    "IRD_CONFIG_DIR=\"${PROJECT_SOURCE_DIR}/config\"")
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
