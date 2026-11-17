cmake_minimum_required(VERSION 2.8.3)
project(pv_lio_plus)

# SET(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_CXX_FLAGS "-std=c++14 -O0 -ggdb")
SET(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3 -pthread -Wall")

set(PV_LIO_PLUS_OUTPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../output" CACHE PATH "Output root for this build")
add_definitions(-DROOT_DIR=\"${PV_LIO_PLUS_OUTPUT_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(Boost REQUIRED COMPONENTS timer)

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(PythonLibs REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  livox_ros_driver
  eigen_conversions
)

find_package(Eigen3 REQUIRED)
get_target_property(PV_LIO_PLUS_EIGEN_INCLUDE_DIRS Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
if(NOT PV_LIO_PLUS_EIGEN_INCLUDE_DIRS)
  message(FATAL_ERROR "Eigen3::Eigen does not expose its include directory")
endif()
find_package(PCL 1.8 REQUIRED)
#find_package(GTSAM REQUIRED QUIET)

message(STATUS "Selected Eigen include directories: ${PV_LIO_PLUS_EIGEN_INCLUDE_DIRS}")

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PV_LIO_PLUS_EIGEN_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  include)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs livox_ros_driver
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS
)

add_library(pv_lio_plus_hvox_native STATIC
  include/map_manager/native/hvox/VoxelMap.cpp
  include/map_manager/native/hvox/ProbabilisticKernelOptimizer.cpp
  src/map_manager/hvox_observation.cpp)
target_include_directories(pv_lio_plus_hvox_native BEFORE PRIVATE ${PV_LIO_PLUS_EIGEN_INCLUDE_DIRS})
target_link_libraries(pv_lio_plus_hvox_native ${catkin_LIBRARIES})

add_library(pv_lio_plus_i_octree_native STATIC include/map_manager/native/i_octree/i_octree.cpp)
target_include_directories(pv_lio_plus_i_octree_native BEFORE PRIVATE ${PV_LIO_PLUS_EIGEN_INCLUDE_DIRS})
target_compile_options(pv_lio_plus_i_octree_native PRIVATE
  -fno-fast-math -fno-associative-math -ffp-contract=off -fno-lto)
set_target_properties(pv_lio_plus_i_octree_native PROPERTIES INTERPROCEDURAL_OPTIMIZATION FALSE)

set(PV_LIO_PLUS_MAP_MANAGER_ADAPTER_SOURCES
  # Compile Eigen/SVD native code per consumer to keep assertion macros consistent.
  include/map_manager/native/bievr/voxel_map.cpp
  src/map_manager/backends/bievr_backend.cpp
  src/observation/measurement.cpp
  include/map_manager/native/ig_lio/voxel_map.cpp
  src/map_manager/backends/i_octree_backend.cpp
  src/map_manager/backends/ig_lio_backend.cpp
  src/map_manager/point_plane.cpp)

add_executable(pv_lio_plus_node
  src/lio_main.cpp
  src/mapping.cpp
  src/utils.cpp
  src/preprocess.cpp
  src/imu_processing.cpp
  src/publish_subscribe.cpp
  ${PV_LIO_PLUS_MAP_MANAGER_ADAPTER_SOURCES})
add_dependencies(pv_lio_plus_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pv_lio_plus_node pv_lio_plus_hvox_native pv_lio_plus_i_octree_native ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES})
target_include_directories(pv_lio_plus_node PRIVATE ${PYTHON_INCLUDE_DIRS})

if(CATKIN_ENABLE_TESTING)
  option(PV_LIO_PLUS_ENABLE_ASAN_TESTS "Build the R-VoxelMap AddressSanitizer test" OFF)
  add_test(NAME pv_lio_plus_config_profiles
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/config_profiles_tests.py)
  catkin_add_gtest(pv_lio_plus_observation_tests tests/observation_tests.cpp)
  catkin_add_gtest(pv_lio_plus_solid_time_tests tests/solid_time_tests.cpp src/preprocess.cpp)
  if(TARGET pv_lio_plus_solid_time_tests)
    target_include_directories(pv_lio_plus_solid_time_tests BEFORE PRIVATE ${PV_LIO_PLUS_EIGEN_INCLUDE_DIRS})
    add_dependencies(pv_lio_plus_solid_time_tests ${catkin_EXPORTED_TARGETS})
    target_link_libraries(pv_lio_plus_solid_time_tests ${catkin_LIBRARIES} ${PCL_LIBRARIES})
  endif()
  catkin_add_gtest(pv_lio_plus_bievr_map_tests tests/bievr_map_tests.cpp
    include/map_manager/native/bievr/voxel_map.cpp)
  catkin_add_gtest(pv_lio_plus_bievr_observation_tests tests/bievr_observation_tests.cpp src/observation/measurement.cpp)
  catkin_add_gtest(pv_lio_plus_weight_model_tests tests/weight_model_tests.cpp src/observation/measurement.cpp)
  foreach(bievr_test pv_lio_plus_bievr_map_tests pv_lio_plus_bievr_observation_tests pv_lio_plus_weight_model_tests)
    if(TARGET ${bievr_test})
      target_include_directories(${bievr_test} BEFORE PRIVATE ${PV_LIO_PLUS_EIGEN_INCLUDE_DIRS})
      target_compile_options(${bievr_test} PRIVATE -UNDEBUG -UEIGEN_NO_DEBUG)
      target_link_libraries(${bievr_test} ${catkin_LIBRARIES})
    endif()
  endforeach()
  catkin_add_gtest(pv_lio_plus_map_contract_tests tests/map_contract_tests.cpp ${PV_LIO_PLUS_MAP_MANAGER_ADAPTER_SOURCES})
  catkin_add_gtest(pv_lio_plus_r_voxelmap_tests tests/r_voxelmap_tests.cpp src/utils.cpp src/preprocess.cpp
    ${PV_LIO_PLUS_MAP_MANAGER_ADAPTER_SOURCES})
  catkin_add_gtest(pv_lio_plus_hvox_map_tests tests/hvox_map_tests.cpp ${PV_LIO_PLUS_MAP_MANAGER_ADAPTER_SOURCES})
  catkin_add_gtest(pv_lio_plus_hvox_observation_tests tests/hvox_observation_tests.cpp)
  catkin_add_gtest(pv_lio_plus_i_octree_tests tests/i_octree_tests.cpp ${PV_LIO_PLUS_MAP_MANAGER_ADAPTER_SOURCES})
  catkin_add_gtest(pv_lio_plus_ig_lio_tests tests/ig_lio_tests.cpp ${PV_LIO_PLUS_MAP_MANAGER_ADAPTER_SOURCES})
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/i_octree_numeric_oracle_tests.cpp)
    catkin_add_gtest(pv_lio_plus_i_octree_numeric_oracle_tests tests/i_octree_numeric_oracle_tests.cpp)
    if(TARGET pv_lio_plus_i_octree_numeric_oracle_tests)
      target_compile_options(pv_lio_plus_i_octree_numeric_oracle_tests PRIVATE
        -fno-fast-math -fno-associative-math -ffp-contract=off -fno-lto)
      target_link_libraries(pv_lio_plus_i_octree_numeric_oracle_tests pv_lio_plus_i_octree_native ${catkin_LIBRARIES})
    endif()
  endif()
  if(PV_LIO_PLUS_ENABLE_ASAN_TESTS)
    catkin_add_gtest(pv_lio_plus_r_voxelmap_asan_tests tests/r_voxelmap_asan_tests.cpp)
    add_library(pv_lio_plus_i_octree_asan_native STATIC include/map_manager/native/i_octree/i_octree.cpp)
    target_include_directories(pv_lio_plus_i_octree_asan_native BEFORE PRIVATE ${PV_LIO_PLUS_EIGEN_INCLUDE_DIRS})
    target_compile_options(pv_lio_plus_i_octree_asan_native PRIVATE -O1 -UNDEBUG
      -fsanitize=address,undefined -fno-omit-frame-pointer
      -fno-fast-math -fno-associative-math -ffp-contract=off -fno-lto)
    set_target_properties(pv_lio_plus_i_octree_asan_native PROPERTIES INTERPROCEDURAL_OPTIMIZATION FALSE)
    catkin_add_gtest(pv_lio_plus_i_octree_asan_tests tests/i_octree_tests.cpp)
  endif()
  if(TARGET pv_lio_plus_observation_tests)
    target_include_directories(pv_lio_plus_observation_tests BEFORE PRIVATE ${PV_LIO_PLUS_EIGEN_INCLUDE_DIRS})
  endif()
  if(TARGET pv_lio_plus_map_contract_tests)
    # Native map headers define non-inline functions; the test includes the
    # manager implementation in one translation unit, just as mapping.cpp does.
    target_include_directories(pv_lio_plus_map_contract_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    add_dependencies(pv_lio_plus_map_contract_tests ${catkin_EXPORTED_TARGETS})
    target_link_libraries(pv_lio_plus_map_contract_tests pv_lio_plus_hvox_native pv_lio_plus_i_octree_native
      ${catkin_LIBRARIES} ${PCL_LIBRARIES})
  endif()
  if(TARGET pv_lio_plus_r_voxelmap_tests)
    target_include_directories(pv_lio_plus_r_voxelmap_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    add_dependencies(pv_lio_plus_r_voxelmap_tests ${catkin_EXPORTED_TARGETS})
    # Keep Eigen bounds assertions active for degenerate and rebuild tests.
    target_compile_options(pv_lio_plus_r_voxelmap_tests PRIVATE -UNDEBUG -UEIGEN_NO_DEBUG)
    target_link_libraries(pv_lio_plus_r_voxelmap_tests pv_lio_plus_hvox_native pv_lio_plus_i_octree_native
      ${catkin_LIBRARIES} ${PCL_LIBRARIES})
  endif()
  if(TARGET pv_lio_plus_hvox_map_tests)
    target_include_directories(pv_lio_plus_hvox_map_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    add_dependencies(pv_lio_plus_hvox_map_tests ${catkin_EXPORTED_TARGETS})
    target_compile_options(pv_lio_plus_hvox_map_tests PRIVATE -UNDEBUG -UEIGEN_NO_DEBUG)
    target_link_libraries(pv_lio_plus_hvox_map_tests pv_lio_plus_hvox_native pv_lio_plus_i_octree_native
      ${catkin_LIBRARIES} ${PCL_LIBRARIES})
  endif()
  if(TARGET pv_lio_plus_i_octree_tests)
    target_include_directories(pv_lio_plus_i_octree_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    add_dependencies(pv_lio_plus_i_octree_tests ${catkin_EXPORTED_TARGETS})
    target_compile_options(pv_lio_plus_i_octree_tests PRIVATE -UNDEBUG -UEIGEN_NO_DEBUG)
    target_link_libraries(pv_lio_plus_i_octree_tests pv_lio_plus_hvox_native pv_lio_plus_i_octree_native
      ${catkin_LIBRARIES} ${PCL_LIBRARIES})
  endif()
  if(TARGET pv_lio_plus_ig_lio_tests)
    target_include_directories(pv_lio_plus_ig_lio_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    add_dependencies(pv_lio_plus_ig_lio_tests ${catkin_EXPORTED_TARGETS})
    target_compile_options(pv_lio_plus_ig_lio_tests PRIVATE -UNDEBUG -UEIGEN_NO_DEBUG)
    target_link_libraries(pv_lio_plus_ig_lio_tests pv_lio_plus_hvox_native pv_lio_plus_i_octree_native
      ${catkin_LIBRARIES} ${PCL_LIBRARIES})
  endif()
  if(TARGET pv_lio_plus_i_octree_asan_tests)
    target_compile_definitions(pv_lio_plus_i_octree_asan_tests PRIVATE I_OCTREE_NATIVE_ONLY)
    target_compile_options(pv_lio_plus_i_octree_asan_tests PRIVATE -O1 -UNDEBUG
      -fsanitize=address,undefined -fno-omit-frame-pointer
      -fno-fast-math -fno-associative-math -ffp-contract=off -fno-lto)
    set_target_properties(pv_lio_plus_i_octree_asan_tests PROPERTIES LINK_FLAGS "-fsanitize=address,undefined" INTERPROCEDURAL_OPTIMIZATION FALSE)
    target_link_libraries(pv_lio_plus_i_octree_asan_tests pv_lio_plus_i_octree_asan_native ${catkin_LIBRARIES})
  endif()
  if(TARGET pv_lio_plus_hvox_observation_tests)
    target_include_directories(pv_lio_plus_hvox_observation_tests BEFORE PRIVATE ${PV_LIO_PLUS_EIGEN_INCLUDE_DIRS})
    target_link_libraries(pv_lio_plus_hvox_observation_tests pv_lio_plus_hvox_native ${catkin_LIBRARIES})
  endif()
  if(TARGET pv_lio_plus_r_voxelmap_asan_tests)
    target_include_directories(pv_lio_plus_r_voxelmap_asan_tests BEFORE PRIVATE ${PV_LIO_PLUS_EIGEN_INCLUDE_DIRS})
    target_compile_options(pv_lio_plus_r_voxelmap_asan_tests PRIVATE
      -O1 -UNDEBUG -UEIGEN_NO_DEBUG -fsanitize=address -fno-omit-frame-pointer)
    set_target_properties(pv_lio_plus_r_voxelmap_asan_tests PROPERTIES LINK_FLAGS "-fsanitize=address")
    target_link_libraries(pv_lio_plus_r_voxelmap_asan_tests ${catkin_LIBRARIES})
  endif()
endif()
