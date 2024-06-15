cmake_minimum_required(VERSION 3.8)
project(calib_correspondences)

set(CMAKE_BUILD_TYPE "Release")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(message_filters REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(YAML-cpp REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)


add_executable(image_cloud_reader
  src/auxiliar.cpp
  src/ImageCloud2Correspondences.cpp
  src/Parameters.cpp
  src/ProcessSensorData.cpp)

ament_target_dependencies(
  image_cloud_reader 
  rclcpp 
  std_msgs 
  sensor_msgs 
  custom_msgs 
  geometry_msgs 
  tf2_eigen
  message_filters
  Boost
  cv_bridge
  OpenCV
  YAML-cpp)

add_executable(corr_subscriber
  src/auxiliar.cpp
  src/Calibration.cpp
  src/CorrespondencesSub.cpp
  src/Parameters.cpp)

ament_target_dependencies(
  corr_subscriber 
  rclcpp 
  std_msgs 
  sensor_msgs 
  custom_msgs 
  geometry_msgs 
  tf2_eigen
  message_filters
  Boost
  YAML-cpp)

target_link_libraries(image_cloud_reader ${PCL_LIBRARIES} Boost::thread ${YAML_CPP_LIBRARIES})
target_link_libraries(corr_subscriber ${YAML_CPP_LIBRARIES})

install(TARGETS
  image_cloud_reader
  corr_subscriber
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/)
  
ament_package()