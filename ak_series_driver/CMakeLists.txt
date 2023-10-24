cmake_minimum_required(VERSION 3.8)
project(ak_series_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/test_node.cpp
 

)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)

ament_target_dependencies(
  ${PROJECT_NAME}
  PUBLIC
  rclcpp
  
)

add_executable(testCode src/test_node.cpp)
ament_target_dependencies(testCode rclcpp)

install(
  TARGETS 
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/ak_series_driver/
  DESTINATION include/ak_series_driver/
)

install(
  TARGETS 
  testCode
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp 
)
ament_export_include_directories(include)

ament_package()
