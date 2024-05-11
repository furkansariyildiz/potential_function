cmake_minimum_required(VERSION 3.8)
project(potential_function)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include ${BOOST_INCLUDE_DIRS})

rosidl_generate_interfaces(${PROJECT_NAME}_msgs
  "msg/TargetPoseList.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

add_executable(potential_function src/potential_function.cpp)
target_link_libraries(potential_function ${catkin_LIBRARIES})
ament_target_dependencies(potential_function rclcpp std_msgs)

install(TARGETS
        potential_function
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
        launch
        config
        DESTINATION share/${PROJECT_NAME}/
)

# This is for Galactic or Foxy 
rosidl_target_interfaces(potential_function ${PROJECT_NAME}_msgs "rosidl_typesupport_cpp")


# This is for only Humble (rosidl_get_typesupport_target) 
# ROS2 Humble
# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp") 
# target_link_libraries(potential_function "${cpp_typesupport_target}")

ament_export_dependencies(rosidl_default_runtime)
ament_package()
