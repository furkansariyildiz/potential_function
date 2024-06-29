cmake_minimum_required(VERSION 3.8)
project(potential_function)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Add MPFR and GMP manually
include_directories(include ${BOOST_INCLUDE_DIRS} /usr/include /usr/include/boost/multiprecision)
link_directories(/usr/lib /usr/local/lib)

add_library(pid_controller src/pid_controller.cpp)

rosidl_generate_interfaces(${PROJECT_NAME}_msgs
  "msg/RobotPosition.msg"
  "msg/RobotsPositionList.msg"
  "msg/TargetPose.msg"
  "msg/TargetPoseList.msg"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs nav_msgs
)

add_executable(potential_function src/potential_function.cpp)
target_link_libraries(potential_function ${catkin_LIBRARIES} pid_controller mpfr gmp m) # m kütüphanesi de eklenmeli

ament_target_dependencies(potential_function rclcpp std_msgs nav_msgs tf2)
ament_export_targets(potential_function HAS_LIBRARY_TARGET)

ament_python_install_package(scripts)

install(TARGETS
        potential_function
        pid_controller
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
        launch
        config
        DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
        scripts/target_pose_list_publisher.py
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/potential_function
        DESTINATION include
)

install(
        TARGETS potential_function
        EXPORT potential_function
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

# This is for Galactic or Foxy 
rosidl_target_interfaces(potential_function ${PROJECT_NAME}_msgs "rosidl_typesupport_cpp")

# This is for only Humble (rosidl_get_typesupport_target) 
# ROS2 Humble
# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp") 
# target_link_libraries(potential_function "${cpp_typesupport_target}")

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
ament_package()
