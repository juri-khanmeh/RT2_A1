cmake_minimum_required(VERSION 3.5)
project(rt2_assignment1)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
#find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
#find_package(std_msgs REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RandomPosition.srv" 
  "srv/Command.srv" 
  "srv/Position.srv"
 )
#ament_export_dependencies(rosidl_default_runtime)
 
#rosidl_generate_interfaces(${PROJECT_NAME}
 #  
# )
#rosidl_generate_interfaces(${PROJECT_NAME}
  
# )



add_library(positionS_component SHARED
  src/position_service.cpp)
  
add_library(stateM_component SHARED
  src/state_machine.cpp)
  
rclcpp_components_register_node(positionS_component
  PLUGIN "rt2_assignment1::RandomServer"
  EXECUTABLE comp_positionS)
  
rclcpp_components_register_node(stateM_component
  PLUGIN "rt2_assignment1::StateMachine"
  EXECUTABLE comp_stateM)
  
ament_target_dependencies(positionS_component rclcpp rclcpp_components)

ament_target_dependencies(stateM_component rclcpp rclcpp_components)

rosidl_target_interfaces(positionS_component
  ${PROJECT_NAME} "rosidl_typesupport_cpp")   

rosidl_target_interfaces(stateM_component
  ${PROJECT_NAME} "rosidl_typesupport_cpp")  

install(TARGETS
  positionS_component
  stateM_component
  comp_positionS
  comp_stateM
  DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib)
  
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(
  FILES mapping_rules.yaml
  DESTINATION share/${PROJECT_NAME})  
  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
