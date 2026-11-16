cmake_minimum_required(VERSION 3.16)
project(d1_host_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS Humble's Python modules belong to Ubuntu's Python, not Conda.
if(NOT DEFINED Python3_EXECUTABLE)
  set(Python3_EXECUTABLE /usr/bin/python3 CACHE FILEPATH "ROS Python interpreter")
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Threads REQUIRED)

set(D1_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/robot_sdk")
set(D1_SDK_LIB_DIR "${D1_SDK_ROOT}/lib/${CMAKE_SYSTEM_PROCESSOR}")
set(D1_SDK_LIBRARY "${D1_SDK_LIB_DIR}/librobot_sdk.so.0.2.1")
if(NOT EXISTS "${D1_SDK_LIBRARY}")
  message(FATAL_ERROR "RobotSDK 0.2.1 library missing: ${D1_SDK_LIBRARY}")
endif()

add_executable(d1_bridge src/d1_bridge.cpp)
ament_target_dependencies(d1_bridge rclcpp geometry_msgs std_msgs)
target_include_directories(d1_bridge SYSTEM PRIVATE "${D1_SDK_ROOT}/include")
target_link_libraries(d1_bridge "${D1_SDK_LIBRARY}" Threads::Threads)
set_target_properties(d1_bridge PROPERTIES
  BUILD_RPATH "${D1_SDK_LIB_DIR}"
  INSTALL_RPATH "$ORIGIN/sdk")

add_executable(teleop_keyboard src/teleop_keyboard.cpp)
ament_target_dependencies(teleop_keyboard rclcpp geometry_msgs std_msgs)
target_link_libraries(teleop_keyboard Threads::Threads)

install(TARGETS d1_bridge teleop_keyboard DESTINATION lib/${PROJECT_NAME})
install(FILES "${D1_SDK_LIBRARY}" DESTINATION lib/${PROJECT_NAME}/sdk)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
