cmake_minimum_required(VERSION 3.8)
project(web_control_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Qt5 설정
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(humanoid_interfaces REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Network)

include_directories(include)

# ===== Receiver Node =====
add_executable(receiver src/receiver/receiver.cpp)
ament_target_dependencies(receiver 
  rclcpp 
  std_msgs 
  humanoid_interfaces
)
target_link_libraries(receiver
  Qt5::Core
  Qt5::Network
)

# ===== Sender Node =====
add_executable(sender src/sender/sender.cpp)
ament_target_dependencies(sender 
  rclcpp 
  std_msgs 
  humanoid_interfaces
)
target_link_libraries(sender
  Qt5::Core
  Qt5::Network
)

# ===== Install Targets =====
install(TARGETS
  receiver
  sender
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  include/
  DESTINATION include/
)

# ===== Lint / Test =====
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
