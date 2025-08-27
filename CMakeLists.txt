cmake_minimum_required(VERSION 3.16)
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
find_package(Qt5 REQUIRED COMPONENTS Core Network)
find_package(humanoid_interfaces REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/Robot1receiverMsg.msg"
  "msg/Robot1senderMsg.msg"
  "msg/Robot2receiverMsg.msg"
  "msg/Robot2senderMsg.msg"
  "msg/Robot3receiverMsg.msg"
  "msg/Robot3senderMsg.msg"
  "msg/Robot4receiverMsg.msg"
  "msg/Robot4senderMsg.msg"

  "msg/ImuflagMsg.msg"
  "msg/NodeManagerMsg.msg"
)

# 인터페이스 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs builtin_interfaces
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

include_directories(include)

# ===== Receiver Nuc Node =====
add_executable(receiver_nuc src/receiver/receiver_nuc.cpp)
ament_target_dependencies(receiver_nuc 
  rclcpp 
  std_msgs 
)

set_target_properties(receiver_nuc PROPERTIES
  AUTOMOC ON
  AUTOUIC ON
  AUTORCC ON
)

target_link_libraries(receiver_nuc
  Qt5::Core
  Qt5::Network
  "${cpp_typesupport_target}"
)
# ==============================

# ===== Receiver Master Node =====
add_executable(receiver_master src/receiver/receiver_master.cpp)
ament_target_dependencies(receiver_master 
  rclcpp 
  std_msgs 
)

set_target_properties(receiver_master PROPERTIES
  AUTOMOC ON
  AUTOUIC ON
  AUTORCC ON
)

target_link_libraries(receiver_master
  Qt5::Core
  Qt5::Network
  "${cpp_typesupport_target}"
)
# ==============================

# ===== Sender Nuc Node =====
add_executable(sender_nuc src/sender/sender_nuc.cpp)
ament_target_dependencies(sender_nuc 
  rclcpp 
  std_msgs 
  humanoid_interfaces
)

set_target_properties(sender_nuc PROPERTIES
  AUTOMOC ON
  AUTOUIC ON
  AUTORCC ON
)

target_link_libraries(sender_nuc
  Qt5::Core
  Qt5::Network
  "${cpp_typesupport_target}"
)
# ==============================

# ===== Sender Master Node =====
add_executable(sender_master src/sender/sender_master.cpp)
ament_target_dependencies(sender_master 
  rclcpp 
  std_msgs 
)

set_target_properties(sender_master PROPERTIES
  AUTOMOC ON
  AUTOUIC ON
  AUTORCC ON
)

target_link_libraries(sender_master
  Qt5::Core
  Qt5::Network
  "${cpp_typesupport_target}"
)
# ==============================

# ===== Install Targets =====
install(TARGETS
  receiver_nuc
  receiver_master
  sender_nuc
  sender_master
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  include/
  DESTINATION include/
)

install(PROGRAMS
  script/node_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

# ===== Lint / Test =====
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
