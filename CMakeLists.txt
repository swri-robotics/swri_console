cmake_minimum_required(VERSION 3.10)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

project(swri_console)

### ROS PACKAGES ###
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_transport REQUIRED)

### QT PACKAGES ###
# Prefer Qt5. This is because jazzy and kilted have both Qt5
# and Qt6 installed, but Rqt is build against Qt5. Lyrical does
# not have Qt5, so it will build against Qt6 in that case
find_package(Qt5 QUIET COMPONENTS Core Gui Widgets)
if(Qt5_FOUND)
  set(QT_VERSION_MAJOR 5)
else()
  find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets)
  set(QT_VERSION_MAJOR 6)
endif()
message(STATUS "Building swri_console against Qt${QT_VERSION_MAJOR}")

set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(include)

set(UI_FILES
  ui/console_window.ui
)
set(HEADER_FILES
  include/swri_console/bag_reader.h
  include/swri_console/console_master.h
  include/swri_console/console_window.h
  include/swri_console/log_database.h
  include/swri_console/log_database_proxy_model.h
  include/swri_console/node_click_handler.h
  include/swri_console/node_list_model.h
  include/swri_console/rosout_log_loader.h
  include/swri_console/ros_thread.h
  include/swri_console/unix_signal_handler.h
  )

set(SRC_FILES
  src/bag_reader.cpp
  src/console_master.cpp
  src/console_window.cpp
  src/log_database.cpp
  src/log_database_proxy_model.cpp
  src/node_click_handler.cpp
  src/node_list_model.cpp
  src/rosout_log_loader.cpp
  src/ros_thread.cpp
  src/settings_keys.cpp
  src/unix_signal_handler.cpp
  )

qt_add_resources(RCC_SRCS resources/images.qrc)
qt_wrap_ui(SRC_FILES ${UI_FILES})
qt_wrap_cpp(SRC_FILES ${HEADER_FILES})

add_executable(swri_console ${HEADER_FILES} ${SRC_FILES} ${RCC_SRCS} src/main.cpp)

target_link_libraries(swri_console PUBLIC
  Qt${QT_VERSION_MAJOR}::Core
  Qt${QT_VERSION_MAJOR}::Gui
  Qt${QT_VERSION_MAJOR}::Widgets
  ${rcl_interfaces_TARGETS}
  rclcpp::rclcpp
  rcutils::rcutils
  ${rosbag2_storage_TARGETS}
  ${rosbag2_transport_TARGETS}
  rmw::rmw
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
  ament_cmake
  Qt${QT_VERSION_MAJOR}Core
  Qt${QT_VERSION_MAJOR}Gui
  Qt${QT_VERSION_MAJOR}Widgets
  rclcpp
  rcl_interfaces
  rcutils
  rosbag2_storage
  rosbag2_transport
  rmw)
ament_package()
