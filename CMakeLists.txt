cmake_minimum_required(VERSION 3.10)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

project(swri_console)

### ROS PACKAGES ###
find_package(ament_cmake REQUIRED)
# find_package(rosbag_storage REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)

### QT PACKAGES ###
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets REQUIRED)

find_package(Boost COMPONENTS chrono thread REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(include)

set(UI_FILES
  ui/console_window.ui)
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
  )
qt5_add_resources(RCC_SRCS resources/images.qrc)
qt5_wrap_ui(SRC_FILES ${UI_FILES})
qt5_wrap_cpp(SRC_FILES ${HEADER_FILES})

add_executable(swri_console ${HEADER_FILES} ${SRC_FILES} ${RCC_SRCS} src/main.cpp)
target_link_libraries(swri_console
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
  Boost::chrono
  Boost::thread
)

ament_target_dependencies(${PROJECT_NAME}
  ament_cmake
  rclcpp
  rcl_interfaces
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_export_dependencies(${RUNTIME_DEPS})
ament_package()
