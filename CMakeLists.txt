cmake_minimum_required(VERSION 2.8.3)
project(swri_console)

find_package(catkin REQUIRED COMPONENTS rosbag_storage roscpp rosgraph_msgs)
find_package(Qt4 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS Qt4
  CATKIN_DEPENDS rosbag_storage roscpp rosgraph_msgs
)

include(${QT_USE_FILE})
include_directories(include 
  ${catkin_INCLUDE_DIRS} 
  ${QT_INCLUDE_DIR} )
add_definitions(${QT_DEFINITIONS})

set(QT_USE_QTCORE TRUE)
set(QT_USE_QTGUI TRUE)

# Build mapviz node
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(src)
file (GLOB UI_FILES
  ui/console_window.ui)
file (GLOB HEADER_FILES
  include/swri_console/bag_reader.h
  include/swri_console/console_master.h
  include/swri_console/console_window.h
  include/swri_console/log_database.h
  include/swri_console/node_list_model.h
  include/swri_console/log_database_proxy_model.h
  include/swri_console/ros_thread.h)
file (GLOB SRC_FILES
  src/bag_reader.cpp
  src/console_master.cpp
  src/console_window.cpp
  src/log_database.cpp
  src/node_list_model.cpp
  src/log_database_proxy_model.cpp
  src/ros_thread.cpp
  src/settings_keys.cpp)
qt4_add_resources(RCC_SRCS resources/images.qrc)
qt4_wrap_ui(SRC_FILES ${UI_FILES})
qt4_wrap_cpp(SRC_FILES ${HEADER_FILES})

add_executable(swri_console ${HEADER_FILES} ${SRC_FILES} ${RCC_SRCS} src/main.cpp)
target_link_libraries(swri_console
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

