cmake_minimum_required(VERSION 3.7.2)
project(warehouse_ros_sqlite)

add_compile_options(-std=c++14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(PkgConfig REQUIRED)
find_package(Boost REQUIRED COMPONENTS)

pkg_check_modules(SQLite3 REQUIRED IMPORTED_TARGET sqlite3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rostest
  rostime
  std_msgs
  warehouse_ros
  class_loader
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS warehouse_ros
)

add_library(${PROJECT_NAME}
  src/database_connection.cpp
  src/message_collection_helper.cpp
  src/metadata.cpp
  src/query.cpp
  src/result_iteration_helper.cpp
)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 14)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} PkgConfig::SQLite3)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})


install(FILES sqlite_database_connection_plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
