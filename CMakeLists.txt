cmake_minimum_required(VERSION 3.7.2)
project(warehouse_ros_sqlite)

set(CMAKE_CXX_VISIBILITY_PRESET hidden)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(PkgConfig REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

pkg_check_modules(SQLite3 REQUIRED IMPORTED_TARGET sqlite3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rostest
  rostime
  warehouse_ros
  class_loader
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS warehouse_ros roscpp
)

add_library(${PROJECT_NAME} SHARED
  src/database_connection.cpp
  src/message_collection_helper.cpp
  src/metadata.cpp
  src/query.cpp
  src/result_iteration_helper.cpp
)
# generate macros for symbol visibility
include(GenerateExportHeader)
generate_export_header(${PROJECT_NAME} EXPORT_FILE_NAME ${PROJECT_NAME}/${PROJECT_NAME}_export.h)
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>")

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 14)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} PkgConfig::SQLite3)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})


install(FILES sqlite_database_connection_plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/${PROJECT_NAME}_export.h
        DESTINATION include/${PROJECT_NAME})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(DBConnection test/DatabaseConnection.cpp)
  target_link_libraries(DBConnection ${PROJECT_NAME})

  catkin_add_gtest(SchemaVersion test/SchemaVersion.cpp)
  target_link_libraries(SchemaVersion ${PROJECT_NAME} PkgConfig::SQLite3)

  add_rostest_gtest(TestLoader test/loader.test test/TestLoader.cpp)
  target_include_directories(TestLoader SYSTEM PUBLIC ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
  target_link_libraries(TestLoader ${catkin_LIBRARIES} ${Boost_LIBRARIES})

  catkin_add_gtest(BusyHandler test/BusyHandler.cpp)
  target_include_directories(TestLoader SYSTEM PUBLIC ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
  target_link_libraries(BusyHandler ${PROJECT_NAME} ${Boost_LIBRARIES})
endif()
