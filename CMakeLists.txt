cmake_minimum_required(VERSION 3.7.2)
project(warehouse_ros_sqlite VERSION 1.0.1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(sqlite3_vendor REQUIRED)
find_package(SQLite3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system thread)
find_package(class_loader REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(warehouse_ros REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/database_connection.cpp
  src/message_collection_helper.cpp
  src/metadata.cpp
  src/query.cpp
  src/result_iteration_helper.cpp
)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_VISIBILITY_PRESET hidden)
set_property(TARGET ${PROJECT_NAME} PROPERTY VISIBILITY_INLINES_HIDDEN ON)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
# generate macros for symbol visibility
include(GenerateExportHeader)
generate_export_header(${PROJECT_NAME} EXPORT_FILE_NAME ${PROJECT_NAME}/${PROJECT_NAME}_export.hpp)
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>")
target_link_libraries(${PROJECT_NAME} OpenSSL::Crypto)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 14)
target_link_libraries(${PROJECT_NAME}
  SQLite::SQLite3
)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${Boost_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME}
  class_loader
  rclcpp
  warehouse_ros
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/${PROJECT_NAME}_export.hpp
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(geometry_msgs REQUIRED)

  ament_add_gtest(DBConnection test/DatabaseConnection.cpp)
  target_link_libraries(DBConnection ${PROJECT_NAME})
  ament_add_gtest(SchemaVersion test/SchemaVersion.cpp)
  target_link_libraries(SchemaVersion ${PROJECT_NAME})

  ament_add_gtest(TestLoader test/TestLoader.cpp)
  target_link_libraries(TestLoader ${PROJECT_NAME})
  target_link_libraries(TestLoader ${Boost_LIBRARIES})

  ament_add_gtest(BusyHandler test/BusyHandler.cpp)
  target_link_libraries(BusyHandler ${PROJECT_NAME})
  target_link_libraries(BusyHandler ${Boost_LIBRARIES})

  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(FILTERS "-build/include_order")
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_cmake
  rclcpp
  warehouse_ros
  class_loader
  OpenSSL
  sqlite3_vendor
  SQLite3
)

pluginlib_export_plugin_description_file(warehouse_ros sqlite_database_connection_plugin_description.xml)

ament_package()
