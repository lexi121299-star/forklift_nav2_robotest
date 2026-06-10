cmake_minimum_required(VERSION 3.8)
project(forklift_nav2_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(forklift_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(dependencies
  forklift_msgs
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_library(${PROJECT_NAME} SHARED
  src/forklift_mpc_controller.cpp
  src/forklift_mpc_preview_window.cpp
  src/forklift_mpc_types.cpp
  src/forklift_mpc_trajectory.cpp
  src/forklift_vehicle_model.cpp
  src/oru_global_planner.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

pluginlib_export_plugin_description_file(nav2_core plugins.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES plugins.xml README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_forklift_mpc_types
    test/test_forklift_mpc_types.cpp
  )
  if(TARGET test_forklift_mpc_types)
    target_link_libraries(test_forklift_mpc_types
      ${PROJECT_NAME}
    )
    ament_target_dependencies(test_forklift_mpc_types
      geometry_msgs
      tf2
      tf2_geometry_msgs
    )
  endif()

  ament_add_gtest(test_forklift_mpc_trajectory
    test/test_forklift_mpc_trajectory.cpp
  )
  if(TARGET test_forklift_mpc_trajectory)
    target_link_libraries(test_forklift_mpc_trajectory
      ${PROJECT_NAME}
    )
    ament_target_dependencies(test_forklift_mpc_trajectory
      geometry_msgs
      nav_msgs
      tf2
      tf2_geometry_msgs
    )
  endif()

  ament_add_gtest(test_forklift_mpc_preview_window
    test/test_forklift_mpc_preview_window.cpp
  )
  if(TARGET test_forklift_mpc_preview_window)
    target_link_libraries(test_forklift_mpc_preview_window
      ${PROJECT_NAME}
    )
    ament_target_dependencies(test_forklift_mpc_preview_window
      geometry_msgs
      nav_msgs
    )
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})
ament_package()
