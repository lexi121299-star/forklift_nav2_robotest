cmake_minimum_required(VERSION 3.8)
project(forklift_nav2_demo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch urdf config worlds docs behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/forklift_teleop.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME forklift_teleop
)

install(
  PROGRAMS scripts/forklift_auto_slam_mapper.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME forklift_auto_slam_mapper
)

install(
  PROGRAMS scripts/forklift_manual_path_follower.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME forklift_manual_path_follower
)

install(
  PROGRAMS scripts/forklift_follow_path_acceptance.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME forklift_follow_path_acceptance
)

install(
  PROGRAMS scripts/forklift_ab_dynamic_obstacle_acceptance.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME forklift_ab_dynamic_obstacle_acceptance
)

install(
  PROGRAMS scripts/forklift_ab_acceptance.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME forklift_ab_acceptance
)

install(
  FILES scripts/forklift_ab_acceptance.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/forklift_p6_5a_pivot_acceptance.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME forklift_p6_5a_pivot_acceptance
)

install(
  PROGRAMS scripts/forklift_p6_reverse_acceptance.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME forklift_p6_reverse_acceptance
)

install(
  PROGRAMS scripts/wait_for_sim_ready.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME wait_for_sim_ready
)

install(
  PROGRAMS scripts/spawn_forklift_entity.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME spawn_forklift_entity
)

install(
  PROGRAMS scripts/forklift_scan_self_filter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME forklift_scan_self_filter
)

ament_package()
