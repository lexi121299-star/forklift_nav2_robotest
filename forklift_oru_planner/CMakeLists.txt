cmake_minimum_required(VERSION 3.8)
project(forklift_oru_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME} STATIC
  vendor/src/oru_lattice_core.cpp
)
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vendor/include>"
    "$<INSTALL_INTERFACE:include>"
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY vendor/include/
  DESTINATION include
)

install(
  DIRECTORY primitives vendor/licenses
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_oru_lattice_core
    test/test_oru_lattice_core.cpp
  )
  if(TARGET test_oru_lattice_core)
    target_link_libraries(test_oru_lattice_core ${PROJECT_NAME})
    target_compile_definitions(test_oru_lattice_core PRIVATE
      TEST_PRIMITIVE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/primitives/forklift_16_heading.mprim"
    )
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
