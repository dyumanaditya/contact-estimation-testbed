cmake_minimum_required(VERSION 3.8)
project(contact_estimation_testbed)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

set(MUJOCO_LIB_PATH ${PROJECT_SOURCE_DIR}/lib)
file(GLOB mujoco_libraries lib/*.so*)

add_executable(contact_estimation_testbed src/contact_estimation_testbed.cpp)
ament_target_dependencies(contact_estimation_testbed rclcpp)
target_link_libraries(contact_estimation_testbed ${mujoco_libraries})
target_include_directories(contact_estimation_testbed PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(contact_estimation_testbed PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Set RPATH so the executable knows where to find the MuJoCo library at runtime
set_target_properties(contact_estimation_testbed PROPERTIES
  INSTALL_RPATH "${MUJOCO_LIB_PATH}"
  BUILD_RPATH "${MUJOCO_LIB_PATH}"
)

install(TARGETS contact_estimation_testbed
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  lib
  DESTINATION lib/${PROJECT_NAME}/
)

install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
