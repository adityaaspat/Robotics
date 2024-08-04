cmake_minimum_required(VERSION 3.8)
project(trajectory_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ompl REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rtabmap_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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


add_executable(slam_2D_planner src/karto_planner.cpp)
add_executable(mppi_controller src/mppi.cpp)


ament_target_dependencies(slam_2D_planner rclcpp std_msgs geometry_msgs ompl tf2_geometry_msgs tf2_ros nav_msgs rtabmap_msgs)
ament_target_dependencies(mppi_controller rclcpp std_msgs geometry_msgs ompl tf2_geometry_msgs tf2_ros nav_msgs rtabmap_msgs)

target_link_libraries(slam_2D_planner ${OMPL_LIBRARIES})
target_include_directories(slam_2D_planner PRIVATE ${OMPL_INCLUDE_DIRS})

install(TARGETS
    kinova_planner
    slam_2D_planner
    mppi_controller
  DESTINATION lib/${PROJECT_NAME})
