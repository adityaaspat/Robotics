#include "rclcpp/rclcpp.hpp"
#include <tuple>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // Include this for tf2::doTransform
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rtabmap_msgs/msg/map_data.hpp" // Include the MapData message header
#include <rtabmap_msgs/msg/node.hpp>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <iostream>
#include <vector>
#include <cmath>


class Planner : public rclcpp::Node
{
public:
  Planner()
  : Node("Trajectory_plan"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&Planner::map_callback, this, std::placeholders::_1));
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&Planner::odom_callback, this, std::placeholders::_1));
    goal_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/move_base_simple/goal", 10, std::bind(&Planner::goal_callback, this, std::placeholders::_1));
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
  }
  geometry_msgs::msg::PoseStamped goal_pose_;
  geometry_msgs::msg::PoseStamped robot_pose_;
  std::vector<int8_t> grid_data;
  int width,height;
  float resolution, origin_x, origin_y, goal_theta;
  int robot_grid_x, robot_grid_y, goal_x,goal_y;
  std::vector<std::tuple<double, double>> planned_path_;
private:
  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // Update the goal pose with the received goal
    goal_pose_ = *msg;
    auto [goal_grid_x, goal_grid_y]=world_to_grid(goal_pose_.pose.position.x,goal_pose_.pose.position.y);

    const auto& orientation = msg->pose.orientation;
    tf2::Quaternion quaternion;
            quaternion.setX(orientation.x);
            quaternion.setY(orientation.y);
            quaternion.setZ(orientation.z);
            quaternion.setW(orientation.w);

    // Convert quaternion to Euler angles (roll, pitch, yaw)
    double roll, pitch, yaw;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    goal_theta=yaw;
    goal_x = goal_grid_x;
    goal_y = goal_grid_y;
    RCLCPP_INFO(this->get_logger(), "Received Goal with X %d, Y %d and Theta %f",goal_x, goal_y,goal_theta);
    RCLCPP_INFO(this->get_logger(), "Robot Start state X %d, Y %d ",robot_grid_x, robot_grid_y);
    RCLCPP_INFO(this->get_logger(), "Received map with width %d and height %d",width, height);
    RCLCPP_INFO(this->get_logger(), "Resolution: %f",resolution);
    RCLCPP_INFO(this->get_logger(), "Origin: x: %f, y: %f ", origin_x, origin_y);
    RCLCPP_INFO(this->get_logger(), "Planning Optimal Path.....");
    plan_path();
  }
   void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
        grid_data = msg->data;
        width=msg->info.width;
        height=msg->info.height;
        resolution=msg->info.resolution;
        origin_x=msg->info.origin.position.x;
        origin_y=msg->info.origin.position.y;


  }
  std::tuple<int, int> world_to_grid(double world_x, double world_y) 
  {
    int grid_x = static_cast<int>((world_x - origin_x) / resolution);
    int grid_y = static_cast<int>((world_y - origin_y) / resolution);
    return std::make_tuple(grid_x, grid_y);
  }
  std::tuple<double, double> grid_to_world(int row, int col) 
  {
    double x = origin_x + (col * resolution);
    double y = origin_y + (row * resolution);
    return std::make_tuple(round(x * 1e5) / 1e5, round(y * 1e5) / 1e5);
  }
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    try {
      geometry_msgs::msg::TransformStamped transformStamped = tf_buffer_.lookupTransform(
        "map", "odom", tf2::TimePointZero);

      geometry_msgs::msg::PoseStamped odom_pose;
      odom_pose.header = msg->header;
      odom_pose.pose = msg->pose.pose;

      tf2::doTransform(odom_pose, robot_pose_, transformStamped);

      auto [robot_x, robot_y]=world_to_grid(robot_pose_.pose.position.x,robot_pose_.pose.position.y);
      robot_grid_x=robot_x;
      robot_grid_y=robot_y;
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform odom to map: %s", ex.what());
    }
  }

bool isStateValid(const ompl::base::State *state) 
{
    const auto *pos = state->as<ompl::base::RealVectorStateSpace::StateType>();
    int x = (int)pos->values[0];
    int y = (int)pos->values[1];
    if (x < 0 || x >= width || y < 0 || y >= height)
        return false; // Out of bounds
    int linear_index= x*width + y;
    int8_t value_8_t = grid_data[linear_index];
    int value=(int)value_8_t;
    return value < 40; // Free space
}
void plan_path()
  {
    // Define the state space (2D)
    auto space(std::make_shared<ompl::base::RealVectorStateSpace>(2));

    // Set the bounds of the space
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.setHigh(0, width - 1);
    bounds.setHigh(1, height - 1);
    space->setBounds(bounds);

    // Create a space information instance for this state space
    auto si(std::make_shared<ompl::base::SpaceInformation>(space));
    si->setStateValidityChecker([this](const ompl::base::State *state) {
      return isStateValid(state);
    });

    // Define the start and goal states
    ompl::base::ScopedState<> start(space);
    start[0] = robot_grid_x; // start x
    start[1] = robot_grid_y; // start y

    ompl::base::ScopedState<> goal(space);
    goal[0] = goal_x; // goal x
    goal[1] = goal_y; // goal y

    // Create a problem definition
    auto pdef(std::make_shared<ompl::base::ProblemDefinition>(si));
    pdef->setStartAndGoalStates(start, goal);

    // Create a planner for the defined space
    auto planner(std::make_shared<ompl::geometric::RRTstar>(si));
    planner->setProblemDefinition(pdef);
    planner->setup();

    // Create a termination condition for 1 second
    ompl::base::PlannerTerminationCondition ptc = ompl::base::timedPlannerTerminationCondition(2.0);

    // Try to solve the problem
    ompl::base::PlannerStatus solved = planner->solve(ptc);
    // double odom_x,odom_y;
    if (solved) {
      std::cout << "Found solution:" << std::endl;
      // Get the obtained path
      ompl::geometric::PathGeometric path = *std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(pdef->getSolutionPath());
      planned_path_.clear();
      for (size_t i = 0; i < path.getStateCount(); i++) 
      {
        const auto *state = path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
        int x = static_cast<int>(state->values[1]);
        int y = static_cast<int>(state->values[0]);
        auto [odom_x, odom_y]=grid_to_world(x,y);
        planned_path_.emplace_back(odom_x, odom_y);
      }

      for (const auto &[x, y] : planned_path_) {
        RCLCPP_INFO(this->get_logger(), "Path point: x = %f, y = %f", x, y);
      }
      publish_path();
      path.printAsMatrix(std::cout);
    } else {
      std::cout << "No solution found" << std::endl;
    }
  }

 void publish_path()
  {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = "map";

    for (const auto& [world_x, world_y] : planned_path_) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = this->now();
      pose.header.frame_id = "map";
      pose.pose.position.x = world_x;
      pose.pose.position.y = world_y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      path_msg.poses.push_back(pose);
    }

    path_publisher_->publish(path_msg);
  }
  




  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Planner>());
  rclcpp::shutdown();
  return 0;
}
