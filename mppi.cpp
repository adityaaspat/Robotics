#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" 
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"  // Include this header for tf2::getYaw
#include <vector>
#include <tuple>
#include <cmath>
#include <algorithm>  // Include this header for std::reverse
#include <Eigen/Dense>

class PathSubscriber : public rclcpp::Node
{
public:
    PathSubscriber()
    : Node("path_subscriber"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)

    {
        path_subscription = this->create_subscription<nav_msgs::msg::Path>(
            "/planned_path", 
            10, 
            std::bind(&PathSubscriber::path_callback, this, std::placeholders::_1));
        map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&PathSubscriber::map_callback, this, std::placeholders::_1));
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&PathSubscriber::odom_callback, this, std::placeholders::_1));
        control_publisher= this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);
        generate_combinations();
    
    }
std::vector<std::tuple<double, double>> waypoints;
geometry_msgs::msg::PoseStamped robot_pose_;
std::vector<std::vector<double>> actions;
rclcpp::TimerBase::SharedPtr timer_;
int flag=0;
std::size_t current_waypoint_index;
std::vector<double> robot_state;
std::vector<double> goal;
int width,height,map_x,map_y;

float resolution, origin_x, origin_y;

private:
void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
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
void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    try {
      geometry_msgs::msg::TransformStamped transformStamped = tf_buffer_.lookupTransform(
        "map", "odom", tf2::TimePointZero);

      geometry_msgs::msg::PoseStamped odom_pose;
      odom_pose.header = msg->header;
      odom_pose.pose = msg->pose.pose;
      tf2::doTransform(odom_pose, robot_pose_, transformStamped);
      double yaw = tf2::getYaw(robot_pose_.pose.orientation);

      robot_state={robot_pose_.pose.position.x,robot_pose_.pose.position.y,yaw};

      auto [robot_x, robot_y]=world_to_grid(robot_pose_.pose.position.x,robot_pose_.pose.position.y);
      map_x=robot_x;
      map_y=robot_y;
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform odom to map: %s", ex.what());
    }
  }

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg) 
    {
        // Access the header information
        RCLCPP_INFO(this->get_logger(), "Robot state: %f %f , %d %d", robot_pose_.pose.position.x,robot_pose_.pose.position.y,map_x,map_y);

        waypoints.clear();

        // Process each pose in the path
        for (const auto& pose_stamped : msg->poses)
        {
            auto position = pose_stamped.pose.position;
            waypoints.emplace_back(position.x,position.y);
        }
        RCLCPP_INFO(this->get_logger(), "Num of waypoints: %zu", waypoints.size());

         for (const auto &[x, y] : waypoints) {
        RCLCPP_INFO(this->get_logger(), "Path point: x = %f, y = %f", x, y);
        }
        flag=0;
        current_waypoint_index=0;
        if (!timer_ && flag==0)
        {
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(500),
                std::bind(&PathSubscriber::mppi_control, this));
        }
    }

// Linspace function
    std::vector<double> linspace(double start, double end, int num)
    {
        std::vector<double> values;
        double step = (end - start) / (num - 1);
        for (int i = 0; i < num; ++i)
        {
            values.push_back(start + i * step);
        }
        return values;
    }
// // Generate action combinations
    void generate_combinations()
    {
        const size_t num_samples = 44;
        const double angular_min = -2.84, angular_max = 2.84;

        std::vector<double> linear_combinations(88);
        for (int i=0;i<88;i++)
        {   
            double v;
            if (i<44)
                {
                    v=0.1;
                }
            else
            {
                v=-0.1;
            }
            linear_combinations[i]=v;
        }
        std::vector<double> angular_combinations1 = linspace(angular_min, angular_max, num_samples);
        for (int i=0;i<88;i++)
        {   
            double v;
            double w;
            v=linear_combinations[i];
            if (i<44)
                w=angular_combinations1[i];
            else
            {
                w=angular_combinations1[i-44];
            }
                actions.emplace_back(std::vector<double>{v, w});
            
        }
    }
    double cost_function( std::vector<double> state)
    {   
        double dx = state[0] - goal[0];
        double dy = state[1] - goal[1];
        return std::sqrt(dx * dx + dy * dy);
    }
    std::vector<double> step(const std::vector<double>& state, const std::vector<double>& action, double dt = 0.5)
    {   
        double x = state[0];
        double y = state[1];
        double theta = state[2];
        double v = action[0];
        double w = action[1];

        theta += w * dt;
        x += v * std::cos(theta) * dt;
        y += v * std::sin(theta) * dt;

        return {x, y, theta};
    }
    // Generate trajectories based on state and action combinations
    std::vector<double> calculate_best_action(const std::vector<double>& state)
    {

        std::vector<std::vector<double>> next_states(actions.size(), std::vector<double>(3));
        std::vector<double> costs_of_states(actions.size());

        for (size_t i = 0; i < actions.size(); ++i)
        {
            std::vector<double> action = actions[i];
            std::vector<double> new_state = step(state, action);
            next_states[i] = new_state;
            costs_of_states[i]=cost_function(new_state);
        }
        auto min_it = std::min_element(costs_of_states.begin(), costs_of_states.end());
    
        // Get the index of the minimum element
        int min_index = std::distance(costs_of_states.begin(), min_it);
        std::vector<double> best_action=actions[min_index];

        return best_action;

    }

void mppi_control()
    {
        if (current_waypoint_index == waypoints.size())
        {
            RCLCPP_INFO(this->get_logger(), "Reached all waypoints.");
            RCLCPP_INFO(this->get_logger(), "Stopping...");
            
            geometry_msgs::msg::Twist twist;
            twist.linear.x = 0.0; // Linear speed
            twist.angular.z = 0.0; // Angular speed
            control_publisher->publish(twist);
            flag=1;
            return;
        }
        else
        {
            auto [g_x, g_y] = waypoints[current_waypoint_index];
            goal={g_x,g_y};
            double distance = std::sqrt(std::pow(goal[0] - robot_state[0], 2) +
                                        std::pow(goal[1] - robot_state[1], 2));
            if (distance < 0.1)
                {   
                RCLCPP_INFO(this->get_logger(), "Reached waypoint number :  %zu (%f , %f), Robot state: (%d,%d)",current_waypoint_index,robot_state[0],robot_state[1],map_x,map_y);
                current_waypoint_index++;
                }
        
            std::vector<double> best_action=calculate_best_action(robot_state);
            geometry_msgs::msg::Twist twist;
            twist.linear.x = best_action[0]; // Linear speed
            twist.angular.z = best_action[1]; // Angular speed

            control_publisher->publish(twist);
    
        }
    }
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr control_publisher;
};
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathSubscriber>());
    rclcpp::shutdown();
    return 0;
}