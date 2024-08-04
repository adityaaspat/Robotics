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
#include "tf2/utils.h"  // Include this header for tf2::getYaw
#include <vector>
#include <tuple>
#include <cmath>
#include <algorithm>  // Include this header for std::reverse
#include <Eigen/Dense>
class RRTNode {
public:
    double x, y;
    RRTNode* parent;
    double cost;

    RRTNode(double x, double y) : x(x), y(y), parent(nullptr), cost(0.0) {}
};


class RRTStar {
public:
    RRTStar(const std::vector<int>& start, const std::vector<int>& map_bounds, std::vector<int8_t> grid_data, double step_size = 2.0, int max_iter = 1000, double radius = 1) 
        : step_size(step_size), max_iter(max_iter), radius(radius), flag(0), map_bounds(map_bounds) , data(grid_data)
        {
        start_node = new RRTNode(start[0], start[1]);
        goal_node = new RRTNode(100, 100);
        tree.push_back(start_node);
        srand(time(0));
    }
// public variables
RRTNode* start_node;
RRTNode* goal_node;

    std::vector<RRTNode*> build_rrt_star() {
        for (int i = 0; i < max_iter; ++i) {
            RRTNode* random_node = get_random_node();
            RRTNode* nearest_node = get_nearest_node(random_node);
            RRTNode* new_node = steer(nearest_node, random_node);

            if (!is_within_bounds(new_node) || is_wall(new_node)) {
                delete new_node;
                continue;
            }

            auto nearby_nodes = get_nearby_nodes(new_node);
            choose_parent(new_node, nearby_nodes);
            tree.push_back(new_node);
            rewire(new_node, nearby_nodes);

            if (flag == 0) {
                int linear_index = static_cast<int>(new_node->y) * map_bounds[1] + static_cast<int>(new_node->x);  // row* num_of_rows + col, num_of rows=height
                int8_t value_8_t = data[linear_index];
                int value = static_cast<int>(value_8_t);
                if (value == -1) {
                    // this->goal_node = new RRTNode(new_node->x, new_node->y);
                    this->goal_node->x=new_node->x;
                    this->goal_node->y=new_node->y;
                    flag = 1;
                }
            }
        }

        RRTNode* goal_node = get_nearest_node(this->goal_node);
        if (is_goal_reached(goal_node)) {
            this->goal_node->parent = goal_node;
            this->goal_node->cost = goal_node->cost + distance(goal_node, this->goal_node);
            tree.push_back(this->goal_node);
            return get_path();
        }

        return {};
    }


private:
    
    std::vector<RRTNode*> tree;
    double step_size;
    int max_iter;
    double radius;
    int flag = 0;
    std::vector<int> map_bounds;
    std::vector<int8_t> data;

    RRTNode* get_random_node() {
        double x = static_cast<double>(rand()) / RAND_MAX * (map_bounds[1] - map_bounds[0]) + map_bounds[0];
        double y = static_cast<double>(rand()) / RAND_MAX * (map_bounds[3] - map_bounds[2]) + map_bounds[2];
        return new RRTNode(x, y);
    }

    RRTNode* get_nearest_node(RRTNode* random_node) {
        RRTNode* nearest_node = tree[0];
        double min_dist = distance(nearest_node, random_node);
        for (auto node : tree) {
            double dist = distance(node, random_node);
            if (dist < min_dist) {
                nearest_node = node;
                min_dist = dist;
            }
        }
        return nearest_node;
    }

    std::vector<RRTNode*> get_nearby_nodes(RRTNode* new_node) {
        std::vector<RRTNode*> nearby_nodes;
        for (auto node : tree) {
            if (distance(node, new_node) <= radius) {
                nearby_nodes.push_back(node);
            }
        }
        return nearby_nodes;
    }

    double distance(RRTNode* node1, RRTNode* node2) {
        return sqrt(pow(node1->x - node2->x, 2) + pow(node2->y - node2->y, 2));
    }

    RRTNode* steer(RRTNode* from_node, RRTNode* to_node) {
        RRTNode* new_node = new RRTNode(from_node->x, from_node->y);
        double theta = atan2(to_node->y - from_node->y, to_node->x - from_node->x);
        new_node->x += step_size * cos(theta);
        new_node->y += step_size * sin(theta);
        double dist = distance(from_node, new_node);
        new_node->parent = from_node;
        new_node->cost = from_node->cost + dist;
        return new_node;
    }

    bool is_goal_reached(RRTNode* node) {
        return distance(node, goal_node) <= step_size;
    }

    void choose_parent(RRTNode* new_node, const std::vector<RRTNode*>& nearby_nodes) {
        RRTNode* best_node = new_node->parent;
        double min_cost = new_node->cost;
        for (auto node : nearby_nodes) {
            double cost = node->cost + distance(node, new_node);
            if (cost < min_cost) {
                best_node = node;
                min_cost = cost;
            }
        }
        new_node->parent = best_node;
        new_node->cost = min_cost;
    }

    void rewire(RRTNode* new_node, const std::vector<RRTNode*>& nearby_nodes) {
        for (auto node : nearby_nodes) {
            double cost = new_node->cost + distance(new_node, node);
            if (cost < node->cost) {
                node->parent = new_node;
                node->cost = cost;
            }
        }
    }

    bool is_wall(RRTNode* node) {
        int linear_index = static_cast<int>(node->y) * map_bounds[1] + static_cast<int>(node->x);
        int8_t value_8_t = data[linear_index];
        int value = static_cast<int>(value_8_t);
        return value > 70;
    }

    bool is_within_bounds(RRTNode* node) {
        return map_bounds[0] <= node->x && node->x <= map_bounds[1] && map_bounds[2] <= node->y && node->y <= map_bounds[3];
    }

    std::vector<RRTNode*> get_path() {
        std::vector<RRTNode*> path;
        RRTNode* node = goal_node;
        while (node != nullptr) {
            path.push_back(node);
            node = node->parent;
        }
        std::reverse(path.begin(), path.end());
        return path;
    }
};


class Explore : public rclcpp::Node
{
public:
    Explore()
    : Node("rrt_star_goal_finder"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
    {
        map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&Explore::map_callback, this, std::placeholders::_1));
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&Explore::odom_callback, this, std::placeholders::_1)); 
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("exploration_rrt_goal", 10);
        
    }

    int width, height, map_x, map_y,flag=0;
    double resolution, origin_x, origin_y;
    std::vector<double> robot_state;
    geometry_msgs::msg::PoseStamped robot_pose_;
    std::vector<int8_t> grid_data;

    void call_RRT_star() {
        std::vector<int> start = {map_x, map_y};
        std::vector<int> map_bounds = {0, width, 0, height}; // 0-x,0-y

        RRTStar rrt_star(start, map_bounds, grid_data);
        std::vector<RRTNode*> path = rrt_star.build_rrt_star();

        // Print the path
        // if (!path.empty()) {
        //     std::cout << "Path:" << std::endl;
        //     for (const auto& point : path) {
        //         std::cout << "(" << point->x << ", " << point->y << ")" << std::endl;
        //     }
        // } else {
        //     std::cout << "No path found." << std::endl;
        

        std::cout<<"GOAL: "<< "(" << rrt_star.goal_node->x << ", " << rrt_star.goal_node->y << ")" << std::endl;
        double x = origin_x + (rrt_star.goal_node->x * resolution);
        double y = origin_y + (rrt_star.goal_node->y * resolution);
        std::cout<<"GOAL IN MAP: "<< "(" << x << ", " << y << ")" << std::endl;

        auto pose_stamped_msg = geometry_msgs::msg::PoseStamped();
        pose_stamped_msg.header.stamp = this->get_clock()->now();
        pose_stamped_msg.header.frame_id = "map"; 
        pose_stamped_msg.pose.position.x = x;
        pose_stamped_msg.pose.position.y = y;
        pose_stamped_msg.pose.position.z = 0.0;  
        pose_stamped_msg.pose.orientation.x = 0.0;
        pose_stamped_msg.pose.orientation.y = 0.0;
        pose_stamped_msg.pose.orientation.z = 0.0;
        pose_stamped_msg.pose.orientation.w = 1.0;
        publisher_->publish(pose_stamped_msg);
    }

    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        grid_data = msg->data;
        width = msg->info.width;
        height = msg->info.height;
        resolution = msg->info.resolution;
        origin_x = msg->info.origin.position.x;
        origin_y = msg->info.origin.position.y;
        for (int i=0;i<height;i++)
        {
            for(int j=0;j<width;j++)
            {   
               int linear_index= i*width+j;
               if (grid_data[linear_index]<70 && grid_data[linear_index]>0 )
               {
                grid_data[linear_index]=-1;
               }
            }
        }
        if (flag==1)
            {
            call_RRT_star();
            rclcpp::shutdown();
            }
    }
  
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Transform the odometry position to the map frame
        
        geometry_msgs::msg::PoseStamped odom_pose;
        odom_pose.header = msg->header;
        odom_pose.pose = msg->pose.pose;
        
        try {
            robot_pose_ = tf_buffer_.transform(odom_pose, "map", tf2::durationFromSec(1.0));
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform odom to map: %s", ex.what());
            return;
        }
        flag=1;
        double x = robot_pose_.pose.position.x;
        double y = robot_pose_.pose.position.y;

        tf2::Quaternion quat;
        tf2::fromMsg(robot_pose_.pose.orientation, quat);
        double theta = tf2::getYaw(quat);

        map_x = static_cast<int>((x - origin_x) / resolution);
        map_y = static_cast<int>((y - origin_y) / resolution);

        robot_state = {x, y, theta};
    }

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Explore>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
