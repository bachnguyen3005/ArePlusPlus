#include <unordered_map>
#include <queue>
#include <stdexcept>
#include <optional>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"

#include <fstream>
#include <sstream>
#include <vector>
#include <utility>

#include "Pose2d.h"

// TODO timer callback that tries to assign a job.
// Check the current waypoint to see if at destination.
// Need some variables to encode state. Trigger a motion request when none are active,
// and monitor to see if it stalls/reaches. If stalled, abort and go to the next task (log an error)
// If reached, try to get the next waypoint. If no waypoints remain, log sucessful completion and then load the next task.

class TaskPlanner : public rclcpp::Node {
public:
    TaskPlanner(std::vector<std::pair<int, int>> initial_tasks) : Node("task_planner") {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&TaskPlanner::odom_callback, this, std::placeholders::_1));

        move_orders_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/move_orders", 10, std::bind(&TaskPlanner::move_orders_callback, this, std::placeholders::_1));
        for (auto task: initial_tasks){
            order_queue.push(task);
        }
        RCLCPP_INFO(this->get_logger(), "Received %li tasks", initial_tasks.size());
    }

    TaskPlanner() : Node("task_planner") {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&TaskPlanner::odom_callback, this, std::placeholders::_1));

        move_orders_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/move_orders", 10, std::bind(&TaskPlanner::move_orders_callback, this, std::placeholders::_1));
    }

    // Set a product location
    void set_product_location(int id, float x, float y, float yaw) {
        product_locations[id] = Pose2d(x, y, yaw);
    }

    // Set all product locations 
    void set_all_product_locations(const std::unordered_map<int, Pose2d>& new_locations) {
        product_locations = new_locations;
    }

    // Get a product location
    Pose2d get_product_location(int id) {
        auto it = product_locations.find(id);
        if (it == product_locations.end()) {
            throw std::runtime_error("Product ID not found");
        }
        return it->second;
    }

    // Set all station locations (overwrites the entire map)
    void set_all_station_locations(const std::unordered_map<int, Pose2d>& new_locations) {
        station_locations = new_locations;
    }

    // Get a station location
    Pose2d get_station_location(int id) {
        auto it = station_locations.find(id);
        if (it == station_locations.end()) {
            throw std::runtime_error("Station ID not found");
        }
        return it->second;
    }

    // Get the next order from the queue
    std::pair<int, int> get_next_order() {
        if (order_queue.empty())
        {
            throw std::runtime_error("No orders available");
        }
        auto order = order_queue.front();
        order_queue.pop();
        return order;
    }

    // Check if there are any orders in the queue
    bool has_orders() {
        return !order_queue.empty();
    }

    bool get_next_navtask(std::vector<geometry_msgs::msg::Pose>& poses) {
        if (!has_orders()) {
            return false;
        }

        std::pair<int, int> current_order_ = get_next_order();

        try {
            poses.clear();

            // Get product location
            auto product_loc = get_product_location(current_order_.first);
            poses.push_back(product_loc.to_pose());

            // Get station location
            auto station_loc = get_station_location(current_order_.second);
            poses.push_back(station_loc.to_pose());

            return true;

        } catch (const std::runtime_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Error generating navtask: %s", e.what());
            return false;
        }
    }

    void set_activation_state(bool state) {
        activate = state;
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received odom: x=%f, y=%f, z=%f",
                    msg->pose.pose.position.x,
                    msg->pose.pose.position.y,
                    msg->pose.pose.position.z);
    }

    void move_orders_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 2)
        {
            int first = msg->data[0];
            int second = msg->data[1];
            order_queue.push(std::make_pair(first, second));
            RCLCPP_INFO(this->get_logger(), "Queued move order: (%d, %d)", first, second);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Invalid move order");
        }
    }

    std::unordered_map<int, Pose2d> product_locations;
    std::unordered_map<int, Pose2d> station_locations;
    std::queue<std::pair<int, int>> order_queue;
    
    bool activate = false;
    Pose2d current_waypoint = Pose2d(0, 0, 0);
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr move_orders_sub_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TaskPlanner>();

    std::vector<std::pair<int, int>> initial_tasks;
    bool do_auto_load_orders = true;
    bool do_auto_load_locations = true;

    if (do_auto_load_orders) {
        initial_tasks.push_back(std::pair<int, int>(3, 1));
        initial_tasks.push_back(std::pair<int, int>(2, 3));
        initial_tasks.push_back(std::pair<int, int>(3, 2));
    }

    // Update the TaskPlanner with initial tasks
    node = std::make_shared<TaskPlanner>(initial_tasks);

    if (do_auto_load_locations) {
        std::unordered_map<int, Pose2d> station_locations;
        std::unordered_map<int, Pose2d> product_locations;
        
        station_locations[1] = Pose2d(1, 5, 0);
        station_locations[2] = Pose2d(3, 5, 0);
        station_locations[3] = Pose2d(5, 5, 0);

        station_locations[-1] = Pose2d(1, 3, 0);  // TODO -stations are for products to stay
        station_locations[-2] = Pose2d(3, 3, 0);
        station_locations[-3] = Pose2d(5, 3, 0);
        station_locations[-4] = Pose2d(1, 0, 0);
        station_locations[-5] = Pose2d(3, 0, 0);
        station_locations[-6] = Pose2d(5, 0, 0);

        product_locations[1] = Pose2d(1, 3, 0);
        product_locations[2] = Pose2d(3, 3, 0);
        product_locations[3] = Pose2d(5, 3, 0);
        product_locations[4] = Pose2d(1, 0, 0);
        product_locations[5] = Pose2d(3, 0, 0);
        product_locations[6] = Pose2d(5, 0, 0);

        node->set_all_product_locations(product_locations);
        node->set_all_station_locations(station_locations);
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}