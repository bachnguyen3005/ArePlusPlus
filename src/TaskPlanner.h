#ifndef TASK_PLANNER_H
#define TASK_PLANNER_H

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <unordered_map>
#include <queue>
#include "Structures.h"
#include "sensor_msgs/msg/image.hpp" // For AR tag detection
#include "sensor_msgs/msg/camera_info.hpp" // For AR tag detection
#include "apriltag_msgs/msg/april_tag_detection_array.hpp" // For AR tag detection
#include "cv_bridge/cv_bridge.h" // For AR tag detection
#include "opencv2/opencv.hpp" // For AR tag detection

class TaskPlanner : public rclcpp::Node {
public:
    TaskPlanner(std::vector<std::pair<int, int>> initial_tasks) : Node("task_planner") {
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TaskPlanner::timer_callback, this));

        for (auto task: initial_tasks){
            Order order(task.first, task.second);
            pending_orders_.push(order);
        }
    }

    TaskPlanner() : Node("task_planner") {
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TaskPlanner::timer_callback, this));
    }

private:
    void timer_callback();  // This one gets to do the main logic
    
    // These need to be implemented & doxygened
    bool is_at_target(const Pose2d& target);  // < Thish
    void manual_go_to_point(const Pose2d& target);  // < Thish
    void nav2_go_to_point(const Pose2d& target);  // < Jack
    void prep_next_order();  // < Adi
    bool load_locations_from_file();  // < Dinh, lmk if you want a hand here.
    bool get_visible_station_code(int& tag_id);  // < Dinh

    // Other todo:
    // E-stop services
    // Doxygen
    // Status outputs, with a silence flag

    // Member variables
    std::queue<Order> pending_orders_;
    std::queue<NavNode> current_job_points_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unordered_map<int, Station> station_locations;
    std::unordered_map<int, int> product_locations;  // Products are only allowed to be at stations

    std::vector<NavNode> generatePathToStation(const Pose2d& destination);

    bool is_active = false;  // Is the system allowed to perform operations

    int pickup_station_id = 0;
    int dropoff_station_id = 0;
    int package_id = 0;
    JobStatus status = JobStatus::Idle;
};

#endif // TASK_PLANNER_H