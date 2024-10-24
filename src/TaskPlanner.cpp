#include "TaskPlanner.h"

bool TaskPlanner::is_at_target(const Pose2d& target) {
        // Jack, Thish here
        // You need to make sure both xy and yaw are within spec. 
        // If Nav2 is doing the movement, you need to check whether nav2 thinks its done. 
        //        This will involve using a callback, and doing error handling on the callback. See the older code for a reference.
        return false;
}
void TaskPlanner::nav2_go_to_point(const Pose2d& target) {
        // Jack here
        // You need to interlock with the manual code to make sure only one is running at any one time.
        // The most recent one to have started gets priority
}

void TaskPlanner::manual_go_to_point(const Pose2d& target) {
        // Thish here
        // You need to interlock with the nav2 code to make sure only one is running at any one time.
        // The most recent one to have started gets priority
        // Make yourself some tests for this to make sure it works
}

void TaskPlanner::prep_next_order() {
        if (!pending_orders_.size()) {
                return;
        }

        Order new_order = pending_orders_.front();
        pending_orders_.pop();
        
        // TODO
        //if (new_order.product_id not in product_locations): print warning and retry next
        //if (new_order.station_id not in station_locations): print warning and retry next 

        while (current_job_points_.size()) {
                current_job_points_.pop();
        }

        auto pathCtoA = station_locations[product_locations[new_order.product_id]].path;
        auto pathCtoB = station_locations[new_order.station_id].path;

        NavNode node(ActionType::start);
        current_job_points_.push(node);

        for (const auto& point : pathCtoA) {
                current_job_points_.push(point);
        }

        node.action_type = ActionType::pickup;
        current_job_points_.push(node);
        node.action_type = ActionType::advance_state;
        current_job_points_.push(node);
    
        for (auto it = pathCtoA.rbegin() + 1; it != pathCtoA.rend(); ++it) {
                current_job_points_.push(*it);
        }

        node.action_type = ActionType::advance_state;
        current_job_points_.push(node);

        for (const auto& point : pathCtoB) {
                current_job_points_.push(point);
        }

        node.action_type = ActionType::dropoff;
        current_job_points_.push(node);
        node.action_type = ActionType::advance_state;
        current_job_points_.push(node);

        for (auto it = pathCtoB.rbegin() + 1; it != pathCtoB.rend(); ++it) {
                current_job_points_.push(*it);
        }

        node.action_type = ActionType::finish;
        current_job_points_.push(node);

        package_id = new_order.product_id;
        dropoff_station_id = new_order.station_id;
        pickup_station_id = product_locations[new_order.product_id];
}

std::vector<NavNode> generatePathToStation(const Pose2d& destination) {
    std::vector<NavNode> path;
    NavNode node;
    node.pose = destination;  
    path.push_back(node);
    node.is_final_approach = true
    return path;
}


bool TaskPlanner::load_locations_from_file() {
        /*
        shelf#2 (-2,2)    shelf#3 (2,2)

        shelf#1 (-2,-1)   shelf#4 (2,-1)
        */

        //std::vector<NavNode> pth;  // TODO each needs its own path from the center

        // Load station locations with paths generated from the center to the station
        station_locations[1] = Station(1, Pose2d(1, 1, 0), generatePathToStation(Pose2d(1, 1, 0)));
        station_locations[2] = Station(2, Pose2d(2, 1, 0), generatePathToStation(Pose2d(2, 1, 0)));
        station_locations[3] = Station(3, Pose2d(5, 1, 0), generatePathToStation(Pose2d(5, 1, 0)));

        station_locations[-1] = Station(-1, Pose2d(-2, -1, 0), generatePathToStation(Pose2d(-2, -1, 0)));
        station_locations[-2] = Station(-2, Pose2d(-2, 2, 0), generatePathToStation(Pose2d(-2, 2, 0)));
        station_locations[-3] = Station(-3, Pose2d(2, 2, 0), generatePathToStation(Pose2d(2, 2, 0)));
        station_locations[-4] = Station(-4, Pose2d(2, -1, 0), generatePathToStation(Pose2d(2, -1, 0)));

    return true;
}


void TaskPlanner::timer_callback() {
        if (!this->is_active) {  // If the system is disabled
                return;
        }
        // If the current job list is empty, try to get new orders
        if (current_job_points_.empty()) {
            prep_next_order();
            
            // If its still empty, nothing left to do
            if (current_job_points_.empty()) {
                this->is_active = false;
                return;
            }
        }
        
        // Check if we're at our current target
        if (is_at_target(current_job_points_.front().pose) ||  // This is for a normal action
        current_job_points_.front().action_type != ActionType::normal) {  // This indicates a special action
            current_job_points_.pop();  // Clear the current task
            
            if (!current_job_points_.empty()) {
                switch (current_job_points_.front().action_type) {
                case ActionType::normal:
                        if (current_job_points_.front().is_manual_approach) {
                                manual_go_to_point(current_job_points_.front().pose);  // Command to move to the next one
                        } else {
                                nav2_go_to_point(current_job_points_.front().pose);  // Let Nav2 do it
                        }
                        break;
                case ActionType::advance_state:
                        if (status == JobStatus::ToPickup) {
                                status = JobStatus::FromPickup;
                        } else if (status == JobStatus::FromPickup) {
                                status = JobStatus::ToDestination;
                        } else if (status == JobStatus::ToDestination) {
                                status = JobStatus::FromDestination;
                        } else {
                                // TODO @suraj Print an error. This should not trigger, implies the job wasn't reset, or somehow had more than 4 phases
                        }
                        break;
                case ActionType::pickup:
                        // TODO print a message idk @suraj
                        break;
                case ActionType::dropoff:
                        // TODO print a message idk @suraj
                        break;
                case ActionType::start:
                        // TODO print a message? @suraj
                        status = JobStatus::ToPickup;
                        break;
                case ActionType::finish:
                        // TODO not sure. Print a message? @suraj
                        break;
                default:
                        // This means an invalid action type, trigger an error
                        break;
                }
            }
        }

        if (current_job_points_.front().is_final_approach) {
            int station_id = 0;
            if (!get_visible_station_code(station_id)) return;

            // The apriltag doesn't match the expected value
            if (status == JobStatus::ToPickup && station_id != pickup_station_id) {
                // TODO log an error or abort idk @suraj
            }
            if (status == JobStatus::ToDestination && station_id != dropoff_station_id) {
                // TODO log an error or abort idk @suraj
            }
        }
    }


//ros2 run apriltag_ros apriltag_node --ros-args   -r image_rect:=/camera/image_raw   -r camera_info:=/camera/camera_info   -p family:=36h11   -p size:=0.5   -p max_hamming:=0
bool TaskPlanner::get_visible_station_code(int& tag_id) {
    // Attempt to get an image from the camera
    auto image_msg = rclcpp::wait_for_message<sensor_msgs::msg::Image>("/camera/image_raw", shared_from_this());
    if (!image_msg) {
        RCLCPP_WARN(this->get_logger(), "Failed to get image from /camera/image_raw.");
        return false;  // No image available
    }

    // Convert the ROS image to an OpenCV Mat
    cv::Mat current_image;
    try {
        current_image = cv_bridge::toCvCopy(image_msg, "bgr8")->image;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "CV Bridge error: %s", e.what());
        return false;  // Conversion failed
    }

    // Display the captured image (optional)
    cv::imshow("Current Camera Image", current_image);
    cv::waitKey(1);

    // Attempt to get tag detection data
    auto detection_msg = rclcpp::wait_for_message<apriltag_msgs::msg::AprilTagDetectionArray>(
        "/detections", shared_from_this());
    if (!detection_msg || detection_msg->detections.empty()) {
        RCLCPP_INFO(this->get_logger(), "No AR tags detected.");
        return false;  // No tags detected
    }

    // Use the first detected tag
    const auto& detection = detection_msg->detections[0];
    tag_id = detection.id;  // Update the tag_id with the detected tag's ID

    RCLCPP_INFO(this->get_logger(), "Detected AR tag: ID = %d", tag_id);

    // Optionally draw the detected tag on the image
    std::vector<cv::Point> corners;
    for (const auto& corner : detection.corners) {
        corners.emplace_back(corner.x, corner.y);
    }
    for (size_t i = 0; i < corners.size(); ++i) {
        cv::line(current_image, corners[i], corners[(i + 1) % corners.size()], 
                 cv::Scalar(0, 255, 0), 2);  // Green lines
    }

    // Display the tag ID on the image
    cv::Point center(detection.centre.x, detection.centre.y);
    cv::putText(current_image, std::to_string(tag_id), center, 
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);  // Red text

    // Display the final annotated image (optional)
    cv::imshow("Tag Detection", current_image);
    cv::waitKey(1);

    return true;  // Tag successfully detected
};