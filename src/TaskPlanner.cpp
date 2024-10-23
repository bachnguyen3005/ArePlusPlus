#include "TaskPlanner.h"

bool TaskPlanner::is_at_target(const Pose2d& target) {
        return false;
}
void TaskPlanner::go_to_point(const Pose2d& target) {

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

        // TODO start event -> navnode special type

        for (const auto& point : pathCtoA) {
                current_job_points_.push(point);
        }

        // TODO pickup event
        // TODO change state event
    
        for (auto it = pathCtoA.rbegin() + 1; it != pathCtoA.rend(); ++it) {
                current_job_points_.push(*it);
        }

        // TODO change state event

        for (const auto& point : pathCtoB) {
                current_job_points_.push(point);
        }

        // TODO dropoff event
        // TODO change state event

        for (auto it = pathCtoB.rbegin() + 1; it != pathCtoB.rend(); ++it) {
                current_job_points_.push(*it);
        }

        // TODO finished event

        package_id = new_order.product_id;
        dropoff_station_id = new_order.station_id;
        pickup_station_id = product_locations[new_order.product_id];
}

bool TaskPlanner::load_locations_from_file() {

}

bool get_visible_station_code(int& tag_id) {

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

        // TODO handle special nav_nodes
        // Pickup/dropoff
        
        // Check if we're at our current target
        if (is_at_target(current_job_points_.front().pose)) {
            current_job_points_.pop();  // Clear the current task
            
            if (!current_job_points_.empty()) {
                if (current_job_points_.front().is_manual_approach) {
                        go_to_point(current_job_points_.front().pose);  // Command to move to the next one
                } else {
                        // TODO NAV2 approach
                }
            }
        }

        if (current_job_points_.front().is_final_approach) {
            int station_id = 0;
            if (!get_visible_station_code(station_id)) return;
            // TODO check against expected id
            //if (station_id != current_job_target_id)
        }
    }

bool TaskPlanner::get_visible_station_code(int& tag_id) {
        
}