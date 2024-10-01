#include "geometry_msgs/msg/pose.hpp"

struct Pose2d {
public:
    double pos_x = 0;
    double pos_y = 0;
    double yaw = 0;

    Pose2d() = default;
    Pose2d(double x, double y, double yaw) : pos_x(x), pos_y(y), yaw(yaw) {}

    geometry_msgs::msg::Pose to_pose() const {
        geometry_msgs::msg::Pose pose;
        pose.position.x = static_cast<double>(pos_x);
        pose.position.y = static_cast<double>(pos_y);
        pose.position.z = 0.0;  

        double cy = std::cos(yaw * 0.5);
        double sy = std::sin(yaw * 0.5);
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = sy;
        pose.orientation.w = cy;

        return pose;
    }
};
