#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <vector>

struct Waypoint {
    double latitude;
    double longitude;
    double altitude;
};

class WaypointNavNode : public rclcpp::Node {
public:
    WaypointNavNode() : Node("waypoint_nav_node"), current_waypoint_(0) {
        velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/drone/cmd_vel", 10);
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/drone/gps", 10, std::bind(&WaypointNavNode::gps_callback, this, std::placeholders::_1));
        obstacle_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
            "/drone/obstacle_data", 10, std::bind(&WaypointNavNode::obstacle_callback, this, std::placeholders::_1));

        
        waypoints_ = {
            {37.773972, -122.431297, 10.0},
            {37.774159, -122.431587, 15.0},
            {37.774400, -122.431947, 20.0},
            {37.774150, -122.432297, 15.0},
            {37.773900, -122.431587, 10.0},
            {37.773972, -122.431297, 5.0}
        };
    }

private:
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        
        if (msg->status.status < 0) {
            RCLCPP_WARN(this->get_logger(), "No GPS fix available.");
            return;
        }

        
        if (current_waypoint_ < waypoints_.size() && has_reached_waypoint(msg->latitude, msg->longitude)) {
            RCLCPP_INFO(this->get_logger(), "Reached waypoint %d", current_waypoint_);
            current_waypoint_++;
        }

        if (current_waypoint_ < waypoints_.size()) {
            navigate_to_waypoint(waypoints_[current_waypoint_]);
        } else {
            RCLCPP_INFO(this->get_logger(), "Completed all waypoints.");
        }
    }

    void obstacle_callback(const sensor_msgs::msg::Range::SharedPtr msg) {
        if (msg->range < 1.0) {
            RCLCPP_WARN(this->get_logger(), "Obstacle detected! Executing avoidance maneuver.");
            perform_avoidance();
        }
    }

    bool has_reached_waypoint(double latitude, double longitude) {
        
        double distance = sqrt(pow(latitude - waypoints_[current_waypoint_].latitude, 2) +
                               pow(longitude - waypoints_[current_waypoint_].longitude, 2));
        return distance < 0.0001;  // Adjust this threshold as needed
    }

    void navigate_to_waypoint(const Waypoint& waypoint) {
        geometry_msgs::msg::Twist cmd_vel;
        
       
        cmd_vel.linear.z = (waypoint.altitude > 0) ? 1.0 : -1.0; // Ascend/Descend
        cmd_vel.linear.x = 1.0; 
        cmd_vel.angular.z = 0.0; 

        velocity_pub_->publish(cmd_vel);
    }

    void perform_avoidance() {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = -1.0; 
        cmd_vel.angular.z = 0.5;  

        velocity_pub_->publish(cmd_vel);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr obstacle_sub_;
    std::vector<Waypoint> waypoints_;
    int current_waypoint_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointNavNode>());
    rclcpp::shutdown();
    return 0;
}

