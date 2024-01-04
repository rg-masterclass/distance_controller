#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

class DistanceDisplay : public rclcpp::Node {
public:    
    DistanceDisplay() : Node("distance_display") {
        odom_subscription = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10, std::bind(&DistanceDisplay::odomCallback, this, std::placeholders::_1));
    }
private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "X: %2f, Y: %2f, Z: %2f, W: %2f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DistanceDisplay>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
