#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tuple>

using namespace std::chrono_literals;

class DistanceController : public rclcpp::Node {
public:    
    DistanceController() : Node("distance_controller") {
        odom_subscription = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10, std::bind(&DistanceController::odomCallback, this, std::placeholders::_1));

        velocity_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        timer = this->create_wall_timer(
            20ms, std::bind(&DistanceController::timerCallback, this));

        lastExecutionTime = std::chrono::high_resolution_clock::now();
    }
private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "X: %f", msg->pose.pose.position.x);
        currentPosition = msg->pose.pose.position;
    }

    void timerCallback() {

        float deltaTime = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::high_resolution_clock::now() - lastExecutionTime).count() / 1000000000.0;

        float newErrorDistance = std::get<0>(goals[goalCounter]) - currentPosition.x;

        // Error for the proportional term
        float errorProportional = errorDistance;

        // Error for the integral term.
        errorIntegral = errorIntegral + newErrorDistance * deltaTime;

        // Error for the derivative term.
        float errorDerivative = (newErrorDistance - errorDistance) / deltaTime;

        float speedFactor = Kp * errorProportional + Ki * errorIntegral + Kd * errorDerivative;

        cmd_vel_msg.linear.x = (newErrorDistance > 0.01) ? speedFactor * 0.5 : 0.0;

        velocity_publisher->publish(cmd_vel_msg);

        if (newErrorDistance < 0.01) {
            
            goalCounter++;

            rclcpp::sleep_for(std::chrono::seconds(1));

            if (goalCounter == goals.size()) {
                this->timer->cancel();
                rclcpp::shutdown();
                return;
            }
        }

        errorDistance = newErrorDistance;
        lastExecutionTime = std::chrono::high_resolution_clock::now();
    }

    // PID gains
    float Kp = 1.5;
    float Ki = 0.1;
    float Kd = 0.5;

    geometry_msgs::msg::Point currentPosition;

    std::chrono::time_point<std::chrono::system_clock> lastExecutionTime;

    size_t goalCounter = 0;
    std::vector<std::tuple<float, float>> goals = {{1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}};

    float errorIntegral;
    float errorDistance;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher;
    rclcpp::TimerBase::SharedPtr timer;

    geometry_msgs::msg::Twist cmd_vel_msg;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DistanceController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
