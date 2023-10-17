// #include <cstdio>

#include "sensor_msgs/msg/joy.hpp"

#include "rclcpp/rclcpp.hpp"
#include "bicopter_msgs/msg/angle_commands.hpp"

using std::placeholders::_1;

class InputController : public rclcpp::Node{

  public:
    InputController(): Node("bicopter_input_controller"){

      RCLCPP_INFO(this->get_logger(), "Initializing...");

      // initialize subscription
      joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&InputController::joy_callback, this, _1));

      // initialize publisher
      angle_commands_pub_ = this->create_publisher<bicopter_msgs::msg::AngleCommands>("/bicopter/angle_commands", 10);

      RCLCPP_INFO(this->get_logger(), "Initialized.");
    }

  private:

    void joy_callback(const sensor_msgs::msg::Joy msg){

      // roll, pitch, yaw_rate, throttle
      angle_commands_.r = -msg.axes[3];
      angle_commands_.p = msg.axes[4];
      angle_commands_.w_z = msg.axes[0];
      angle_commands_.v_z = msg.axes[1];

      angle_commands_pub_->publish(angle_commands_);
    }

    // input subscription
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    // publisher to angle commands
    rclcpp::Publisher<bicopter_msgs::msg::AngleCommands>::SharedPtr angle_commands_pub_;
    bicopter_msgs::msg::AngleCommands angle_commands_ = bicopter_msgs::msg::AngleCommands();

};

int main(int argc, char ** argv)
{

  // init, spin, shutdown
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InputController>());
  rclcpp::shutdown();

  return 0;
}
