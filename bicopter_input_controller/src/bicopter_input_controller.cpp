#include "bicopter_input_controller/bicopter_input_controller.hpp"

using namespace bicopter_input_controller;
using std::placeholders::_1;


BicopterInputController::BicopterInputController()
: Node("bicopter_input_controller"){

    RCLCPP_INFO(this->get_logger(), "Initializing...");

    // initialize subscription
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&BicopterInputController::joy_callback, this, _1));

    // initialize publisher
    angle_commands_pub_ = this->create_publisher<bicopter_msgs::msg::AngleCommands>("/bicopter/angle_commands", 10);
  
  RCLCPP_INFO(this->get_logger(), "Initialized!");
}

void BicopterInputController::joy_callback(const sensor_msgs::msg::Joy msg){

    // roll, pitch, yaw_rate, throttle
    angle_commands_.r = -msg.axes[3];
    angle_commands_.p = msg.axes[4];
    angle_commands_.w_z = msg.axes[0];
    angle_commands_.v_z = msg.axes[1];

    angle_commands_pub_->publish(angle_commands_);
}

int main(int argc, char ** argv)
{

  // init, spin, shutdown
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BicopterInputController>());
  rclcpp::shutdown();

  return 0;
}
