#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "bicopter_msgs/msg/angle_commands.hpp"

namespace bicopter_input_controller{

class BicopterInputController : public rclcpp::Node{
    public:
        BicopterInputController();

    private: 
    // input subscription
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    void joy_callback(const sensor_msgs::msg::Joy joy_input);

    // publisher to angle commands
    rclcpp::Publisher<bicopter_msgs::msg::AngleCommands>::SharedPtr angle_commands_pub_;
    bicopter_msgs::msg::AngleCommands angle_commands_;

};

} // end namespace bicopter_input_controller
