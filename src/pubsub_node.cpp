/*
This program maps the input of the joy_node to to run the ros2_controls_demos/example_2
*/

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
// following headers can be used depending on the input and output messag type
// #include "std_msgs/msg/string.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include "std_msgs/msg/string.hpp"             //built-in message type you will use to publish data
// #include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp> // message type diffbot uses
#include <sensor_msgs/msg/joy.hpp>             // message type used by the joy_node

using std::placeholders::_1;

// axes and buttons of 8BitDo Pro 2

enum Axis
{
  LEFT_STICK_X = 0,
  LEFT_STICK_Y = 1,
  LEFT_TRIGGER = 2,
  RIGHT_STICK_X = 3,
  RIGHT_STICK_Y = 4,
  D_PAD_X = 5,
  D_PAD_Y = 6
};
enum Button
{
  Y = 3,
  B = 1,
  A = 2,
  X = 3,
  L = 4,
  R = 5,
  ZL = 6,
  ZR = 7,
  MINUS = 8,
  PLUS = 9,
  LEFT_STICK_BTN = 10,
  RIGHT_STICK_BTN = 11,
  HOME = 12,
  CIRCLE = 13

};

// Create the node class named Joy2Cmd which inherits the attributes
// and methods of the rclcpp::Node class.
class Joy2Cmd : public rclcpp::Node
{
public:
  // Constructor creates a node named joy2cmd.
  Joy2Cmd()
      : Node("joy2cmd")
  {
    // Create the subscription.
    // The topic_callback function executes whenever data is published
    // to the 'joy' topic.
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&Joy2Cmd::topic_callback, this, _1));

    // Publisher publishes String messages to a topic named "/diffbot_base_controller/cmd_vel".
    // The size of the queue is 10 messages.
    publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/diffbot_base_controller/cmd_vel", 10);
  }

private:
  // Receives the published joy input
  void topic_callback(const sensor_msgs::msg::Joy &msg) const
  {
    // Declare message in the publishing message type
    auto message = geometry_msgs::msg::TwistStamped();

    // setting the required to inputs to the output
    message.twist.linear.x = msg.axes[LEFT_STICK_Y];
    message.twist.angular.z = msg.axes[LEFT_STICK_X];

    // Publish the message
    publisher_->publish(message);
  }

  // Declare the subscription attribute
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;

  // Declaration of the publisher_ attribute
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Joy2Cmd>());
  rclcpp::shutdown();
  return 0;
}
