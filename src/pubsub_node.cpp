#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <libserial/SerialPort.h>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

enum Axis
{
  LEFT_STICK_X = 0,
  LEFT_STICK_Y = 1,
  RIGHT_STICK_X = 2,
  RIGHT_STICK_Y = 3,
  D_PAD_Y = 4,
  D_PAD_X = 5
};
enum Button
{
  Y = 0,
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

LibSerial::SerialPort serial_conn_;

class Joy2Cmd : public rclcpp::Node
{
public:
  Joy2Cmd()
      : Node("joy2cmd")
  {
    std::string serial_device = "/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0";
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(LibSerial::BaudRate::BAUD_9600);

    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&Joy2Cmd::topic_callback, this, _1));

    diff_cmd = this->create_publisher<geometry_msgs::msg::Twist>("/diffbot_base_controller/cmd_vel_unstamped", 10);
  }

private:
  void topic_callback(const sensor_msgs::msg::Joy &msg) const
  {
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

    auto cmd_ = geometry_msgs::msg::Twist();

    double fwd = msg.axes[LEFT_STICK_Y];
    double turn = msg.axes[RIGHT_STICK_X] * 1.5;

    cmd_.linear.x = fwd;
    cmd_.angular.z = turn;

    diff_cmd->publish(cmd_);

    if (msg.buttons[R] == 1)
    {
      serial_conn_.FlushIOBuffers();
      serial_conn_.Write("1");
      clock->sleep_for(2ms);
    }
    else if (msg.buttons[ZR] == 1)
    {
      serial_conn_.FlushIOBuffers();
      serial_conn_.Write("2");
      clock->sleep_for(2ms);
    }
    else if (msg.buttons[L] == 1)
    {
      serial_conn_.FlushIOBuffers();
      serial_conn_.Write("3");
      clock->sleep_for(2ms);
    }
    else if (msg.buttons[ZL] == 1)
    {
      serial_conn_.FlushIOBuffers();
      serial_conn_.Write("4");
      clock->sleep_for(2ms);
    }
    else if (msg.buttons[B] == 1)
    {
      serial_conn_.FlushIOBuffers();
      serial_conn_.Write("0");
      clock->sleep_for(2ms);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr diff_cmd;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Joy2Cmd>());
  rclcpp::shutdown();
  return 0;
}
