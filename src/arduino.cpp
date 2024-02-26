#include <libserial/SerialPort.h>
#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

// following headers can be used depending on the input and output messag type
#include "rclcpp/rclcpp.hpp"

int main()
{
   // Instantiate a Serial Port and a Serial Stream object.
   LibSerial::SerialPort serial_conn_;
   // std::string serial_device = "/dev/ttyACM0";
   std::string serial_device = "/dev/serial/by-path/pci-0000:00:14.0-usb-0:6:1.0";
   
   serial_conn_.Open(serial_device);
   printf("We are in\n");
   serial_conn_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
   std::string received = "";
   int32_t timeout_ms_ = 1000;
   sleep(5);

   for (int i = 0; i < 3; i++)
   {
      printf("%d\n" ,i);
      // serial_conn_.FlushIOBuffers();
      // serial_conn_.Write("1");
      serial_conn_.FlushIOBuffers();
      serial_conn_.Write("3");
      sleep(1);
      // serial_conn_.ReadLine(received, '\n', timeout_ms_);
      // std::cout << received  << std::endl;
      sleep(2);

      // serial_conn_.FlushIOBuffers();
      // serial_conn_.Write("2");
      serial_conn_.FlushIOBuffers();
      serial_conn_.Write("4");
      sleep(1);
      // serial_conn_.ReadLine(received, '\n', timeout_ms_);
      // std::cout << received  << std::endl;
      sleep(2);
   }

   serial_conn_.Close();
   printf("We are out\n");
   sleep(1);

   //     <param name="left_wheel_name">left_wheel_joint</param>
   //   <param name="right_wheel_name">right_wheel_joint</param>
   //   <param name="loop_rate">30</param>
   //   <param name="device">/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0</param>
   //   <param name="baud_rate">57600</param>
   //   <param name="timeout_ms">1000</param>
   //   <param name="enc_counts_per_rev">3436</param>
   //   <param name="pid_p">20</param>
   //   <param name="pid_d">12</param>
   //   <param name="pid_i">0</param>
   //   <param name="pid_o">50</param>

   return 0;
}