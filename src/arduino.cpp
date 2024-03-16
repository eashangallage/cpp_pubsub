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
   serial_conn_.SetBaudRate(LibSerial::BaudRate::BAUD_9600);
   std::string received = "";
   int32_t timeout_ms_ = 1000;
   sleep(5);

   for (int i = 0; i < 3; i++)
   {
      printf("%d\n" ,i);
      serial_conn_.FlushIOBuffers();
      serial_conn_.Write("1");
      // serial_conn_.FlushIOBuffers();
      // serial_conn_.Write("3");
      sleep(10);
      // serial_conn_.ReadLine(received, '\n', timeout_ms_); // getting feedback
      // std::cout << received  << std::endl;
      // sleep(2);

      serial_conn_.FlushIOBuffers();
      serial_conn_.Write("2");
      // serial_conn_.FlushIOBuffers();
      // serial_conn_.Write("4");
      // sleep(1);
      // serial_conn_.ReadLine(received, '\n', timeout_ms_);
      // std::cout << received  << std::endl;
      sleep(10);
   }

   serial_conn_.Close();
   printf("We are out\n");
   sleep(1);

   return 0;
}