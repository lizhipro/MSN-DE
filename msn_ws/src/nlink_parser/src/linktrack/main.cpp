#include <ros/ros.h>

#include "init.h"
#include "init_serial.h"
#include "protocol_extracter/nprotocol_extracter.h"

#include <iomanip>
#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <memory>
#include <atomic>
#include <mutex>
std::vector<std::shared_ptr<serial::Serial>> serial_ports;    // 主线程维护的串口列表
std::vector<std::shared_ptr<serial::Serial>> buffer_ports;  // 子线程的缓冲区
bool has_serial_changed = false;  // 标志是否有端口变化
std::mutex serial_ports_lock;

void printHexData(const std::string &data) {
  if (!data.empty()) {
    std::cout << "data received: ";
    for (int i = 0; i < data.size(); ++i) {
      std::cout << std::hex << std::setfill('0') << std::setw(2)
                << int(uint8_t(data.at(i))) << " ";
    }
    std::cout << std::endl;
  }
}

// 检查串口端口的变化，更新 serial_ports
void monitorPorts() {
  while (ros::ok()) {
    // 扫描当前的串口端口
    auto new_serial_ports = initSerial();
    if (new_serial_ports.empty()) {
      ROS_WARN("No Linktrack UWB devices found, rescan after 1 second...");
    }

    if (new_serial_ports.size() != buffer_ports.size()) {
      ROS_WARN("UWB device num change, current is %d", new_serial_ports.size());
      // 更新缓冲区，并设置端口变化标志
      std::lock_guard<std::mutex> lock(serial_ports_lock);
      buffer_ports = new_serial_ports;
      has_serial_changed = true;
    }
    ros::Duration(1.0).sleep();  // 每秒扫描一次
  }
  ROS_INFO("ROS stop.");
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "linktrack_parser");
  ros::NodeHandle nh;
  // 启动子线程监听端口变化
  std::thread monitor_thread(monitorPorts);
  
  NProtocolExtracter protocol_extraction;
  linktrack::Init init(&protocol_extraction, serial_ports);
  ros::Rate loop_rate(100 ); //HZ
  while (ros::ok()) {
    {
      std::lock_guard<std::mutex> lock(serial_ports_lock);
      if (has_serial_changed) {
        for (auto port : serial_ports) {
          try {
            port->close();
          } catch (serial::IOException& e) {
            ROS_ERROR("Failed to close port: %s", e.what());
            continue;
          }
        }
        serial_ports = buffer_ports;  // 交换缓冲区与主线程使用的列表
        has_serial_changed = false;
      }
    }

    // ROS_ERROR("Debug.........serial_ports.size: %d!", serial_ports.size());
    for (size_t i = 0; i < serial_ports.size(); ++i) {
      if (serial_ports[i] == nullptr) {
        ROS_ERROR("Read serial port data error. serial is NULL!");
        continue;
      }
      if (!serial_ports[i]->isOpen()) {
        try {
          serial_ports[i]->open();
        } catch (serial::IOException& e) {
          ROS_ERROR("Failed to open port: %s", e.what());
          continue;
        }
      }
      if (serial_ports[i]->isOpen()) {
        try {
          // 插入串口后，不会立即available
          auto available_bytes = serial_ports[i]->available();
          std::string str_received;
          if (available_bytes) {
            // ROS_ERROR("Debug.........read from port: %s!", serial_ports[i]->getPort().c_str());
            serial_ports[i]->read(str_received, available_bytes);
            protocol_extraction.AddNewData(str_received);
          }
        } catch (serial::IOException& e) {
          ROS_ERROR("Read serial port data error: %s", e.what());
          serial_ports[i]->close();  // 关闭出错的端口
        }
      }
    }
    
    ros::spinOnce();
    loop_rate.sleep();
  }

  monitor_thread.join();

  return EXIT_SUCCESS;
}
