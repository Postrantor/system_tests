// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "test_communication/msg/u_int32.hpp"

/**
 * @file main.cpp
 * @brief 示例代码，用于测试ROS2订阅者和发布者的有效数据交互 (Example code for testing valid data interaction between
 * ROS2 subscriber and publisher)
 */
int main(int argc, char** argv) {
  // 初始化ROS2节点 (Initialize ROS2 node)
  rclcpp::init(argc, argv);

  // 记录程序开始时间 (Record the start time of the program)
  auto start = std::chrono::steady_clock::now();

  // 创建一个名为"test_subscription_valid_data"的共享节点 (Create a shared node named "test_subscription_valid_data")
  auto node = rclcpp::Node::make_shared("test_subscription_valid_data");

  // 定义回调函数，当收到消息时触发 (Define callback function to be triggered when a message is received)
  auto callback = [](const test_communication::msg::UInt32::ConstSharedPtr received_message) -> void {
    // 打印接收到的消息序号 (Print the sequence number of the received message)
    printf("received message #%u\n", received_message->data);

    // 如果接收到的消息数据为0，则抛出异常 (If the received message data is 0, throw an exception)
    if (received_message->data == 0) {
      fprintf(stderr, "received message data was never sent\n");
      rclcpp::shutdown();
      throw std::runtime_error("received message data was never sent");
    }
  };

  // 创建订阅者，订阅名为"test_subscription_valid_data"的话题，队列大小为10 (Create a subscriber, subscribe to the topic
  // named "test_subscription_valid_data", and set the queue size to 10)
  auto subscriber =
      node->create_subscription<test_communication::msg::UInt32>("test_subscription_valid_data", 10, callback);

  // 设置消息发送频率为5Hz (Set message sending frequency to 5Hz)
  rclcpp::WallRate message_rate(5);
  {
    // 创建一个名为"test_subscription_valid_data"的发布者，队列大小为10 (Create a publisher named
    // "test_subscription_valid_data" with a queue size of 10)
    auto publisher = node->create_publisher<test_communication::msg::UInt32>("test_subscription_valid_data", 10);

    // 等待一段时间，以便订阅者准备好接收消息 (Wait for a while for the subscriber to be ready to receive messages)
    message_rate.sleep();

    uint32_t index = 1;
    // 发布几个数据大于0的消息 (Publish a few messages with data greater than 0)
    while (rclcpp::ok() && index <= 5) {
      printf("publishing message #%u\n", index);
      auto msg = std::make_unique<test_communication::msg::UInt32>();
      msg->data = index;
      publisher->publish(std::move(msg));
      ++index;
      message_rate.sleep();
      rclcpp::spin_some(node);
    }

    message_rate.sleep();
    rclcpp::spin_some(node);
  }
  // 当发布者超出范围时，订阅者不再接收回调 (When the publisher goes out of scope, the subscriber no longer receives
  // callbacks)

  message_rate.sleep();
  rclcpp::spin_some(node);

  // 记录程序结束时间 (Record the end time of the program)
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<float> diff = (end - start);
  printf("published and subscribed for %f seconds\n", diff.count());

  // 关闭ROS2节点 (Shutdown ROS2 node)
  rclcpp::shutdown();
  return 0;
}
