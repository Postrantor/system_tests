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

#ifndef SUBSCRIBE_HELPER_HPP_
#define SUBSCRIBE_HELPER_HPP_

#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

/**
 * @brief 订阅消息并处理接收到的消息 (Subscribe to messages and handle received messages)
 *
 * @tparam T 消息类型 (Message type)
 * @param node 共享指针，指向一个 rclcpp::Node 对象 (Shared pointer to an rclcpp::Node object)
 * @param message_type 消息类型的字符串表示 (String representation of the message type)
 * @param expected_messages 预期接收到的消息列表 (List of expected received messages)
 * @param received_messages 用于记录已接收到的消息的布尔值向量 (Boolean vector for recording received messages)
 * @return rclcpp::SubscriptionBase::SharedPtr 返回订阅对象的共享指针 (Return a shared pointer to the subscription
 * object)
 */
template <typename T>
rclcpp::SubscriptionBase::SharedPtr subscribe(rclcpp::Node::SharedPtr node,
                                              const std::string& message_type,
                                              const std::vector<typename T::SharedPtr>& expected_messages,
                                              std::vector<bool>& received_messages) {
  // 初始化 received_messages 向量为与预期消息数量相同的 false 值 (Initialize the received_messages vector with false
  // values equal to the number of expected messages)
  received_messages.assign(expected_messages.size(), false);

  // 定义回调函数，用于处理接收到的消息 (Define the callback function for handling received messages)
  auto callback = [&expected_messages, &received_messages](const typename T::ConstSharedPtr received_message) -> void {
    // 在预期消息向量中查找接收到的消息 (Find the received message in the vector of expected messages)
    auto received = received_messages.begin();
    bool known_message = false;
    size_t index = 0;
    for (auto expected_message : expected_messages) {
      if (*received_message == *expected_message) {
        *received = true;
        printf("received message #%zu of %zu\n", index + 1, expected_messages.size());
        known_message = true;
        break;
      }
      ++received;
      ++index;
    }
    // 如果接收到的消息与任何预期消息都不匹配，则抛出异常 (If the received message does not match any expected message,
    // throw an exception)
    if (!known_message) {
      throw std::runtime_error("received message does not match any expected message");
    }

    // 当所有预期消息都已接收时，关闭节点 (Shutdown the node when all expected messages have been received)
    for (auto received_msg : received_messages) {
      if (!received_msg) {
        return;
      }
    }
    rclcpp::shutdown();
  };

  // 设置 QoS 策略 (Set the QoS policy)
  auto qos = rclcpp::QoS(rclcpp::KeepLast(expected_messages.size()));

  // 创建订阅并返回共享指针 (Create the subscription and return a shared pointer)
  return node->create_subscription<T>(std::string("test/message/") + message_type, qos, callback);
}

#endif  // SUBSCRIBE_HELPER_HPP_
