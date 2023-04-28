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

#ifndef SUBSCRIBE_BASIC_TYPES_HPP_
#define SUBSCRIBE_BASIC_TYPES_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/msg/builtins.hpp"
#include "test_msgs/msg/constants.hpp"
#include "test_msgs/msg/defaults.hpp"
#include "test_msgs/msg/empty.hpp"

/**
 * @brief 订阅空消息 (Subscribe to Empty messages)
 *
 * @param node 共享指针，指向节点对象 (Shared pointer to the node object)
 * @param message_type 消息类型字符串 (Message type string)
 * @param messages_expected 预期接收的消息列表 (List of expected received messages)
 * @param received_messages 布尔值向量，用于记录已接收消息的状态 (Boolean vector for recording the status of received messages)
 * @return 返回订阅基类的共享指针 (Return a shared pointer to the SubscriptionBase class)
 */
rclcpp::SubscriptionBase::SharedPtr subscribe_empty(
    rclcpp::Node::SharedPtr node,
    const std::string& message_type,
    const std::vector<test_msgs::msg::Empty::SharedPtr>& messages_expected,
    std::vector<bool>& received_messages);

/**
 * @brief 订阅基本类型消息 (Subscribe to BasicTypes messages)
 *
 * @param node 共享指针，指向节点对象 (Shared pointer to the node object)
 * @param message_type 消息类型字符串 (Message type string)
 * @param messages_expected 预期接收的消息列表 (List of expected received messages)
 * @param received_messages 布尔值向量，用于记录已接收消息的状态 (Boolean vector for recording the status of received messages)
 * @return 返回订阅基类的共享指针 (Return a shared pointer to the SubscriptionBase class)
 */
rclcpp::SubscriptionBase::SharedPtr subscribe_basic_types(
    rclcpp::Node::SharedPtr node,
    const std::string& message_type,
    const std::vector<test_msgs::msg::BasicTypes::SharedPtr>& messages_expected,
    std::vector<bool>& received_messages);

/**
 * @brief 订阅内置类型消息 (Subscribe to Builtins messages)
 *
 * @param node 共享指针，指向节点对象 (Shared pointer to the node object)
 * @param message_type 消息类型字符串 (Message type string)
 * @param messages_expected 预期接收的消息列表 (List of expected received messages)
 * @param received_messages 布尔值向量，用于记录已接收消息的状态 (Boolean vector for recording the status of received messages)
 * @return 返回订阅基类的共享指针 (Return a shared pointer to the SubscriptionBase class)
 */
rclcpp::SubscriptionBase::SharedPtr subscribe_builtins(
    rclcpp::Node::SharedPtr node,
    const std::string& message_type,
    const std::vector<test_msgs::msg::Builtins::SharedPtr>& messages_expected,
    std::vector<bool>& received_messages);

/**
 * @brief 订阅常量类型消息 (Subscribe to Constants messages)
 *
 * @param node 共享指针，指向节点对象 (Shared pointer to the node object)
 * @param message_type 消息类型字符串 (Message type string)
 * @param messages_expected 预期接收的消息列表 (List of expected received messages)
 * @param received_messages 布尔值向量，用于记录已接收消息的状态 (Boolean vector for recording the status of received messages)
 * @return 返回订阅基类的共享指针 (Return a shared pointer to the SubscriptionBase class)
 */
rclcpp::SubscriptionBase::SharedPtr subscribe_constants(
    rclcpp::Node::SharedPtr node,
    const std::string& message_type,
    const std::vector<test_msgs::msg::Constants::SharedPtr>& messages_expected,
    std::vector<bool>& received_messages);

/**
 * @brief 订阅默认类型消息 (Subscribe to Defaults messages)
 *
 * @param node 共享指针，指向节点对象 (Shared pointer to the node object)
 * @param message_type 消息类型字符串 (Message type string)
 * @param messages_expected 预期接收的消息列表 (List of expected received messages)
 * @param received_messages 布尔值向量，用于记录已接收消息的状态 (Boolean vector for recording the status of received messages)
 * @return 返回订阅基类的共享指针 (Return a shared pointer to the SubscriptionBase class)
 */
rclcpp::SubscriptionBase::SharedPtr subscribe_defaults(
    rclcpp::Node::SharedPtr node,
    const std::string& message_type,
    const std::vector<test_msgs::msg::Defaults::SharedPtr>& messages_expected,
    std::vector<bool>& received_messages);

#endif  // SUBSCRIBE_BASIC_TYPES_HPP_
