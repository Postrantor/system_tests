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

#ifndef SUBSCRIBE_ARRAY_TYPES_HPP_
#define SUBSCRIBE_ARRAY_TYPES_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "test_msgs/msg/arrays.hpp"
#include "test_msgs/msg/bounded_plain_sequences.hpp"
#include "test_msgs/msg/bounded_sequences.hpp"
#include "test_msgs/msg/multi_nested.hpp"
#include "test_msgs/msg/nested.hpp"
#include "test_msgs/msg/unbounded_sequences.hpp"

/**
 * @brief 订阅数组消息 (Subscribe to array messages)
 *
 * @param node 共享指针的节点对象 (Shared pointer to the node object)
 * @param message_type 消息类型字符串 (Message type string)
 * @param expected_messages 预期接收到的数组消息列表 (List of expected array messages to be received)
 * @param received_messages 用于记录已接收消息的布尔向量 (Boolean vector for tracking received messages)
 * @return 返回共享指针的订阅对象 (Returns a shared pointer to the subscription object)
 */
rclcpp::SubscriptionBase::SharedPtr subscribe_arrays(
    rclcpp::Node::SharedPtr node,
    const std::string& message_type,
    const std::vector<test_msgs::msg::Arrays::SharedPtr>& expected_messages,
    std::vector<bool>& received_messages);

/**
 * @brief 订阅无界序列消息 (Subscribe to unbounded sequence messages)
 *
 * @param node 共享指针的节点对象 (Shared pointer to the node object)
 * @param message_type 消息类型字符串 (Message type string)
 * @param expected_messages 预期接收到的无界序列消息列表 (List of expected unbounded sequence messages to be received)
 * @param received_messages 用于记录已接收消息的布尔向量 (Boolean vector for tracking received messages)
 * @return 返回共享指针的订阅对象 (Returns a shared pointer to the subscription object)
 */
rclcpp::SubscriptionBase::SharedPtr subscribe_unbounded_sequences(
    rclcpp::Node::SharedPtr node,
    const std::string& message_type,
    const std::vector<test_msgs::msg::UnboundedSequences::SharedPtr>& expected_messages,
    std::vector<bool>& received_messages);

/**
 * @brief 订阅有界普通序列消息 (Subscribe to bounded plain sequence messages)
 *
 * @param node 共享指针的节点对象 (Shared pointer to the node object)
 * @param message_type 消息类型字符串 (Message type string)
 * @param expected_messages 预期接收到的有界普通序列消息列表 (List of expected bounded plain sequence messages to be
 * received)
 * @param received_messages 用于记录已接收消息的布尔向量 (Boolean vector for tracking received messages)
 * @return 返回共享指针的订阅对象 (Returns a shared pointer to the subscription object)
 */
rclcpp::SubscriptionBase::SharedPtr subscribe_bounded_plain_sequences(
    rclcpp::Node::SharedPtr node,
    const std::string& message_type,
    const std::vector<test_msgs::msg::BoundedPlainSequences::SharedPtr>& expected_messages,
    std::vector<bool>& received_messages);

/**
 * @brief 订阅有界序列消息 (Subscribe to bounded sequence messages)
 *
 * @param node 共享指针的节点对象 (Shared pointer to the node object)
 * @param message_type 消息类型字符串 (Message type string)
 * @param expected_messages 预期接收到的有界序列消息列表 (List of expected bounded sequence messages to be received)
 * @param received_messages 用于记录已接收消息的布尔向量 (Boolean vector for tracking received messages)
 * @return 返回共享指针的订阅对象 (Returns a shared pointer to the subscription object)
 */
rclcpp::SubscriptionBase::SharedPtr subscribe_bounded_sequences(
    rclcpp::Node::SharedPtr node,
    const std::string& message_type,
    const std::vector<test_msgs::msg::BoundedSequences::SharedPtr>& expected_messages,
    std::vector<bool>& received_messages);

/**
 * @brief 订阅多重嵌套消息 (Subscribe to multi-nested messages)
 *
 * @param node 共享指针的节点对象 (Shared pointer to the node object)
 * @param message_type 消息类型字符串 (Message type string)
 * @param expected_messages 预期接收到的多重嵌套消息列表 (List of expected multi-nested messages to be received)
 * @param received_messages 用于记录已接收消息的布尔向量 (Boolean vector for tracking received messages)
 * @return 返回共享指针的订阅对象 (Returns a shared pointer to the subscription object)
 */
rclcpp::SubscriptionBase::SharedPtr subscribe_multi_nested(
    rclcpp::Node::SharedPtr node,
    const std::string& message_type,
    const std::vector<test_msgs::msg::MultiNested::SharedPtr>& expected_messages,
    std::vector<bool>& received_messages);

/**
 * @brief 订阅嵌套消息 (Subscribe to nested messages)
 *
 * @param node 共享指针的节点对象 (Shared pointer to the node object)
 * @param message_type 消息类型字符串 (Message type string)
 * @param expected_messages 预期接收到的嵌套消息列表 (List of expected nested messages to be received)
 * @param received_messages 用于记录已接收消息的布尔向量 (Boolean vector for tracking received messages)
 * @return 返回共享指针的订阅对象 (Returns a shared pointer to the subscription object)
 */
rclcpp::SubscriptionBase::SharedPtr subscribe_nested(
    rclcpp::Node::SharedPtr node,
    const std::string& message_type,
    const std::vector<test_msgs::msg::Nested::SharedPtr>& expected_messages,
    std::vector<bool>& received_messages);

#endif  // SUBSCRIBE_ARRAY_TYPES_HPP_
