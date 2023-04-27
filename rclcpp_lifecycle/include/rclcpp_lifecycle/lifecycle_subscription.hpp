// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP_LIFECYCLE__LIFECYCLE_SUBSCRIPTION_HPP_
#define RCLCPP_LIFECYCLE__LIFECYCLE_SUBSCRIPTION_HPP_

#include <memory>
#include <string>
#include <utility>

#include "rclcpp/logging.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/any_subscription_callback.hpp"

#include "rclcpp_lifecycle/managed_entity.hpp"


namespace rclcpp_lifecycle
{
/// brief child class of rclcpp Subscriber class.
/**
 * Overrides all subscription functions to check for enabled/disabled state.
 */
template<
  typename MessageT,
  typename AllocatorT = std::allocator<void>,
  /// MessageT::custom_type if MessageT is a TypeAdapter,
  /// otherwise just MessageT.
  typename SubscribedT = typename rclcpp::TypeAdapter<MessageT>::custom_type,
  /// MessageT::ros_message_type if MessageT is a TypeAdapter,
  /// otherwise just MessageT.
  typename ROSMessageT = typename rclcpp::TypeAdapter<MessageT>::ros_message_type,
  typename MessageMemoryStrategyT = rclcpp::message_memory_strategy::MessageMemoryStrategy<
    ROSMessageT,
    AllocatorT
  >>
class LifecycleSubscription : public SimpleManagedEntity,
  public rclcpp::Subscription<MessageT, AllocatorT, SubscribedT, ROSMessageT, MessageMemoryStrategyT>
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(LifecycleSubscription)

  using MessageAllocTraits = rclcpp::allocator::AllocRebind<MessageT, AllocatorT>;
  using MessageAlloc = typename MessageAllocTraits::allocator_type;
  using MessageDeleter = rclcpp::allocator::Deleter<MessageAlloc, MessageT>;
  using MessageUniquePtr = std::unique_ptr<MessageT, MessageDeleter>;

  LifecycleSubscription(
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    const rosidl_message_type_support_t & type_support_handle,
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    rclcpp::AnySubscriptionCallback<MessageT, AllocatorT> callback,
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & options,
    typename MessageMemoryStrategyT::SharedPtr message_memory_strategy,
    std::shared_ptr<rclcpp::topic_statistics::SubscriptionTopicStatistics<ROSMessageT>> subscription_topic_statistics = nullptr)
  : rclcpp::Subscription<MessageT, AllocatorT, SubscribedT, ROSMessageT, MessageMemoryStrategyT>(node_base, type_support_handle, topic_name, qos, callback, options, message_memory_strategy, subscription_topic_statistics),
    should_log_(true),
    logger_(rclcpp::get_logger("LifecycleSubscription"))
  {
  }

  ~LifecycleSubscription() {}

  /// LifecycleSubscription handle_message function
  /**
   * The handle_message function checks whether the communication
   * was enabled or disabled and forwards the message
   * to the actual rclcpp Subscription base class
   */
  virtual void
  handle_message(std::shared_ptr<void> & message,
    const rclcpp::MessageInfo & message_info)
  {
    if (!this->is_activated()) {
      log_subscription_not_enabled();
      return;
    }
    rclcpp::Subscription<MessageT, AllocatorT, SubscribedT, ROSMessageT, MessageMemoryStrategyT>::handle_message(message, message_info);
  }

  void
  on_activate() override
  {
    SimpleManagedEntity::on_activate();
    should_log_ = true;
  }

private:
  /// LifecycleSubscription log helper function
  /**
   * Helper function that logs a message saying that subscriber can't handle
   * because it's not enabled.
   */
  void log_subscription_not_enabled()
  {
    // Nothing to do if we are not meant to log
    if (!should_log_) {
      return;
    }

    // Log the message
    RCLCPP_WARN(
      logger_,
      "Trying to handle message on the topic '%s', but the subscription is not activated",
      this->get_topic_name());

    // We stop logging until the flag gets enabled again
    should_log_ = false;
  }

  bool should_log_ = true;
  rclcpp::Logger logger_;
};

}  // namespace rclcpp_lifecycle

#endif  // RCLCPP_LIFECYCLE__LIFECYCLE_SUBSCRIPTION_HPP_
