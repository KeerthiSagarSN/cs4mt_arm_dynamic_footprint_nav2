////////////////////////////////////////////////////////////////////
// Copyright 2025 IMR Robotics & Automation Group
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// CORESENSE Project — RAMP Task
// MTArmFootprintInput.cpp
//
// Afferent component for the ArmFootprint cognitive module.
// Subscribes to /joint_states to receive real-time arm joint positions.
// These are consumed by ArmFootprintCognitiveCore via afferent_->get_msg().
////////////////////////////////////////////////////////////////////

#include "cs4home_core/Afferent.hpp"
#include "cs4home_core/macros.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_components/register_node_macro.hpp"

/**
 * @class MTArmFootprintInput
 * @brief Afferent component that subscribes to /joint_states.
 *
 * Configured via YAML topics list (cs4home_core Afferent convention):
 *
 *   afferent:
 *     topics:
 *       - /joint_states
 *
 * The Core reads the latest JointState message via:
 *   afferent_->get_msg<sensor_msgs::msg::JointState>(0)
 */
class MTArmFootprintInput : public cs4home_core::Afferent {
public:
  RCLCPP_SMART_PTR_DEFINITIONS(MTArmFootprintInput)

  explicit MTArmFootprintInput(rclcpp_lifecycle::LifecycleNode::SharedPtr parent)
      : Afferent("arm_footprint_input", parent) {
    RCLCPP_DEBUG(parent_->get_logger(), "Afferent created: [arm_footprint_input]");
  }

  bool configure() override {
    RCLCPP_INFO(parent_->get_logger(),
                "🦾 [ArmFootprintInput] Configuring — subscribing to /joint_states");
    return Afferent::configure();
  }
};

CS_REGISTER_COMPONENT(MTArmFootprintInput)
