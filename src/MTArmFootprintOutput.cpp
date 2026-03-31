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
// MTArmFootprintOutput.cpp
//
// Efferent component for the ArmFootprint cognitive module.
// Publishes the computed dynamic footprint polygon to Nav2 costmap topics.
//
// Configured via YAML topics list (cs4home_core Efferent convention):
//
//   efferent:
//     topics:
//       - /local_costmap/footprint
//       - /global_costmap/footprint
////////////////////////////////////////////////////////////////////

#include "cs4home_core/Efferent.hpp"
#include "cs4home_core/macros.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_components/register_node_macro.hpp"

/**
 * @class MTArmFootprintOutput
 * @brief Efferent component that publishes the dynamic footprint polygon.
 *
 * Publishes geometry_msgs::msg::PolygonStamped to:
 *   - /local_costmap/footprint   (index 0)
 *   - /global_costmap/footprint  (index 1)
 *
 * The Core calls:
 *   efferent_->publish(0, local_polygon_msg);
 *   efferent_->publish(1, global_polygon_msg);
 */
class MTArmFootprintOutput : public cs4home_core::Efferent {
public:
  RCLCPP_SMART_PTR_DEFINITIONS(MTArmFootprintOutput)

  explicit MTArmFootprintOutput(rclcpp_lifecycle::LifecycleNode::SharedPtr parent)
      : Efferent("arm_footprint_output", parent) {
    RCLCPP_INFO(parent_->get_logger(), "Efferent created: [arm_footprint_output]");
  }

  bool configure() override {
    RCLCPP_INFO(parent_->get_logger(),
                "📤 [ArmFootprintOutput] Configuring — publishing to costmap footprint topics");
    return Efferent::configure();
  }
};

CS_REGISTER_COMPONENT(MTArmFootprintOutput)
