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
// MTArmFootprintCognitiveModule.cpp
//
// CognitiveModule that wires together:
//   Afferent  → MTArmFootprintInput        (/joint_states)
//   Core      → MTArmFootprintCognitiveCore (TF projection + convex hull)
//   Efferent  → MTArmFootprintOutput        (/local_costmap/footprint,
//                                            /global_costmap/footprint)
//
// Configured via the standard cs4home_core YAML module config.
////////////////////////////////////////////////////////////////////

#include "cs4home_core/CognitiveModule.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using namespace std::chrono_literals;

/**
 * @class MTArmFootprintCognitiveModule
 * @brief CognitiveModule for real-time MPO-700 + UR10 footprint projection.
 *
 * Part of the CORESENSE RAMP cognitive navigation architecture.
 * Computes the Minkowski sum of the mobile base footprint and the
 * downward projection of the UR10 arm links, updating Nav2 costmap
 * footprints in real-time via set_parameters.
 */
class MTArmFootprintCognitiveModule : public cs4home_core::CognitiveModule {
public:
  explicit MTArmFootprintCognitiveModule(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : cs4home_core::CognitiveModule("arm_footprint_cognitive_module", options) {

    RCLCPP_INFO(this->get_logger(),
                "🦾 MTArmFootprintCognitiveModule initialized");
    RCLCPP_INFO(this->get_logger(),
                "   Computes: base_rect ⊕ convex_hull(arm_projection)");
    RCLCPP_INFO(this->get_logger(),
                "   Updates:  /local_costmap + /global_costmap footprint");
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("main"),
              "🚀 Starting MTArmFootprintCognitiveModule (CORESENSE RAMP)");

  auto node = std::make_shared<MTArmFootprintCognitiveModule>();

  RCLCPP_INFO(rclcpp::get_logger("main"),
              "🦾 ArmFootprint Cognitive Module running...");

  rclcpp::spin(node->get_node_base_interface());

  RCLCPP_INFO(rclcpp::get_logger("main"),
              "🛑 MTArmFootprintCognitiveModule shutdown");

  rclcpp::shutdown();
  return 0;
}
