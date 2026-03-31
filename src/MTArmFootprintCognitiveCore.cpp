////////////////////////////////////////////////////////////////////
// Copyright 2025 IMR Robotics & Automation Group
// Author: Keerthi Sagar
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// CORESENSE Project - RAMP Task
// MTArmFootprintCognitiveCore.cpp
//
// Cognitive Core: real-time MPO-700 + UR10 footprint computation.
// All parameters are loaded from params.yaml - no hardcoded values.
//
// Supports:
//   - Circular or rectangular base geometry (use_circular_base param)
//   - Manual YAML link list
////////////////////////////////////////////////////////////////////

#include "cs4home_core/Core.hpp"
#include "cs4home_core/macros.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <future>
#include <limits>

using namespace std::chrono_literals;

class MTArmFootprintCognitiveCore : public cs4home_core::Core {
public:
  RCLCPP_SMART_PTR_DEFINITIONS(MTArmFootprintCognitiveCore)

  explicit MTArmFootprintCognitiveCore(
      rclcpp_lifecycle::LifecycleNode::SharedPtr parent)
  : Core("arm_footprint_cognitive_core", parent)
  {
    RCLCPP_INFO(parent_->get_logger(), "ArmFootprintCognitiveCore created");
  }

  // -----------------------------------------------------------------------
  // cs4home_core::Core interface
  // -----------------------------------------------------------------------

  bool configure() override
  {
    // ------------------------------------------------------------------
    // Base geometry mode
    // ------------------------------------------------------------------
    parent_->declare_parameter("use_circular_base",   false);
    parent_->declare_parameter("base_radius",         0.60);
    parent_->declare_parameter("base_circle_samples", 16);
    parent_->declare_parameter("base_half_x",         0.45);
    parent_->declare_parameter("base_half_y",         0.40);

    use_circular_base_   = parent_->get_parameter("use_circular_base").as_bool();
    base_radius_         = parent_->get_parameter("base_radius").as_double();
    base_circle_samples_ = parent_->get_parameter("base_circle_samples").as_int();
    base_half_x_         = parent_->get_parameter("base_half_x").as_double();
    base_half_y_         = parent_->get_parameter("base_half_y").as_double();

    // ------------------------------------------------------------------
    // Hull computation
    // ------------------------------------------------------------------
    parent_->declare_parameter("safety_padding",               0.08);
    parent_->declare_parameter("segment_samples",              8);
    parent_->declare_parameter("update_period_ms",             50);
    parent_->declare_parameter("base_frame",                   std::string("base_link"));
    parent_->declare_parameter("service_update_every_n_ticks", 10);

    safety_padding_               = parent_->get_parameter("safety_padding").as_double();
    segment_samples_              = parent_->get_parameter("segment_samples").as_int();
    update_period_ms_             = parent_->get_parameter("update_period_ms").as_int();
    base_frame_                   = parent_->get_parameter("base_frame").as_string();
    service_update_every_n_ticks_ = parent_->get_parameter(
        "service_update_every_n_ticks").as_int();

    // ------------------------------------------------------------------
    // Arm link names - loaded from params.yaml as string list
    // ------------------------------------------------------------------
    parent_->declare_parameter("arm_links", std::vector<std::string>{
      "ur10base_link", "ur10shoulder_link", "ur10upper_arm_link",
      "ur10forearm_link", "ur10wrist_1_link", "ur10wrist_2_link",
      "ur10wrist_3_link", "ur10tool0", "neo_gripper_mount_link",
      "robotiq_140_base_link", "left_outer_knuckle", "right_outer_knuckle"
    });
    arm_links_ = parent_->get_parameter("arm_links").as_string_array();

    // ------------------------------------------------------------------
    // Link radii - loaded from params.yaml as double list
    // ------------------------------------------------------------------
    parent_->declare_parameter("link_radii", std::vector<double>{
      0.10, 0.10, 0.09, 0.08, 0.07, 0.07,
      0.07, 0.05, 0.08, 0.09, 0.06, 0.06
    });
    link_radii_ = parent_->get_parameter("link_radii").as_double_array();

    if (link_radii_.size() != arm_links_.size()) {
      RCLCPP_ERROR(parent_->get_logger(),
                   "link_radii size (%zu) != arm_links size (%zu) - check params.yaml",
                   link_radii_.size(), arm_links_.size());
      return false;
    }

    // ------------------------------------------------------------------
    // TF + publishers
    // ------------------------------------------------------------------
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(parent_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    local_costmap_client_ = parent_->create_client<rcl_interfaces::srv::SetParameters>(
        "/local_costmap/local_costmap/set_parameters");
    global_costmap_client_ = parent_->create_client<rcl_interfaces::srv::SetParameters>(
        "/global_costmap/global_costmap/set_parameters");

    local_footprint_pub_ = parent_->create_publisher<geometry_msgs::msg::Polygon>(
        "/local_costmap/footprint", rclcpp::QoS(10));
    global_footprint_pub_ = parent_->create_publisher<geometry_msgs::msg::Polygon>(
        "/global_costmap/footprint", rclcpp::QoS(10));

    // ------------------------------------------------------------------
    // Log configuration
    // ------------------------------------------------------------------
    if (use_circular_base_) {
      RCLCPP_INFO(parent_->get_logger(),
                  "Configured | base: CIRCLE (r=%.2fm, %d pts) | "
                  "links: %zu | padding: %.3fm | samples: %d | %dms",
                  base_radius_, base_circle_samples_,
                  arm_links_.size(), safety_padding_,
                  segment_samples_, update_period_ms_);
    } else {
      RCLCPP_INFO(parent_->get_logger(),
                  "Configured | base: RECTANGLE %.2fx%.2fm | "
                  "links: %zu | padding: %.3fm | samples: %d | %dms",
                  base_half_x_ * 2, base_half_y_ * 2,
                  arm_links_.size(), safety_padding_,
                  segment_samples_, update_period_ms_);
    }

    RCLCPP_INFO(parent_->get_logger(), "Arm links:");
    for (size_t i = 0; i < arm_links_.size(); i++) {
      RCLCPP_INFO(parent_->get_logger(),
                  "  [%zu] %s (r=%.3f)",
                  i, arm_links_[i].c_str(), link_radii_[i]);
    }

    return true;
  }

  bool activate() override
  {
    timer_ = parent_->create_wall_timer(
        std::chrono::milliseconds(update_period_ms_),
        std::bind(&MTArmFootprintCognitiveCore::timer_callback, this));
    RCLCPP_INFO(parent_->get_logger(), "ArmFootprintCognitiveCore activated");
    return true;
  }

  bool deactivate() override
  {
    timer_ = nullptr;
    RCLCPP_INFO(parent_->get_logger(), "ArmFootprintCognitiveCore deactivated");
    return true;
  }

  // -----------------------------------------------------------------------
  // Timer callback
  // -----------------------------------------------------------------------

  void timer_callback()
  {
    // Start with base geometry - circular or rectangular
    auto all_points = use_circular_base_
        ? get_base_circle_points()
        : get_base_rectangle_points();

    // Add per-link disc points
    for (size_t i = 0; i < arm_links_.size(); i++) {
      auto pts = project_link_to_ground(arm_links_[i], link_radii_[i]);
      all_points.insert(all_points.end(), pts.begin(), pts.end());
    }

    // Add segment sample points between consecutive links
    for (size_t i = 0; i + 1 < arm_links_.size(); i++) {
      auto pts = sample_link_segment(
          arm_links_[i], arm_links_[i + 1],
          link_radii_[i], segment_samples_);
      all_points.insert(all_points.end(), pts.begin(), pts.end());
    }

    // Fallback to base if not enough points
    if (all_points.size() < 3) {
      RCLCPP_WARN_THROTTLE(parent_->get_logger(),
                           *parent_->get_clock(), 5000,
                           "Not enough points for hull, using base fallback");
      all_points = use_circular_base_
          ? get_base_circle_points()
          : get_base_rectangle_points();
    }

    auto hull   = convex_hull(all_points);
    auto padded = pad_polygon(hull, safety_padding_);

    publish_polygon(padded);

    RCLCPP_DEBUG(parent_->get_logger(), "hull: %zu pts", padded.size());
  }

  // -----------------------------------------------------------------------
  // Geometry
  // -----------------------------------------------------------------------

  struct Point2D { double x, y; };

  // Rectangular base
  std::vector<Point2D> get_base_rectangle_points()
  {
    return {
      { base_half_x_,  base_half_y_},
      {-base_half_x_,  base_half_y_},
      {-base_half_x_, -base_half_y_},
      { base_half_x_, -base_half_y_},
    };
  }

  // Circular base
  std::vector<Point2D> get_base_circle_points()
  {
    std::vector<Point2D> pts;
    for (int i = 0; i < base_circle_samples_; i++) {
      double angle = 2.0 * M_PI * i / base_circle_samples_;
      pts.push_back({
        base_radius_ * std::cos(angle),
        base_radius_ * std::sin(angle)
      });
    }
    return pts;
  }

  std::vector<Point2D> project_link_to_ground(
      const std::string & link_name, double radius)
  {
    std::vector<Point2D> pts;
    try {
      auto tf = tf_buffer_->lookupTransform(
          base_frame_, link_name,
          tf2::TimePointZero, tf2::durationFromSec(0.05));
      double cx = tf.transform.translation.x;
      double cy = tf.transform.translation.y;
      pts.push_back({cx, cy});
      constexpr int N = 8;
      for (int i = 0; i < N; i++) {
        double angle = 2.0 * M_PI * i / N;
        pts.push_back({cx + radius * std::cos(angle),
                       cy + radius * std::sin(angle)});
      }
    } catch (const tf2::TransformException & ex) {
      RCLCPP_DEBUG(parent_->get_logger(),
                   "TF [%s]: %s", link_name.c_str(), ex.what());
    }
    return pts;
  }

  std::vector<Point2D> sample_link_segment(
      const std::string & from_link,
      const std::string & to_link,
      double radius,
      int n_samples)
  {
    std::vector<Point2D> pts;
    try {
      auto tf_from = tf_buffer_->lookupTransform(
          base_frame_, from_link,
          tf2::TimePointZero, tf2::durationFromSec(0.05));
      auto tf_to = tf_buffer_->lookupTransform(
          base_frame_, to_link,
          tf2::TimePointZero, tf2::durationFromSec(0.05));

      double x0 = tf_from.transform.translation.x;
      double y0 = tf_from.transform.translation.y;
      double x1 = tf_to.transform.translation.x;
      double y1 = tf_to.transform.translation.y;

      for (int s = 0; s <= n_samples; s++) {
        double t  = static_cast<double>(s) / static_cast<double>(n_samples);
        double cx = x0 + t * (x1 - x0);
        double cy = y0 + t * (y1 - y0);
        constexpr int N = 8;
        for (int i = 0; i < N; i++) {
          double angle = 2.0 * M_PI * i / N;
          pts.push_back({cx + radius * std::cos(angle),
                         cy + radius * std::sin(angle)});
        }
        pts.push_back({cx, cy});
      }
    } catch (const tf2::TransformException & ex) {
      RCLCPP_DEBUG(parent_->get_logger(),
                   "TF segment [%s->%s]: %s",
                   from_link.c_str(), to_link.c_str(), ex.what());
    }
    return pts;
  }

  double cross(const Point2D & O, const Point2D & A, const Point2D & B)
  {
    return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
  }

  std::vector<Point2D> convex_hull(std::vector<Point2D> pts)
  {
    int n = static_cast<int>(pts.size());
    if (n < 3) return pts;

    int start = 0;
    for (int i = 1; i < n; i++) {
      if (pts[i].x < pts[start].x ||
         (pts[i].x == pts[start].x && pts[i].y < pts[start].y)) {
        start = i;
      }
    }

    std::vector<Point2D> hull;
    int cur = start;
    do {
      hull.push_back(pts[cur]);
      int nxt = (cur + 1) % n;
      for (int i = 0; i < n; i++) {
        if (cross(pts[cur], pts[nxt], pts[i]) < 0.0) nxt = i;
      }
      cur = nxt;
    } while (cur != start);

    return hull;
  }

  std::vector<Point2D> pad_polygon(
      const std::vector<Point2D> & poly, double padding)
  {
    if (poly.empty()) return poly;

    double cx = 0.0, cy = 0.0;
    for (const auto & p : poly) { cx += p.x; cy += p.y; }
    cx /= poly.size();
    cy /= poly.size();

    std::vector<Point2D> padded;
    for (const auto & p : poly) {
      double dx   = p.x - cx;
      double dy   = p.y - cy;
      double dist = std::sqrt(dx * dx + dy * dy);
      if (dist < 1e-9) { padded.push_back(p); continue; }
      padded.push_back({
        p.x + padding * dx / dist,
        p.y + padding * dy / dist
      });
    }
    return padded;
  }

  void publish_polygon(const std::vector<Point2D> & poly)
  {
    geometry_msgs::msg::Polygon msg;
    for (const auto & p : poly) {
      geometry_msgs::msg::Point32 pt;
      pt.x = static_cast<float>(p.x);
      pt.y = static_cast<float>(p.y);
      pt.z = 0.0f;
      msg.points.push_back(pt);
    }
    local_footprint_pub_->publish(msg);
    global_footprint_pub_->publish(msg);
  }

private:
  // Base geometry mode
  bool   use_circular_base_;
  double base_radius_;
  int    base_circle_samples_;
  double base_half_x_;
  double base_half_y_;

  // Arm links and radii - loaded from params.yaml
  std::vector<std::string> arm_links_;
  std::vector<double>      link_radii_;

  // Configuration
  std::string base_frame_;
  double      safety_padding_;
  int         update_period_ms_;
  int         segment_samples_;
  int         service_update_every_n_ticks_;
  int         service_tick_counter_{0};

  // TF
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Service clients
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr local_costmap_client_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr global_costmap_client_;

  // Footprint publishers
  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr local_footprint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr global_footprint_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

CS_REGISTER_COMPONENT(MTArmFootprintCognitiveCore)
