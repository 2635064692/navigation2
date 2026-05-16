// Copyright (c) 2023 Dexory
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

#include "nav2_collision_monitor/velocity_polygon.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point32.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_collision_monitor
{

VelocityPolygon::VelocityPolygon(
  const nav2_util::LifecycleNode::WeakPtr & node,
  const std::string & polygon_name,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string & base_frame_id,
  const tf2::Duration & transform_tolerance)
: Polygon(node, polygon_name, tf_buffer, base_frame_id, transform_tolerance),
  holonomic_(false)
{
  RCLCPP_INFO(logger_, "[%s]: Creating VelocityPolygon", polygon_name_.c_str());
}

VelocityPolygon::~VelocityPolygon()
{
  RCLCPP_INFO(logger_, "[%s]: Destroying VelocityPolygon", polygon_name_.c_str());
}

bool VelocityPolygon::getParameters(
  std::string & polygon_pub_topic,
  std::string & footprint_topic)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  clock_ = node->get_clock();

  if (!getCommonParameters(polygon_pub_topic)) {
    return false;
  }
  footprint_topic.clear();

  try {
    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".velocity_polygons", rclcpp::PARAMETER_STRING_ARRAY);
    std::vector<std::string> velocity_polygons =
      node->get_parameter(polygon_name_ + ".velocity_polygons").as_string_array();

    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".holonomic", rclcpp::ParameterValue(false));
    holonomic_ = node->get_parameter(polygon_name_ + ".holonomic").as_bool();

    for (const std::string & velocity_polygon_name : velocity_polygons) {
      SubPolygonParameter sub_polygon;
      sub_polygon.velocity_polygon_name = velocity_polygon_name;
      sub_polygon.direction_end_angle = 0.0;
      sub_polygon.direction_start_angle = 0.0;

      const std::string sub_polygon_prefix = polygon_name_ + "." + velocity_polygon_name;
      nav2_util::declare_parameter_if_not_declared(
        node, sub_polygon_prefix + ".points", rclcpp::PARAMETER_STRING);
      std::string poly_string = node->get_parameter(sub_polygon_prefix + ".points").as_string();
      if (!getPolygonFromString(poly_string, sub_polygon.poly)) {
        return false;
      }

      nav2_util::declare_parameter_if_not_declared(
        node, sub_polygon_prefix + ".linear_min", rclcpp::PARAMETER_DOUBLE);
      sub_polygon.linear_min = node->get_parameter(sub_polygon_prefix + ".linear_min").as_double();

      nav2_util::declare_parameter_if_not_declared(
        node, sub_polygon_prefix + ".linear_max", rclcpp::PARAMETER_DOUBLE);
      sub_polygon.linear_max = node->get_parameter(sub_polygon_prefix + ".linear_max").as_double();

      nav2_util::declare_parameter_if_not_declared(
        node, sub_polygon_prefix + ".theta_min", rclcpp::PARAMETER_DOUBLE);
      sub_polygon.theta_min = node->get_parameter(sub_polygon_prefix + ".theta_min").as_double();

      nav2_util::declare_parameter_if_not_declared(
        node, sub_polygon_prefix + ".theta_max", rclcpp::PARAMETER_DOUBLE);
      sub_polygon.theta_max = node->get_parameter(sub_polygon_prefix + ".theta_max").as_double();

      if (holonomic_) {
        nav2_util::declare_parameter_if_not_declared(
          node, sub_polygon_prefix + ".direction_end_angle", rclcpp::ParameterValue(M_PI));
        sub_polygon.direction_end_angle =
          node->get_parameter(sub_polygon_prefix + ".direction_end_angle").as_double();

        nav2_util::declare_parameter_if_not_declared(
          node, sub_polygon_prefix + ".direction_start_angle", rclcpp::ParameterValue(-M_PI));
        sub_polygon.direction_start_angle =
          node->get_parameter(sub_polygon_prefix + ".direction_start_angle").as_double();
      }

      sub_polygons_.push_back(sub_polygon);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      logger_, "[%s]: Error while getting velocity polygon parameters: %s",
      polygon_name_.c_str(), ex.what());
    return false;
  }

  return true;
}

void VelocityPolygon::updatePolygon(const Velocity & cmd_vel_in)
{
  for (const auto & sub_polygon : sub_polygons_) {
    if (isInRange(cmd_vel_in, sub_polygon)) {
      poly_ = sub_polygon.poly;
      updateVisualizationPolygon();
      return;
    }
  }

  if (clock_) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "Velocity is not covered by any of the velocity polygons. x: %.3f y: %.3f tw: %.3f",
      cmd_vel_in.x, cmd_vel_in.y, cmd_vel_in.tw);
  }
}

bool VelocityPolygon::isInRange(
  const Velocity & cmd_vel_in,
  const SubPolygonParameter & sub_polygon) const
{
  bool in_range =
    cmd_vel_in.x <= sub_polygon.linear_max && cmd_vel_in.x >= sub_polygon.linear_min &&
    cmd_vel_in.tw <= sub_polygon.theta_max && cmd_vel_in.tw >= sub_polygon.theta_min;

  if (holonomic_) {
    const double direction = std::atan2(cmd_vel_in.y, cmd_vel_in.x);
    if (sub_polygon.direction_start_angle <= sub_polygon.direction_end_angle) {
      in_range = in_range && direction >= sub_polygon.direction_start_angle &&
        direction <= sub_polygon.direction_end_angle;
    } else {
      in_range = in_range && (direction >= sub_polygon.direction_start_angle ||
        direction <= sub_polygon.direction_end_angle);
    }
  }

  return in_range;
}

bool VelocityPolygon::getPolygonFromString(
  const std::string & poly_string,
  std::vector<Point> & polygon) const
{
  std::string cleaned = poly_string;
  cleaned.erase(std::remove(cleaned.begin(), cleaned.end(), '['), cleaned.end());
  cleaned.erase(std::remove(cleaned.begin(), cleaned.end(), ']'), cleaned.end());

  std::stringstream ss(cleaned);
  std::string token;
  std::vector<double> values;
  while (std::getline(ss, token, ',')) {
    try {
      values.push_back(std::stod(token));
    } catch (const std::exception &) {
      RCLCPP_ERROR(
        logger_, "[%s]: Failed to parse velocity polygon points: %s",
        polygon_name_.c_str(), poly_string.c_str());
      return false;
    }
  }

  if (values.size() <= 6 || values.size() % 2 != 0) {
    RCLCPP_ERROR(
      logger_, "[%s]: Velocity polygon has incorrect points description: %s",
      polygon_name_.c_str(), poly_string.c_str());
    return false;
  }

  polygon.clear();
  for (std::size_t i = 0; i < values.size(); i += 2) {
    polygon.push_back({values[i], values[i + 1]});
  }
  return true;
}

bool VelocityPolygon::updateVisualizationPolygon()
{
  polygon_.points.clear();
  for (const Point & p : poly_) {
    geometry_msgs::msg::Point32 p_s;
    p_s.x = p.x;
    p_s.y = p.y;
    polygon_.points.push_back(p_s);
  }
  return true;
}

}  // namespace nav2_collision_monitor
