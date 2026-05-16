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

#ifndef NAV2_COLLISION_MONITOR__VELOCITY_POLYGON_HPP_
#define NAV2_COLLISION_MONITOR__VELOCITY_POLYGON_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_collision_monitor/polygon.hpp"

namespace nav2_collision_monitor
{

/**
 * @brief Polygon shape selected by commanded velocity ranges.
 */
class VelocityPolygon : public Polygon
{
public:
  VelocityPolygon(
    const nav2_util::LifecycleNode::WeakPtr & node,
    const std::string & polygon_name,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string & base_frame_id,
    const tf2::Duration & transform_tolerance);

  ~VelocityPolygon() override;

  bool getParameters(std::string & polygon_pub_topic, std::string & footprint_topic) override;

  void updatePolygon(const Velocity & cmd_vel_in) override;

protected:
  struct SubPolygonParameter
  {
    std::vector<Point> poly;
    std::string velocity_polygon_name;
    double linear_min;
    double linear_max;
    double theta_min;
    double theta_max;
    double direction_end_angle;
    double direction_start_angle;
  };

  bool isInRange(const Velocity & cmd_vel_in, const SubPolygonParameter & sub_polygon) const;
  bool getPolygonFromString(const std::string & poly_string, std::vector<Point> & polygon) const;
  bool updateVisualizationPolygon();

  rclcpp::Clock::SharedPtr clock_;
  bool holonomic_;
  std::vector<SubPolygonParameter> sub_polygons_;
};

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__VELOCITY_POLYGON_HPP_
