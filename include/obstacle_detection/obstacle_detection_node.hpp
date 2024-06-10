// Copyright 2024 Andrzej_Norbert_Jeremiasz
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OBSTACLE_DETECTION__OBSTACLE_DETECTION_NODE_HPP_
#define OBSTACLE_DETECTION__OBSTACLE_DETECTION_NODE_HPP_

#include "tier4_autoware_utils/ros/logger_level_configure.hpp"
#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <tf2/utils.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <vector>
#include <unordered_map>

namespace obstacle_detection
{
using nav_msgs::msg::OccupancyGrid;
using sensor_msgs::msg::LaserScan;
using nav_msgs::msg::Odometry;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::TransformStamped;

struct NodeParam
{
  int cell_count_th;
};

struct IndexXY
{
  int x;
  int y;

  bool operator==(const IndexXY & other) const
  {
    return x == other.x && y == other.y;
  }
};

struct IndexXYHash
{
  std::size_t operator()(const IndexXY & index) const
  {
    return std::hash<int>()(index.x) ^ (std::hash<int>()(index.y) << 1);
  }
};

class ObstacleDetectionNode : public rclcpp::Node
{
public:
  explicit ObstacleDetectionNode(const rclcpp::NodeOptions & node_options);

private:
  // ros
  rclcpp::Publisher<OccupancyGrid>::SharedPtr occupancy_grid_pub_;

  rclcpp::Subscription<OccupancyGrid>::SharedPtr occupancy_grid_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<LaserScan>::SharedPtr scan_sub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // params
  NodeParam node_param_;

  // variables
  bool is_map_saved;

  OccupancyGrid::ConstSharedPtr occupancy_grid_;
  Odometry::ConstSharedPtr odom_;
  LaserScan:: ConstSharedPtr scan_;

  // functions, callback
  void onOccupancyGrid(const OccupancyGrid::ConstSharedPtr msg);
  void onOdometry(const Odometry::ConstSharedPtr msg);
  void onScan(const LaserScan::ConstSharedPtr msg);

  void updateMap();

  TransformStamped getTransform(const std::string & from, const std::string & to);

  std::unique_ptr<tier4_autoware_utils::LoggerLevelConfigure> logger_configure_;
};
}  // namespace obstacle_detection

#endif  // OBSTACLE_DETECTION__OBSTACLE_DETECTION_NODE_HPP_