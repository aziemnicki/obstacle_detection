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

#include "obstacle_detection/obstacle_detection_node.hpp"

namespace obstacle_detection
{
Pose transformPose(const Pose & pose, const TransformStamped & transform)
{
  Pose transformed_pose;
  tf2::doTransform(pose, transformed_pose, transform);

  return transformed_pose;
}

IndexXY pose2index(const OccupancyGrid & costmap, const Pose & pose_local)
{
  const int index_x = pose_local.position.x / costmap.info.resolution;
  const int index_y = pose_local.position.y / costmap.info.resolution;
  return {index_x, index_y};
}

Pose global2local(
  const OccupancyGrid & costmap, const Pose & pose_global)
{
  tf2::Transform tf_origin;
  tf2::convert(costmap.info.origin, tf_origin);

  TransformStamped transform;
  transform.transform = tf2::toMsg(tf_origin.inverse());

  return transformPose(pose_global, transform);
}

ObstacleDetectionNode::ObstacleDetectionNode(const rclcpp::NodeOptions & node_options)
: Node("obstacle_detection", node_options)
{
  using std::placeholders::_1;

  // NodeParam
  {
    auto & p = node_param_;
    p.cell_count_th = declare_parameter<int>("cell_count_th");
  }

  // Subscribers
  {
    occupancy_grid_sub_ = create_subscription<OccupancyGrid>(
        "~/input/occupancy_grid", 1, std::bind(&ObstacleDetectionNode::onOccupancyGrid, this, _1));
    odom_sub_ = create_subscription<Odometry>(
        "~/input/odometry", 100, std::bind(&ObstacleDetectionNode::onOdometry, this, _1));
    scan_sub_ = create_subscription<LaserScan>(
        "~/input/laser_scan", rclcpp::QoS{5}.best_effort(), std::bind(&ObstacleDetectionNode::onScan, this, _1));
  }

  // Publishers
  {
    rclcpp::QoS qos{1};
    qos.transient_local();
    occupancy_grid_pub_ = create_publisher<OccupancyGrid>("~/output/occupancy_grid", qos);
  }

    // TF
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  is_map_saved = false;
}

void ObstacleDetectionNode::onOccupancyGrid(const OccupancyGrid::ConstSharedPtr msg)
{
  // RCLCPP_INFO(get_logger(), "onOccupancyGrid");
  if (!is_map_saved)
  {
    occupancy_grid_ = msg;
    is_map_saved = true;
  }
}

void ObstacleDetectionNode::onOdometry(const Odometry::ConstSharedPtr msg)
{
  odom_ = msg;
}

void ObstacleDetectionNode::onScan(const LaserScan::ConstSharedPtr msg)
{
  scan_ = msg;

  if (occupancy_grid_ != nullptr && odom_ != nullptr)
  {
    updateMap();
  }
}

void ObstacleDetectionNode::updateMap()
{
  OccupancyGrid updated_map = *occupancy_grid_;

  // Map to count the occurrences of each cell
  std::unordered_map<IndexXY, int, IndexXYHash> index_counts;
  
  const auto base_local = global2local(*occupancy_grid_, odom_->pose.pose);
  const double base_x = base_local.position.x;
  const double base_y = base_local.position.y;
  const double base_yaw = tf2::getYaw(base_local.orientation);

  // Calculate map cells where lidar scans lie and count occurrences
  for (size_t i = 0; i < scan_->ranges.size(); ++i) {
    // Skip invalid ranges
    if (scan_->ranges[i] < scan_->range_min || scan_->ranges[i] > scan_->range_max) {
      continue;
    }

    const double scan_angle = scan_->angle_min + i * scan_->angle_increment;
    const double scan_x = scan_->ranges[i] * cos(scan_angle);
    const double scan_y = scan_->ranges[i] * sin(scan_angle);

    Pose scan_pose_laser;
    scan_pose_laser.position.x = scan_x;
    scan_pose_laser.position.y = scan_y;
    scan_pose_laser.orientation = tier4_autoware_utils::createQuaternionFromYaw(scan_angle);

    const auto scan_pose_base = transformPose(scan_pose_laser, getTransform(odom_->child_frame_id, scan_->header.frame_id));

    const double offset_x = cos(base_yaw) * scan_pose_base.position.x - sin(base_yaw) * scan_pose_base.position.y;
    const double offset_y = sin(base_yaw) * scan_pose_base.position.x + cos(base_yaw) * scan_pose_base.position.y;

    Pose scan_pose_local;
    scan_pose_local.position.x = base_x + offset_x;
    scan_pose_local.position.y = base_y + offset_y;

    const auto index = pose2index(*occupancy_grid_, scan_pose_local);
    index_counts[index]++;
  }

  // Filter out cells that don't meet the threshold
  for (const auto & entry : index_counts) {
    if (entry.second >= node_param_.cell_count_th) {
      const auto & index = entry.first;
      if (index.x >= 0 && static_cast<unsigned int>(index.x) < occupancy_grid_->info.width && 
          index.y >= 0 && static_cast<unsigned int>(index.y) < occupancy_grid_->info.height) {
        updated_map.data[index.x + index.y * occupancy_grid_->info.width] = 100;
      }
    }
  }

  occupancy_grid_pub_->publish(updated_map);
}

TransformStamped ObstacleDetectionNode::getTransform(
  const std::string & from, const std::string & to)
{
  TransformStamped tf;
  try {
    tf =
      tf_buffer_->lookupTransform(from, to, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
  }
  return tf;
}
} // namespace obstacle_detection

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(obstacle_detection::ObstacleDetectionNode)