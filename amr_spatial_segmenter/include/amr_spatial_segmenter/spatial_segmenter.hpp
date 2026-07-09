#ifndef AMR_SPATIAL_SEGMENTER__SPATIAL_SEGMENTER_HPP_
#define AMR_SPATIAL_SEGMENTER__SPATIAL_SEGMENTER_HPP_

/**
 * @file spatial_segmenter.hpp
 * @brief ROS2 lifecycle node that publishes OccupancyGrid spatial segmentation overlays.
 */

#include <memory>
#include <string>

#include "amr_msgs/msg/spatial_segment_array.hpp"
#include "amr_spatial_segmenter/grid_types.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/header.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace amr::spatial_segmenter
{

/// @brief Lifecycle node for rule-based spatial segmentation of live SLAM maps.
class SpatialSegmenter : public rclcpp_lifecycle::LifecycleNode
{
public:
  /// @brief Construct and declare parameters.
  explicit SpatialSegmenter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /// @brief Destroy the node.
  ~SpatialSegmenter() override = default;

private:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  void load_parameters();
  void handle_map(const nav_msgs::msg::OccupancyGrid::SharedPtr message);
  void process_latest_map();
  bool should_process_map() const;
  SegmentationResult analyze_map(const nav_msgs::msg::OccupancyGrid & map) const;
  nav_msgs::msg::OccupancyGrid build_segment_map(
    const nav_msgs::msg::OccupancyGrid & source,
    const SegmentationResult & result) const;
  visualization_msgs::msg::MarkerArray build_markers(
    const std_msgs::msg::Header & header,
    const std::vector<Segment> & segments,
    bool corridor_only,
    bool area_only,
    bool door_only) const;
  amr_msgs::msg::SpatialSegmentArray build_segment_array(
    const std_msgs::msg::Header & header,
    const std::vector<Segment> & segments) const;
  void publish_result(
    const nav_msgs::msg::OccupancyGrid & segment_map,
    const visualization_msgs::msg::MarkerArray & markers,
    const visualization_msgs::msg::MarkerArray & corridor_markers,
    const visualization_msgs::msg::MarkerArray & area_markers,
    const visualization_msgs::msg::MarkerArray & door_markers,
    const amr_msgs::msg::SpatialSegmentArray & segments);

  SegmenterParams params_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
    segment_map_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    markers_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    corridor_centerlines_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    area_boundaries_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    door_candidates_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    debug_cells_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    debug_rejected_areas_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    debug_internal_barriers_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    debug_scanline_intervals_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<amr_msgs::msg::SpatialSegmentArray>::SharedPtr
    segments_publisher_;
  rclcpp::TimerBase::SharedPtr processing_timer_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
  rclcpp::Time latest_map_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_processed_map_time_{0, 0, RCL_ROS_TIME};
  bool map_dirty_{false};
};

}  // namespace amr::spatial_segmenter

#endif  // AMR_SPATIAL_SEGMENTER__SPATIAL_SEGMENTER_HPP_
