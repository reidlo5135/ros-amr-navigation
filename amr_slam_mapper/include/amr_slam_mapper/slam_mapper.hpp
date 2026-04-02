#ifndef AMR_SLAM_MAPPER__SLAM_MAPPER_HPP_
#define AMR_SLAM_MAPPER__SLAM_MAPPER_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <mutex>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace amr::slam::mapper
{

class SlamMapper : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit SlamMapper(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~SlamMapper() = default;

private:
  struct Pose2D
  {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  struct GraphNode
  {
    int id{0};
    Pose2D map_pose{};
    Pose2D raw_odom_pose{};
    sensor_msgs::msg::LaserScan scan{};
    std::vector<float> descriptor;
    int submap_id{0};
  };

  struct GraphEdge
  {
    int from{0};
    int to{0};
    Pose2D relative_pose{};
    double weight{1.0};
    bool loop_closure{false};
  };

  struct Submap
  {
    int id{0};
    int start_node_index{0};
    int end_node_index{0};
    Pose2D anchor_pose{};
    int keyframe_count{0};
  };

  struct LoopClosureCandidate
  {
    bool found{false};
    int node_index{-1};
    Pose2D matched_pose{};
    double descriptor_distance{0.0};
    double match_score{0.0};
  };

  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  void initialize_mapping_map();
  void publish_outputs();
  void refresh_refined_map();
  void publish_temporary_map();
  void publish_raw_temporary_map();
  void publish_refined_temporary_map();
  void publish_corrected_odometry();
  void publish_mapping_pose();
  void publish_graph_debug();
  void publish_map_to_odom_tf();
  std::string build_graph_debug_json() const;
  nav_msgs::msg::OccupancyGrid build_refined_map(
    const nav_msgs::msg::OccupancyGrid & source_map) const;
  int count_neighboring_cells(
    const nav_msgs::msg::OccupancyGrid & map,
    int grid_x,
    int grid_y,
    int minimum_value,
    int maximum_value) const;

  void handle_odometry(const nav_msgs::msg::Odometry::SharedPtr message);
  void handle_imu(const sensor_msgs::msg::Imu::SharedPtr message);
  void handle_scan(const sensor_msgs::msg::LaserScan::SharedPtr message);

  Pose2D build_raw_odom_pose() const;
  Pose2D apply_map_to_odom_transform(const Pose2D & odom_pose) const;
  void update_map_to_odom_transform(const Pose2D & corrected_pose, const Pose2D & raw_odom_pose);
  Pose2D refine_pose_with_scan_matching(
    const sensor_msgs::msg::LaserScan & scan,
    const Pose2D & predicted_pose) const;
  double score_scan_candidate(
    const nav_msgs::msg::OccupancyGrid & map,
    const sensor_msgs::msg::LaserScan & scan,
    const Pose2D & candidate_pose) const;
  double nearest_occupied_distance_cells(
    const nav_msgs::msg::OccupancyGrid & map,
    int grid_x,
    int grid_y,
    int radius_cells) const;
  bool has_nearby_occupied_cell(
    const nav_msgs::msg::OccupancyGrid & map,
    int grid_x,
    int grid_y,
    int radius_cells) const;

  void integrate_scan_into_map(
    const sensor_msgs::msg::LaserScan & scan,
    const Pose2D & corrected_pose,
    nav_msgs::msg::OccupancyGrid & map,
    std::vector<int16_t> & occupancy_scores) const;
  void rebuild_map_from_pose_graph();

  void maybe_add_pose_graph_node(
    const sensor_msgs::msg::LaserScan & scan,
    const Pose2D & corrected_pose,
    const Pose2D & raw_odom_pose);
  std::vector<float> build_scan_descriptor(const sensor_msgs::msg::LaserScan & scan) const;
  double compute_descriptor_distance(
    const std::vector<float> & lhs,
    const std::vector<float> & rhs) const;
  LoopClosureCandidate search_loop_closure_candidate(
    const sensor_msgs::msg::LaserScan & scan,
    const std::vector<float> & descriptor) const;
  void maybe_optimize_pose_graph(const LoopClosureCandidate & candidate, int current_node_index);
  void optimize_pose_graph();
  void update_submap_accumulation(const GraphNode & node);

  Pose2D compose_pose(const Pose2D & lhs, const Pose2D & rhs) const;
  Pose2D inverse_pose(const Pose2D & pose) const;
  Pose2D relative_pose(const Pose2D & from, const Pose2D & to) const;

  bool world_to_grid(
    const nav_msgs::msg::OccupancyGrid & map,
    double x,
    double y,
    int & grid_x,
    int & grid_y) const;
  bool grid_index(
    const nav_msgs::msg::OccupancyGrid & map,
    int grid_x,
    int grid_y,
    std::size_t & index) const;
  void update_cell_score(
    nav_msgs::msg::OccupancyGrid & map,
    std::vector<int16_t> & occupancy_scores,
    int grid_x,
    int grid_y,
    int delta) const;
  void refresh_cell_from_score(
    nav_msgs::msg::OccupancyGrid & map,
    const std::vector<int16_t> & occupancy_scores,
    std::size_t index) const;
  void raytrace_free_cells(
    nav_msgs::msg::OccupancyGrid & map,
    std::vector<int16_t> & occupancy_scores,
    int start_x,
    int start_y,
    int end_x,
    int end_y) const;
  void mark_free_cell(
    nav_msgs::msg::OccupancyGrid & map,
    std::vector<int16_t> & occupancy_scores,
    int grid_x,
    int grid_y) const;
  void mark_occupied_cell(
    nav_msgs::msg::OccupancyGrid & map,
    std::vector<int16_t> & occupancy_scores,
    int grid_x,
    int grid_y) const;

  void set_quaternion_from_yaw(geometry_msgs::msg::Quaternion & orientation, double yaw) const;
  double quaternion_to_yaw(const geometry_msgs::msg::Quaternion & orientation) const;
  double normalize_angle(double angle) const;

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr temporary_map_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr raw_temporary_map_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr refined_temporary_map_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr corrected_odometry_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr mapping_pose_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr graph_debug_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

  nav_msgs::msg::OccupancyGrid temporary_map_;
  nav_msgs::msg::OccupancyGrid refined_temporary_map_;
  std::vector<int16_t> occupancy_scores_;
  mutable std::mutex map_mutex_;

  nav_msgs::msg::Odometry latest_odometry_{};
  Pose2D current_corrected_pose_{};
  Pose2D map_to_odom_{};
  double start_odom_yaw_{0.0};
  double start_imu_yaw_{0.0};
  double latest_imu_yaw_{0.0};
  bool has_latest_odometry_{false};
  bool has_latest_scan_{false};
  bool has_current_corrected_pose_{false};
  bool has_start_odom_yaw_{false};
  bool has_latest_imu_{false};
  bool has_start_imu_yaw_{false};

  std::vector<GraphNode> graph_nodes_;
  std::vector<GraphEdge> graph_edges_;
  std::vector<Submap> submaps_;
  int next_graph_node_id_{1};
  int next_submap_id_{1};

  std::string frame_id_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string odom_topic_;
  std::string imu_topic_;
  std::string scan_topic_;
  std::string temporary_map_topic_;
  std::string raw_temporary_map_topic_;
  std::string refined_temporary_map_topic_;
  std::string corrected_odometry_topic_;
  std::string mapping_pose_topic_;
  std::string graph_debug_topic_;
  int publish_period_ms_;
  double mapping_resolution_;
  int mapping_width_;
  int mapping_height_;
  double mapping_origin_x_;
  double mapping_origin_y_;
  double mapping_origin_yaw_;
  double mapping_min_range_;
  double mapping_max_range_;
  bool use_imu_heading_;
  bool publish_map_to_odom_tf_;

  double scan_matching_linear_window_;
  double scan_matching_linear_step_;
  double scan_matching_angular_window_deg_;
  double scan_matching_angular_step_deg_;
  int scan_matching_max_beams_;
  int scan_matching_min_valid_beams_;
  int scan_matching_occupied_search_radius_cells_;
  int scan_matching_distance_match_radius_cells_;
  int scan_matching_minimum_occupied_cells_;
  double scan_matching_occupied_match_score_;
  double scan_matching_distance_match_score_;
  double scan_matching_distance_penalty_per_cell_;
  double scan_matching_free_space_penalty_;
  double scan_matching_min_score_improvement_;
  double scan_matching_max_translation_correction_;
  double scan_matching_max_yaw_correction_deg_;
  double scan_matching_translation_regularization_weight_;
  double scan_matching_yaw_regularization_weight_;

  int mapping_hit_score_;
  int mapping_free_score_;
  int mapping_occupied_score_threshold_;
  int mapping_free_score_threshold_;
  int mapping_score_min_;
  int mapping_score_max_;
  int refinement_min_occupied_neighbor_count_;
  int refinement_min_free_neighbor_count_;

  double keyframe_distance_threshold_;
  double keyframe_yaw_threshold_;
  int submap_nodes_per_submap_;
  int loop_closure_min_node_separation_;
  double loop_closure_descriptor_threshold_;
  double loop_closure_acceptance_score_;
  double loop_closure_search_linear_window_;
  double loop_closure_search_linear_step_;
  double loop_closure_search_angular_window_deg_;
  double loop_closure_search_angular_step_deg_;
  double odom_edge_weight_;
  double loop_edge_weight_;
  int graph_optimization_iterations_;
  double graph_optimization_step_size_;
  double graph_pose_prior_translation_weight_;
  double graph_pose_prior_yaw_weight_;
  int descriptor_beams_;
};

}  // namespace amr::slam::mapper

#endif  // AMR_SLAM_MAPPER__SLAM_MAPPER_HPP_
