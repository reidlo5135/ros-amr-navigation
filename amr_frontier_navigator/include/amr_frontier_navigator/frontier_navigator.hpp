#ifndef AMR_FRONTIER_NAVIGATOR__FRONTIER_NAVIGATOR_HPP_
#define AMR_FRONTIER_NAVIGATOR__FRONTIER_NAVIGATOR_HPP_

/**
 * @file frontier_navigator.hpp
 * @brief Lifecycle action orchestrator for unknown-goal frontier navigation.
 */

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "amr_frontier_navigator/frontier_utils.hpp"
#include "amr_msgs/action/navigate_to_pose.hpp"
#include "amr_msgs/action/navigate_to_unknown_pose.hpp"
#include "amr_msgs/msg/frontier_navigation_status.hpp"
#include "amr_msgs/srv/plan_segment.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace amr::frontier_navigation
{

/// @brief High-level lifecycle node that resolves unknown goals into reachable staging goals.
class FrontierNavigator : public rclcpp_lifecycle::LifecycleNode
{
public:
  /// @brief Construct the frontier navigator and declare parameters.
  explicit FrontierNavigator(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  /// @brief Destroy the node.
  ~FrontierNavigator() override = default;

private:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using NavigateToPose = amr_msgs::action::NavigateToPose;
  using NavigateToUnknownPose = amr_msgs::action::NavigateToUnknownPose;
  using GoalHandleUnknown = rclcpp_action::ServerGoalHandle<NavigateToUnknownPose>;
  using NavigateGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  /// @brief Candidate staging goal with validation metadata.
  struct StagingCandidate
  {
    geometry_msgs::msg::PoseStamped pose;
    nav_msgs::msg::Path plan;
    double distance_to_original{0.0};
    double plan_length{0.0};
    double score{0.0};
    bool frontier_adjacent{false};
  };

  /// @brief Outcome of a delegated NavigateToPose request.
  struct NavigationOutcome
  {
    bool success{false};
    bool canceled{false};
    bool preempted_for_original{false};
    uint16_t error_code{NavigateToUnknownPose::Result::UNKNOWN};
    std::string message;
  };

  CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const NavigateToUnknownPose::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleUnknown> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleUnknown> goal_handle);
  void execute(const std::shared_ptr<GoalHandleUnknown> goal_handle);

  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr message);
  void local_plan_callback(const nav_msgs::msg::Path::SharedPtr message);

  bool wait_for_map(
    double timeout_sec,
    const std::function<bool()> &is_cancel_requested);
  bool get_latest_map(nav_msgs::msg::OccupancyGrid &map) const;
  bool transform_goal_to_map(
    const geometry_msgs::msg::PoseStamped &input,
    geometry_msgs::msg::PoseStamped &output,
    std::string &error_message) const;
  bool lookup_current_pose(
    geometry_msgs::msg::PoseStamped &pose,
    std::string &error_message) const;
  CellState classify_goal(
    const geometry_msgs::msg::PoseStamped &goal,
    const nav_msgs::msg::OccupancyGrid &map) const;
  bool is_goal_known_free(const geometry_msgs::msg::PoseStamped &goal) const;
  bool wait_for_original_goal_known(
    const geometry_msgs::msg::PoseStamped &original_goal,
    double timeout_sec,
    uint16_t iteration,
    const std::shared_ptr<GoalHandleUnknown> goal_handle);
  std::optional<StagingCandidate> resolve_staging_goal(
    const geometry_msgs::msg::PoseStamped &current_pose,
    const geometry_msgs::msg::PoseStamped &original_goal,
    double max_search_radius_m,
    double min_progress_m,
    std::optional<double> previous_distance_to_original,
    std::string &error_message,
    const std::function<bool()> &is_cancel_requested);
  bool request_plan(
    const geometry_msgs::msg::PoseStamped &start,
    const geometry_msgs::msg::PoseStamped &goal,
    nav_msgs::msg::Path &plan,
    std::string &error_message,
    const std::function<bool()> &is_cancel_requested);
  NavigationOutcome navigate_to_pose(
    const geometry_msgs::msg::PoseStamped &goal,
    const geometry_msgs::msg::PoseStamped &original_goal,
    const std::string &phase,
    uint16_t iteration,
    bool monitor_original_goal,
    const std::shared_ptr<GoalHandleUnknown> goal_handle);
  void cancel_nested_goal();
  void clear_active_goal(const std::shared_ptr<GoalHandleUnknown> goal_handle);

  void publish_pose(
    const rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr &publisher,
    const geometry_msgs::msg::PoseStamped &pose) const;
  void publish_path(const nav_msgs::msg::Path &path) const;
  void publish_empty_overlays(const std::string &phase, const std::string &message);
  void publish_status_and_feedback(
    const std::string &phase,
    const geometry_msgs::msg::PoseStamped &original_goal,
    const geometry_msgs::msg::PoseStamped &active_known_goal,
    uint16_t iteration,
    bool active,
    const std::string &message,
    const std::shared_ptr<GoalHandleUnknown> goal_handle);
  void finalize_result(
    const std::shared_ptr<GoalHandleUnknown> goal_handle,
    const std::shared_ptr<NavigateToUnknownPose::Result> result,
    const std::string &state);

  std::string unknown_action_name_{"/navigate_to_unknown_pose"};
  std::string navigate_action_name_{"/navigate_to_pose"};
  std::string map_topic_{"/map"};
  std::string local_plan_topic_{"/local_plan"};
  std::string unknown_goal_topic_{"/frontier/unknown_goal"};
  std::string known_goal_topic_{"/frontier/known_goal"};
  std::string frontier_global_plan_topic_{"/frontier/global_plan"};
  std::string frontier_local_plan_topic_{"/frontier/local_plan"};
  std::string status_topic_{"/frontier/status"};
  std::string plan_segment_service_{"/plan_segment"};
  std::string map_frame_{"map"};
  std::string base_frame_{"base_footprint"};
  std::string base_fallback_frame_{"base_link"};
  double tf_lookup_timeout_sec_{0.05};
  int free_threshold_{25};
  int occupied_threshold_{65};
  int candidate_step_cells_{1};
  double min_obstacle_clearance_m_{0.20};
  double max_staging_search_radius_m_{3.0};
  double goal_tolerance_m_{0.15};
  double map_wait_timeout_sec_{5.0};
  double goal_known_wait_timeout_sec_{2.0};
  int max_iterations_{8};
  double min_staging_progress_m_{0.15};
  int max_candidate_checks_{500};
  int frontier_neighbor_radius_cells_{1};
  int action_server_wait_timeout_ms_{2000};
  int planner_wait_timeout_ms_{2000};
  int feedback_period_ms_{100};
  bool use_plan_segment_validation_{true};
  bool structured_logging_enabled_{true};

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_plan_subscription_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    unknown_goal_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    known_goal_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr
    frontier_global_plan_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr
    frontier_local_plan_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<amr_msgs::msg::FrontierNavigationStatus>::SharedPtr
    status_publisher_;
  rclcpp::Client<amr_msgs::srv::PlanSegment>::SharedPtr plan_segment_client_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_;
  rclcpp_action::Server<NavigateToUnknownPose>::SharedPtr action_server_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  mutable std::mutex map_mutex_;
  nav_msgs::msg::OccupancyGrid latest_map_;
  bool has_map_{false};

  mutable std::mutex active_goal_mutex_;
  std::shared_ptr<GoalHandleUnknown> active_goal_handle_;

  mutable std::mutex nested_goal_mutex_;
  std::shared_ptr<NavigateGoalHandle> nested_goal_handle_;
  std::atomic_bool frontier_active_{false};
};

}  // namespace amr::frontier_navigation

#endif  // AMR_FRONTIER_NAVIGATOR__FRONTIER_NAVIGATOR_HPP_
