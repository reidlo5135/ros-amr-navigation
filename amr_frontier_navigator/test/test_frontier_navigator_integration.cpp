#include "amr_frontier_navigator/frontier_navigator.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "amr_msgs/action/navigate_to_pose.hpp"
#include "amr_msgs/action/navigate_to_unknown_pose.hpp"
#include "amr_msgs/msg/frontier_navigation_status.hpp"
#include "amr_msgs/srv/plan_segment.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

namespace
{

using namespace std::chrono_literals;
using NavigateToPose = amr_msgs::action::NavigateToPose;
using NavigateToUnknownPose = amr_msgs::action::NavigateToUnknownPose;
using UnknownGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToUnknownPose>;
using DelegatedGoalHandle = rclcpp_action::ServerGoalHandle<NavigateToPose>;

nav_msgs::msg::OccupancyGrid make_map()
{
  nav_msgs::msg::OccupancyGrid map;
  map.header.frame_id = "map";
  map.info.width = 12U;
  map.info.height = 12U;
  map.info.resolution = 0.5F;
  map.info.origin.orientation.w = 1.0;
  map.data.assign(map.info.width * map.info.height, 0);
  return map;
}

geometry_msgs::msg::PoseStamped make_pose(double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation = amr::frontier_navigation::quaternion_from_yaw(yaw);
  return pose;
}

void set_cell(nav_msgs::msg::OccupancyGrid &map, int x, int y, int8_t value)
{
  map.data[static_cast<std::size_t>(y * static_cast<int>(map.info.width) + x)] = value;
}

class FakeNavigationInterfaces
{
public:
  explicit FakeNavigationInterfaces(bool hold_first_goal = false)
  : node(std::make_shared<rclcpp::Node>("frontier_fake_interfaces")),
    hold_first_goal_(hold_first_goal)
  {
    map_publisher = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).reliable().transient_local());
    planner_service = node->create_service<amr_msgs::srv::PlanSegment>(
      "/plan_segment",
      [this](
        const std::shared_ptr<amr_msgs::srv::PlanSegment::Request> request,
        std::shared_ptr<amr_msgs::srv::PlanSegment::Response> response) {
        std::scoped_lock lock(mutex_);
        ++plan_request_count_;
        response->success =
          planner_available_ && !reject_plans_ && plan_request_count_ > rejected_plan_count_;
        response->message = response->success ? "reachable" : "unreachable";
        if (response->success) {
          response->plan.header.frame_id = "map";
          response->plan.poses = {request->start, request->goal};
        }
        condition_.notify_all();
      });
    action_server = rclcpp_action::create_server<NavigateToPose>(
      node,
      "/navigate_to_pose",
      [this](
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const NavigateToPose::Goal>) {
        std::scoped_lock lock(mutex_);
        if (active_delegated_goals_ > 0U) {
          ++overlap_rejection_count_;
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<DelegatedGoalHandle> goal_handle) {
        std::scoped_lock lock(mutex_);
        ++cancel_request_count_;
        pending_cancel_ = goal_handle;
        condition_.notify_all();
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<DelegatedGoalHandle> goal_handle) {
        bool finish_now = false;
        {
          std::scoped_lock lock(mutex_);
          delegated_goals_.push_back(goal_handle->get_goal()->goal_pose);
          ++active_delegated_goals_;
          max_active_delegated_goals_ =
            std::max(max_active_delegated_goals_, active_delegated_goals_);
          finish_now = !hold_first_goal_ || delegated_goals_.size() > 1U;
          condition_.notify_all();
        }
        if (finish_now) {
          {
            std::scoped_lock lock(mutex_);
            --active_delegated_goals_;
          }
          auto result = std::make_shared<NavigateToPose::Result>();
          result->error_code = NavigateToPose::Result::NONE;
          goal_handle->succeed(result);
          std::scoped_lock lock(mutex_);
          condition_.notify_all();
        }
      });
    cancel_timer_ = node->create_wall_timer(5ms, [this]() { finish_pending_cancel(); });
    tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
  }

  void publish_map(nav_msgs::msg::OccupancyGrid map)
  {
    map.header.stamp = node->now();
    map_publisher->publish(map);
  }

  void publish_robot_transform()
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "map";
    transform.child_frame_id = "base_footprint";
    transform.header.stamp = node->now();
    transform.transform.translation.x = 0.25;
    transform.transform.translation.y = 0.25;
    transform.transform.rotation.w = 1.0;
    tf_broadcaster->sendTransform(transform);
  }

  bool wait_for_delegated_goals(std::size_t count, std::chrono::milliseconds timeout)
  {
    std::unique_lock lock(mutex_);
    return condition_.wait_for(lock, timeout, [this, count]() {
      return delegated_goals_.size() >= count;
    });
  }

  std::vector<geometry_msgs::msg::PoseStamped> delegated_goals() const
  {
    std::scoped_lock lock(mutex_);
    return delegated_goals_;
  }

  std::size_t cancel_requests() const
  {
    std::scoped_lock lock(mutex_);
    return cancel_request_count_;
  }

  std::size_t max_active_goals() const
  {
    std::scoped_lock lock(mutex_);
    return max_active_delegated_goals_;
  }

  std::size_t overlap_rejections() const
  {
    std::scoped_lock lock(mutex_);
    return overlap_rejection_count_;
  }

  void reject_first_plans(std::size_t count)
  {
    std::scoped_lock lock(mutex_);
    rejected_plan_count_ = count;
  }

  std::size_t plan_requests() const
  {
    std::scoped_lock lock(mutex_);
    return plan_request_count_;
  }

  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher;
  rclcpp::Service<amr_msgs::srv::PlanSegment>::SharedPtr planner_service;
  rclcpp_action::Server<NavigateToPose>::SharedPtr action_server;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster;

private:
  void finish_pending_cancel()
  {
    std::shared_ptr<DelegatedGoalHandle> goal_handle;
    {
      std::scoped_lock lock(mutex_);
      if (!pending_cancel_ || !pending_cancel_->is_canceling()) {
        return;
      }
      goal_handle = std::move(pending_cancel_);
    }
    auto result = std::make_shared<NavigateToPose::Result>();
    result->error_code = NavigateToPose::Result::UNKNOWN;
    result->error_msg = "canceled by fake server";
    {
      std::scoped_lock lock(mutex_);
      --active_delegated_goals_;
    }
    goal_handle->canceled(result);
    {
      std::scoped_lock lock(mutex_);
      condition_.notify_all();
    }
  }

  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::vector<geometry_msgs::msg::PoseStamped> delegated_goals_;
  std::shared_ptr<DelegatedGoalHandle> pending_cancel_;
  rclcpp::TimerBase::SharedPtr cancel_timer_;
  std::size_t active_delegated_goals_{0U};
  std::size_t max_active_delegated_goals_{0U};
  std::size_t cancel_request_count_{0U};
  std::size_t overlap_rejection_count_{0U};
  std::size_t plan_request_count_{0U};
  std::size_t rejected_plan_count_{0U};
  bool hold_first_goal_{false};
  bool planner_available_{true};
  bool reject_plans_{false};
};

class FrontierNavigatorIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    frontier = std::make_shared<amr::frontier_navigation::FrontierNavigator>(
      rclcpp::NodeOptions().parameter_overrides({
        rclcpp::Parameter("resolver.map_wait_timeout_sec", 0.4),
        rclcpp::Parameter("resolver.goal_known_wait_timeout_sec", 0.2),
        rclcpp::Parameter("resolver.max_staging_search_radius_m", 1.0),
        rclcpp::Parameter("execution.action_server_wait_timeout_ms", 500),
        rclcpp::Parameter("execution.nested_cancel_timeout_ms", 500),
        rclcpp::Parameter("execution.planner_wait_timeout_ms", 500),
        rclcpp::Parameter("execution.feedback_period_ms", 20),
      }));
    client_node = std::make_shared<rclcpp::Node>("frontier_integration_client");
    unknown_client = rclcpp_action::create_client<NavigateToUnknownPose>(
      client_node, "/navigate_to_unknown_pose");
    status_subscription = client_node->create_subscription<
      amr_msgs::msg::FrontierNavigationStatus>(
      "/frontier/status", rclcpp::QoS(1).reliable().transient_local(),
      [this](const amr_msgs::msg::FrontierNavigationStatus::SharedPtr message) {
        std::scoped_lock lock(observation_mutex);
        phases.push_back(message->phase);
      });
    unknown_goal_subscription = client_node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/frontier/unknown_goal", rclcpp::QoS(1).reliable().transient_local(),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
        if (!message->header.frame_id.empty()) {
          std::scoped_lock lock(observation_mutex);
          observed_unknown_goal = *message;
        }
      });
    known_goal_subscription = client_node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/frontier/known_goal", rclcpp::QoS(1).reliable().transient_local(),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
        if (!message->header.frame_id.empty()) {
          std::scoped_lock lock(observation_mutex);
          observed_known_goal = *message;
        }
      });
    global_path_subscription = client_node->create_subscription<nav_msgs::msg::Path>(
      "/frontier/global_plan", rclcpp::SystemDefaultsQoS(),
      [this](const nav_msgs::msg::Path::SharedPtr message) {
        if (!message->poses.empty()) {
          std::scoped_lock lock(observation_mutex);
          ++nonempty_global_paths;
          observation_condition.notify_all();
        }
      });
  }

  void start(const std::shared_ptr<FakeNavigationInterfaces> &fake)
  {
    fake_interfaces = fake;
    executor.add_node(frontier->get_node_base_interface());
    executor.add_node(client_node);
    executor.add_node(fake->node);
    spin_thread = std::thread([this]() { executor.spin(); });
    frontier->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    ASSERT_EQ(
      frontier->get_current_state().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    frontier->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    ASSERT_EQ(
      frontier->get_current_state().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    fake->publish_robot_transform();
    ASSERT_TRUE(unknown_client->wait_for_action_server(2s));
  }

  std::pair<std::shared_ptr<UnknownGoalHandle>, UnknownGoalHandle::WrappedResult> send_and_wait(
    const geometry_msgs::msg::PoseStamped &pose,
    std::chrono::milliseconds timeout = 3s)
  {
    NavigateToUnknownPose::Goal goal;
    goal.unknown_goal = pose;
    goal.max_iterations = 2U;
    auto goal_future = unknown_client->async_send_goal(goal);
    EXPECT_EQ(goal_future.wait_for(timeout), std::future_status::ready);
    auto goal_handle = goal_future.get();
    EXPECT_NE(goal_handle, nullptr);
    auto result_future = unknown_client->async_get_result(goal_handle);
    EXPECT_EQ(result_future.wait_for(timeout), std::future_status::ready);
    return {goal_handle, result_future.get()};
  }

  bool saw_phase(const std::string &phase) const
  {
    std::scoped_lock lock(observation_mutex);
    return std::find(phases.begin(), phases.end(), phase) != phases.end();
  }

  bool wait_for_nonempty_global_path(std::chrono::milliseconds timeout)
  {
    std::unique_lock lock(observation_mutex);
    return observation_condition.wait_for(lock, timeout, [this]() {
      return nonempty_global_paths > 0U;
    });
  }

  void TearDown() override
  {
    if (frontier &&
      frontier->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      frontier->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
    }
    executor.cancel();
    if (spin_thread.joinable()) {
      spin_thread.join();
    }
    fake_interfaces.reset();
  }

  rclcpp::executors::MultiThreadedExecutor executor;
  std::shared_ptr<amr::frontier_navigation::FrontierNavigator> frontier;
  rclcpp::Node::SharedPtr client_node;
  rclcpp_action::Client<NavigateToUnknownPose>::SharedPtr unknown_client;
  rclcpp::Subscription<amr_msgs::msg::FrontierNavigationStatus>::SharedPtr status_subscription;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr unknown_goal_subscription;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr known_goal_subscription;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_subscription;
  std::thread spin_thread;
  std::shared_ptr<FakeNavigationInterfaces> fake_interfaces;
  mutable std::mutex observation_mutex;
  std::condition_variable observation_condition;
  std::vector<std::string> phases;
  geometry_msgs::msg::PoseStamped observed_unknown_goal;
  geometry_msgs::msg::PoseStamped observed_known_goal;
  std::size_t nonempty_global_paths{0U};
};

TEST_F(FrontierNavigatorIntegrationTest, DelegatesKnownGoalOnceAndPreservesPose)
{
  auto fake = std::make_shared<FakeNavigationInterfaces>();
  start(fake);
  fake->publish_map(make_map());

  const auto original = make_pose(2.25, 2.25, 1.1);
  const auto [goal_handle, wrapped_result] = send_and_wait(original);
  (void)goal_handle;

  ASSERT_EQ(wrapped_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_NE(wrapped_result.result, nullptr);
  EXPECT_EQ(wrapped_result.result->error_code, NavigateToUnknownPose::Result::NONE);
  EXPECT_TRUE(wrapped_result.result->final_goal_reached);
  EXPECT_FALSE(wrapped_result.result->original_goal_became_known);
  EXPECT_EQ(wrapped_result.result->iterations, 0U);
  const auto delegated = fake->delegated_goals();
  ASSERT_EQ(delegated.size(), 1U);
  EXPECT_DOUBLE_EQ(delegated.front().pose.position.x, original.pose.position.x);
  EXPECT_DOUBLE_EQ(delegated.front().pose.position.y, original.pose.position.y);
  EXPECT_DOUBLE_EQ(delegated.front().pose.orientation.z, original.pose.orientation.z);
  EXPECT_DOUBLE_EQ(delegated.front().pose.orientation.w, original.pose.orientation.w);
  EXPECT_TRUE(saw_phase("GOAL_ALREADY_KNOWN"));
}

TEST_F(FrontierNavigatorIntegrationTest, WaitsForStagingCancelBeforeOriginalGoal)
{
  auto fake = std::make_shared<FakeNavigationInterfaces>(true);
  start(fake);
  auto initial_map = make_map();
  set_cell(initial_map, 6, 6, -1);
  fake->publish_map(initial_map);

  const auto original = make_pose(3.25, 3.25, -0.7);
  NavigateToUnknownPose::Goal goal;
  goal.unknown_goal = original;
  goal.max_iterations = 2U;
  auto goal_future = unknown_client->async_send_goal(goal);
  ASSERT_EQ(goal_future.wait_for(2s), std::future_status::ready);
  auto goal_handle = goal_future.get();
  ASSERT_NE(goal_handle, nullptr);
  ASSERT_TRUE(fake->wait_for_delegated_goals(1U, 2s));

  auto expanded_map = initial_map;
  set_cell(expanded_map, 6, 6, 0);
  fake->publish_map(expanded_map);

  auto result_future = unknown_client->async_get_result(goal_handle);
  ASSERT_EQ(result_future.wait_for(3s), std::future_status::ready);
  const auto wrapped_result = result_future.get();
  ASSERT_EQ(wrapped_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_NE(wrapped_result.result, nullptr);
  EXPECT_TRUE(wrapped_result.result->original_goal_became_known);
  EXPECT_TRUE(wrapped_result.result->final_goal_reached);
  EXPECT_EQ(fake->cancel_requests(), 1U);
  EXPECT_EQ(fake->max_active_goals(), 1U);
  EXPECT_EQ(fake->overlap_rejections(), 0U);
  const auto delegated = fake->delegated_goals();
  ASSERT_EQ(delegated.size(), 2U);
  EXPECT_NE(delegated.front().pose.position.x, original.pose.position.x);
  EXPECT_DOUBLE_EQ(delegated.back().pose.position.x, original.pose.position.x);
  EXPECT_DOUBLE_EQ(delegated.back().pose.position.y, original.pose.position.y);
  EXPECT_DOUBLE_EQ(delegated.back().pose.orientation.z, original.pose.orientation.z);
  EXPECT_DOUBLE_EQ(delegated.back().pose.orientation.w, original.pose.orientation.w);
  EXPECT_TRUE(saw_phase("NAVIGATING_TO_STAGING_GOAL"));
  EXPECT_TRUE(saw_phase("NAVIGATING_TO_ORIGINAL_GOAL"));
}

TEST_F(FrontierNavigatorIntegrationTest, FallsBackToReachableMapFrontierForOutOfBoundsGoal)
{
  auto fake = std::make_shared<FakeNavigationInterfaces>(true);
  fake->reject_first_plans(2U);
  start(fake);
  fake->publish_map(make_map());

  NavigateToUnknownPose::Goal goal;
  goal.unknown_goal = make_pose(8.25, 3.25, 0.2);
  goal.max_iterations = 2U;
  goal.max_staging_search_radius_m = 0.5F;
  auto goal_future = unknown_client->async_send_goal(goal);
  ASSERT_EQ(goal_future.wait_for(2s), std::future_status::ready);
  auto goal_handle = goal_future.get();
  ASSERT_NE(goal_handle, nullptr);
  ASSERT_TRUE(fake->wait_for_delegated_goals(1U, 2s));
  ASSERT_TRUE(wait_for_nonempty_global_path(2s));

  const auto delegated = fake->delegated_goals();
  ASSERT_EQ(delegated.size(), 1U);
  EXPECT_GE(delegated.front().pose.position.x, 0.0);
  EXPECT_LT(delegated.front().pose.position.x, 6.0);
  EXPECT_GE(delegated.front().pose.position.y, 0.0);
  EXPECT_LT(delegated.front().pose.position.y, 6.0);
  EXPECT_GT(fake->plan_requests(), 2U);
  EXPECT_TRUE(saw_phase("RESOLVING_STAGING_GOAL"));
  EXPECT_TRUE(saw_phase("NAVIGATING_TO_STAGING_GOAL"));

  auto cancel_future = unknown_client->async_cancel_goal(goal_handle);
  ASSERT_EQ(cancel_future.wait_for(2s), std::future_status::ready);
  auto result_future = unknown_client->async_get_result(goal_handle);
  ASSERT_EQ(result_future.wait_for(2s), std::future_status::ready);
  EXPECT_EQ(result_future.get().code, rclcpp_action::ResultCode::CANCELED);
}

TEST_F(FrontierNavigatorIntegrationTest, CancelWhileWaitingForMapAllowsNextGoal)
{
  auto fake = std::make_shared<FakeNavigationInterfaces>();
  start(fake);

  NavigateToUnknownPose::Goal first_goal;
  first_goal.unknown_goal = make_pose(1.25, 1.25, 0.0);
  auto first_goal_future = unknown_client->async_send_goal(first_goal);
  ASSERT_EQ(first_goal_future.wait_for(2s), std::future_status::ready);
  auto first_handle = first_goal_future.get();
  ASSERT_NE(first_handle, nullptr);
  auto cancel_future = unknown_client->async_cancel_goal(first_handle);
  ASSERT_EQ(cancel_future.wait_for(2s), std::future_status::ready);
  auto first_result_future = unknown_client->async_get_result(first_handle);
  ASSERT_EQ(first_result_future.wait_for(2s), std::future_status::ready);
  const auto first_result = first_result_future.get();
  EXPECT_EQ(first_result.code, rclcpp_action::ResultCode::CANCELED);
  ASSERT_NE(first_result.result, nullptr);
  EXPECT_EQ(first_result.result->error_code, NavigateToUnknownPose::Result::CANCELED);

  fake->publish_map(make_map());
  const auto [second_handle, second_result] = send_and_wait(make_pose(2.25, 2.25, 0.4));
  (void)second_handle;
  EXPECT_EQ(second_result.code, rclcpp_action::ResultCode::SUCCEEDED);
}

TEST_F(FrontierNavigatorIntegrationTest, OccupiedGoalFailsWithoutDelegation)
{
  auto fake = std::make_shared<FakeNavigationInterfaces>();
  start(fake);
  auto map = make_map();
  set_cell(map, 4, 4, 100);
  fake->publish_map(map);

  const auto [goal_handle, wrapped_result] = send_and_wait(make_pose(2.25, 2.25, 0.0));
  (void)goal_handle;
  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_NE(wrapped_result.result, nullptr);
  EXPECT_EQ(wrapped_result.result->error_code, NavigateToUnknownPose::Result::OCCUPIED_GOAL);
  EXPECT_TRUE(fake->delegated_goals().empty());
}

}  // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
