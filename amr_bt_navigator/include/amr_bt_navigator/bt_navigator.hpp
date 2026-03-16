#ifndef AMR_BT_NAVIGATOR__BT_NAVIGATOR_HPP_
#define AMR_BT_NAVIGATOR__BT_NAVIGATOR_HPP_

#include <string>

#include "amr_msgs/action/execute_route.hpp"
#include "amr_msgs/msg/motion_command.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace amr_bt_navigator
{

class Btnavigator : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit Btnavigator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using ExecuteRoute = amr_msgs::action::ExecuteRoute;
  using GoalHandleExecuteRoute = rclcpp_action::ServerGoalHandle<ExecuteRoute>;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecuteRoute::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleExecuteRoute> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleExecuteRoute> goal_handle);
  void execute(const std::shared_ptr<GoalHandleExecuteRoute> goal_handle);

  rclcpp_action::Server<ExecuteRoute>::SharedPtr action_server_;
  rclcpp_lifecycle::LifecyclePublisher<amr_msgs::msg::MotionCommand>::SharedPtr motion_command_publisher_;
  std::string action_name_;
  std::string command_topic_;
  std::string default_node_id_;
  uint32_t next_command_id_;
};

}  // namespace amr_bt_navigator

#endif  // AMR_BT_NAVIGATOR__BT_NAVIGATOR_HPP_
