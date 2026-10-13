// waypoint_navigator.cpp
// Sends a list of (x, y, yaw) goals to Nav2's NavigateToPose action server,
// one after the other, and stops if a goal fails.

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class WaypointNavigator : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  WaypointNavigator()
  : Node("waypoint_navigator")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    const auto flat = declare_parameter<std::vector<double>>(
      "waypoints", std::vector<double>{0.5, 0.0, 0.0, 0.5, 0.5, 1.57});

    if (flat.empty() || flat.size() % 3 != 0) {
      throw std::invalid_argument(
              "'waypoints' must be a non-empty list of (x, y, yaw) triples");
    }
    for (size_t i = 0; i < flat.size(); i += 3) {
      waypoints_.push_back(make_pose(flat[i], flat[i + 1], flat[i + 2]));
    }

    client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // Check once per second until Nav2 is up, then start (see start()).
    timer_ = create_wall_timer(1s, std::bind(&WaypointNavigator::start, this));

    RCLCPP_INFO(get_logger(), "Loaded %zu waypoints (frame '%s')",
      waypoints_.size(), frame_id_.c_str());
  }

private:
  geometry_msgs::msg::PoseStamped make_pose(double x, double y, double yaw) const
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id_;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    // Rotation about Z only: quaternion (0, 0, sin(yaw/2), cos(yaw/2))
    pose.pose.orientation.z = std::sin(yaw / 2.0);
    pose.pose.orientation.w = std::cos(yaw / 2.0);
    return pose;
  }

  void start()
  {
    if (!client_->wait_for_action_server(100ms)) {
      RCLCPP_INFO(get_logger(), "Waiting for Nav2 action server 'navigate_to_pose'...");
      return;
    }
    timer_->cancel();
    send_next_goal();
  }

  void send_next_goal()
  {
    if (index_ >= waypoints_.size()) {
      RCLCPP_INFO(get_logger(), "All %zu waypoints reached.", waypoints_.size());
      rclcpp::shutdown();
      return;
    }

    NavigateToPose::Goal goal;
    goal.pose = waypoints_[index_];
    goal.pose.header.stamp = now();

    RCLCPP_INFO(get_logger(), "Sending waypoint %zu/%zu: x=%.2f y=%.2f",
      index_ + 1, waypoints_.size(), goal.pose.pose.position.x, goal.pose.pose.position.y);

    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    options.goal_response_callback = [this](GoalHandle::SharedPtr handle) {
        if (!handle) {
          RCLCPP_ERROR(get_logger(), "Goal was rejected by Nav2.");
          rclcpp::shutdown();
        }
      };

    options.feedback_callback =
      [this](GoalHandle::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> fb) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
          "  distance remaining: %.2f m", fb->distance_remaining);
      };

    options.result_callback = [this](const GoalHandle::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Waypoint %zu reached.", index_ + 1);
            ++index_;
            send_next_goal();
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "Waypoint %zu aborted (no valid path?). Stopping.", index_ + 1);
            rclcpp::shutdown();
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(get_logger(), "Waypoint %zu canceled. Stopping.", index_ + 1);
            rclcpp::shutdown();
            break;
          default:
            RCLCPP_ERROR(get_logger(), "Unknown result code. Stopping.");
            rclcpp::shutdown();
            break;
        }
      };

    client_->async_send_goal(goal, options);
  }

  std::string frame_id_;
  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
  size_t index_{0};
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<WaypointNavigator>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("waypoint_navigator"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
