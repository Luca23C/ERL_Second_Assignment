#include <math.h>

#include <memory>
#include <algorithm>
#include <string>
#include <sstream>
#include <vector>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rclcpp_action/rclcpp_action.hpp"


using namespace std::chrono_literals;

class MoveToCharger : public plansys2::ActionExecutorClient
{
public:
  MoveToCharger()
  : plansys2::ActionExecutorClient("move_to_recharge_zone", 1s), x_(0.0), y_(0.0), progress_(0.0)
  {

    // Charger zone waypoints
    geometry_msgs::msg::PoseStamped wp;
    wp.pose.position.x = 1.0;
    wp.pose.position.y = 1.0;
    waypoints_["cz"] = wp;

    using namespace std::placeholders;
    pos_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose",10,std::bind(&MoveToCharger::current_pos_callback, this, _1));
  }

  void current_pos_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    current_pos_ = msg->pose.pose;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    return ActionExecutorClient::on_activate(previous_state);
  }

private:

  double getDistance(const geometry_msgs::msg::Pose & pos1, const geometry_msgs::msg::Pose & pos2)
  {
    return sqrt(
      (pos1.position.x - pos2.position.x) * (pos1.position.x - pos2.position.x) +
      (pos1.position.y - pos2.position.y) * (pos1.position.y - pos2.position.y));
  }

  void do_work()
  {
    send_feedback(0.0, "Move starting");

    navigation_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(shared_from_this(),"/navigate_to_pose");

    bool is_action_server_ready = false;
    do {
        RCLCPP_INFO(get_logger(), "Waiting for navigation action server...");
        is_action_server_ready = navigation_action_client_->wait_for_action_server(std::chrono::seconds(15));
        if (!is_action_server_ready) {
          RCLCPP_ERROR(get_logger(), "Navigation action server not ready after waiting.");
        }
    } while (!is_action_server_ready);

    RCLCPP_INFO(get_logger(), "Navigation action server ready");

    // Prendi il waypoint calcolato dalla matrice
    auto wp_to_navigate = "cz";
    if (waypoints_.find(wp_to_navigate) == waypoints_.end()) {
      RCLCPP_ERROR(get_logger(), "Waypoint [%s] not found", wp_to_navigate);
      return;
    }

    goal_pos_ = waypoints_[wp_to_navigate];
    navigation_goal_.pose = goal_pos_;

    dist_to_move = getDistance(goal_pos_.pose, current_pos_);

    RCLCPP_INFO(get_logger(), "Current Position: (%.2f, %.2f)", current_pos_.position.x, current_pos_.position.y);
    RCLCPP_INFO(get_logger(), "Goal Position: (%.2f, %.2f)", goal_pos_.pose.position.x, goal_pos_.pose.position.y);
    RCLCPP_INFO(get_logger(), "Distance to Move: %.2f", dist_to_move);

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

    send_goal_options.feedback_callback = [this](NavigationGoalHandle::SharedPtr,NavigationFeedback feedback) {
      send_feedback(std::min(1.0, std::max(0.0, 1.0 - (feedback->distance_remaining / dist_to_move))),"Move running");
    };

    send_goal_options.result_callback = [this](auto) {
      finish(true, 1.0, "Move completed");
    };

    future_navigation_goal_handle_ = navigation_action_client_->async_send_goal(navigation_goal_, send_goal_options);
  }

  // Dichiarazioni generali per la parte di navigation
  std::map<std::string, geometry_msgs::msg::PoseStamped> waypoints_;

  using NavigationGoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
  using NavigationFeedback = const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>;

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_action_client_;
  std::shared_future<NavigationGoalHandle::SharedPtr> future_navigation_goal_handle_;
  NavigationGoalHandle::SharedPtr navigation_goal_handle_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pos_sub_;
  geometry_msgs::msg::Pose current_pos_;
  geometry_msgs::msg::PoseStamped goal_pos_;
  nav2_msgs::action::NavigateToPose::Goal navigation_goal_;

  // Variables useful for the action
  double dist_to_move;
  int x_;
  int y_;
  float progress_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);  // Inizializza il sistema ROS 2
  auto node = std::make_shared<MoveToCharger>();  // Crea un oggetto di tipo MoveToCharger

  node->set_parameter(rclcpp::Parameter("action_name", "move_to_recharge_zone"));  // Imposta il nome dell'azione
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);  // Esegue la transizione di configurazione

  rclcpp::spin(node->get_node_base_interface());  // Inizia l'esecuzione del nodo

  rclcpp::shutdown();  // Spegne il sistema ROS 2

  return 0;
}