/*
#include <memory>
#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class MoveInOrder : public plansys2::ActionExecutorClient
{
public:
  MoveInOrder()
  : plansys2::ActionExecutorClient("move_in_order", 1s)
  {
    progress_ = 0.0;
  }

private:
  void do_work()
  {
    // Simulazione di un movimento ordinato
    if (progress_ < 1.0) {
      progress_ += 0.1;  // Aumenta il progresso del 5% ad ogni chiamata
      send_feedback(progress_, "Moving in order");  // Invia feedback con il progresso
    } else {
      finish(true, 1.0, "Movement completed");  // Completa l'azione quando il progresso è 1.0

      progress_ = 0.0;  // Reset del progresso
      std::cout << std::endl;
    }

    // Stampa il progresso a schermo
    std::cout << "\r\e[K" << std::flush;
    std::cout << "Moving in order ... [" << std::min(100.0, progress_ * 100.0) << "%]  " <<
      std::flush;
  }

  float progress_;  // Variabile per tenere traccia del progresso dell'azione
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);  // Inizializza il sistema ROS 2
  auto node = std::make_shared<MoveInOrder>();  // Crea un oggetto di tipo MoveInOrder

  node->set_parameter(rclcpp::Parameter("action_name", "move_in_order"));  // Imposta il nome dell'azione
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);  // Esegue la transizione di configurazione

  rclcpp::spin(node->get_node_base_interface());  // Inizia l'esecuzione del nodo

  rclcpp::shutdown();  // Spegne il sistema ROS 2

  return 0;
}
*/

#include <memory>
#include <algorithm>
#include <vector>
#include <string>
#include <sstream>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

// Create message for going to the lowest id
geometry_msgs::msg::PoseStamped goal;

class MoveInOrder : public plansys2::ActionExecutorClient
{
public:
  MoveInOrder()
  : plansys2::ActionExecutorClient("move_in_order", 1s), x_(0), y_(0), progress_(0.0)
  {
    // Subscriber per il topic '/matrix_topic'
    matrix_subscriber_ = this->create_subscription<std_msgs::msg::String>("/matrix_topic",10, std::bind(&MoveInOrder::matrix_callback, this, std::placeholders::_1));
  }

private:
  void matrix_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::string data = msg->data;
    std::vector<std::vector<int>> matrix;

    // Parsing del messaggio in una matrice di interi
    std::istringstream stream(data);
    std::string line;
    while (std::getline(stream, line, '\n')) {
      if (line.find("[") != std::string::npos) {
        std::vector<int> row;
        line.erase(std::remove(line.begin(), line.end(), '['), line.end());
        line.erase(std::remove(line.begin(), line.end(), ']'), line.end());
        std::istringstream row_stream(line);
        int value;
        while (row_stream >> value) {
          row.push_back(value);
          if (row_stream.peek() == ',') {
            row_stream.ignore();
          }
        }
        matrix.push_back(row);
      }
    }

    // Analisi della matrice
    if (matrix.size() >= 4) {
      int min_value = std::numeric_limits<int>::max();
      int min_index = -1;
      for (size_t i = 0; i < 4; ++i) {
        if (!matrix[i].empty() && matrix[i][0] < min_value) {
          min_value = matrix[i][0];
          min_index = i;
        }
      }

      if (min_index != -1 && matrix[min_index].size() >= 3) {
        x_ = matrix[min_index][1];
        y_ = matrix[min_index][2];

        goal.pose.position.x = x_;
        goal.pose.position.y = y_;

        RCLCPP_INFO(this->get_logger(), "Selected row: %d, x: %d, y: %d", min_index, x_, y_);
      } else {
        RCLCPP_WARN(this->get_logger(), "Matrix row %d is too short", min_index);
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Matrix has less than 4 rows");
    }
  }

  void do_work()
  {
    // Simulazione di un movimento ordinato
    if (progress_ < 1.0) {
      progress_ += 0.1;  // Aumenta il progresso del 5% ad ogni chiamata
      send_feedback(progress_, "Moving in order");  // Invia feedback con il progresso
    } else {
      finish(true, 1.0, "Movement completed");  // Completa l'azione quando il progresso è 1.0

      progress_ = 0.0;  // Reset del progresso
      std::cout << std::endl;
    }

    navigation_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(shared_from_this(), "/navigate_to_pose");

    bool is_action_server_ready = false;
    do {
      RCLCPP_INFO(get_logger(), "Waiting for navigation action server...");

      is_action_server_ready = navigation_action_client_->wait_for_action_server(std::chrono::seconds(60));
      is_action_server_ready = navigation_action_client_->wait_for_action_server(std::chrono::seconds(15));

	if (!is_action_server_ready) {
	  RCLCPP_ERROR(get_logger(), "Navigation action server not ready after waiting.");
	  //return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
	}
	RCLCPP_INFO(get_logger(), "Navigation action server ready and responding.");

        
    } while (!is_action_server_ready);

    RCLCPP_INFO(get_logger(), "Navigation action server ready");

    navigation_goal_.pose = goal;


    // Stampa il progresso a schermo
    std::cout << "\r\e[K" << std::flush;
    std::cout << "Moving in order ... [" << std::min(100.0, progress_ * 100.0) << "%]  " <<
      std::flush;
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr matrix_subscriber_;
  nav2_msgs::action::NavigateToPose::Goal navigation_goal_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_action_client_;
  int x_;  // Variabile per il secondo elemento della riga selezionata
  int y_;  // Variabile per il terzo elemento della riga selezionata
  float progress_;  // Variabile per tenere traccia del progresso dell'azione
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);  // Inizializza il sistema ROS 2
  auto node = std::make_shared<MoveInOrder>();  // Crea un oggetto di tipo MoveInOrder

  node->set_parameter(rclcpp::Parameter("action_name", "move_in_order"));  // Imposta il nome dell'azione
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);  // Esegue la transizione di configurazione

  rclcpp::spin(node->get_node_base_interface());  // Inizia l'esecuzione del nodo

  rclcpp::shutdown();  // Spegne il sistema ROS 2

  return 0;
}
