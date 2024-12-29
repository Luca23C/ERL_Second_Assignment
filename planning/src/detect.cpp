#include <memory>
#include <algorithm>

#include "rclcpp_action/rclcpp_action.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_aruco_interfaces/srv/get_map_data.hpp"

using namespace std::chrono_literals;


class Detect : public plansys2::ActionExecutorClient {
public:
  Detect()
  : plansys2::ActionExecutorClient("detect", 1s) {
    // Crea il client di servizio
    client_ = this->create_client<ros2_aruco_interfaces::srv::GetMapData>("get_map_data");

    // Controlla se il servizio è disponibile
    if (!client_->wait_for_service(5s)) {
      RCLCPP_ERROR(this->get_logger(), "Il servizio 'get_map_data' non è disponibile");
      throw std::runtime_error("Il servizio 'get_map_data' non è disponibile");
    }
  }

private:

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State & previous_state){
    // Crea una richiesta
    auto request = std::make_shared<ros2_aruco_interfaces::srv::GetMapData::Request>();

    // Invia la richiesta al servizio
    auto result_future = client_->async_send_request(request);

    // Attendi il risultato
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == rclcpp::FutureReturnCode::SUCCESS){
      auto response = result_future.get();
      RCLCPP_INFO(this->get_logger(), "Risultato del servizio: %ld", response->x, response->y, response->marker_id);
    }
    else {
      RCLCPP_ERROR(this->get_logger(), "Errore nella chiamata al servizio");
    }

    finish(true, 1.0, "Detect completed");
    return ActionExecutorClient::on_activate(previous_state);
  }

  rclcpp::Client<ros2_aruco_interfaces::srv::GetMapData>::SharedPtr client_;
  
  void do_work(){}
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);  // Inizializza il sistema ROS 2
  auto node = std::make_shared<Detect>();  // Crea un oggetto di tipo Detect

  node->set_parameter(rclcpp::Parameter("action_name", "detect"));  // Imposta il nome dell'azione
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);  // Esegue la transizione di configurazione

  rclcpp::spin(node->get_node_base_interface());  // Inizia l'esecuzione del nodo

  rclcpp::shutdown();  // Spegne il sistema ROS 2

  return 0;
}

