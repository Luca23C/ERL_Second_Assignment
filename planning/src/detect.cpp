#include <memory>
#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros2_aruco_interfaces/srv/get_map_data.hpp"  // Inclusione del servizio

#include <std_msgs/msg/string.hpp>
// Service for getting map data
#include <sstream>
#include <vector>
using namespace std::chrono_literals;


class Detect : public plansys2::ActionExecutorClient
{
public:
  Detect()
  : plansys2::ActionExecutorClient("detect", 1s), progress_(0.0)
  {
    // Inizializza il client di servizio
    service_client_ = this->create_client<ros2_aruco_interfaces::srv::GetMapData>("/get_map_data");

    // Publisher per pubblicare la matrice aggiornata
    publisher_ = this->create_publisher<std_msgs::msg::String>("matrix_topic", 10);

    // Timer per la pubblicazione periodica della matrice aggiornata
    timer_ = this->create_wall_timer(std::chrono::seconds(1),std::bind(&Detect::publish_updated_matrix, this));

    float x, y, marker_id;
  }

private:
  void do_work()
  {
    if (progress_ == 0.0) {
      // Chiamata al servizio alla prima iterazione
      if (!service_client_->wait_for_service(1s)) {
        //RCLCPP_ERROR(this->get_logger(), "Service /get_map_data not available");
        finish(false, 0.0, "Service not available");
        return;
      }

      // Chiamata al servizio per ottenere le informazioni necessarie (marker id e posizione x, y)
      auto request = std::make_shared<ros2_aruco_interfaces::srv::GetMapData::Request>();
      auto garbage = service_client_->async_send_request(request, [this](rclcpp::Client<ros2_aruco_interfaces::srv::GetMapData>::SharedFuture response) {
        try {
          auto result = response.get();
          //RCLCPP_INFO(this->get_logger(), "junk del servizio: x=%.2f, y=%.2f, marker_id=%.2f", 
          //      result->x, result->y, result->marker_id);
          //RCLCPP_INFO(this->get_logger(), "Received map data: %s", result->map_data.c_str());
        } catch (const std::exception &e) {
          //RCLCPP_ERROR(this->get_logger(), "Failed to call service: %s", e.what());
          finish(false, 0.0, "Service call failed");
        }
      });
    }

    // Simulazione di un processo di rilevamento
    if (progress_ < 1.0) {
      progress_ += 0.05;  // Aumenta il progresso del 5% ad ogni chiamata
      send_feedback(progress_, "Detect running");  // Invia feedback con il progresso
    } 
    else {
      
      // Chiamata al servizio per ottenere le informazioni necessarie (marker id e posizione x, y)
      auto request = std::make_shared<ros2_aruco_interfaces::srv::GetMapData::Request>();
      auto future = service_client_->async_send_request(request, [this](rclcpp::Client<ros2_aruco_interfaces::srv::GetMapData>::SharedFuture response) {
        try {
          auto result = response.get();
            
          //RCLCPP_INFO(this->get_logger(), "Risultato del servizio: x=%.2f, y=%.2f, marker_id=%ld", result->x, result->y, result->marker_id);
          //RCLCPP_INFO(this->get_logger(), "Received map data: %s", result->map_data.c_str());
          update_matrix(result->x, result->y, result->marker_id); // Update matrix based on service result
          publish_updated_matrix();
        } catch (const std::exception &e) {
          //RCLCPP_ERROR(this->get_logger(), "Failed to call service: %.2f", e.what());
          finish(false, 0.0, "Service call failed");
        }
      });
    finish(true, 1.0, "Detect completed");  // Completa l'azione quando il progresso è 1.0

    progress_ = 0.0;  // Reset del progresso
    std::cout << std::endl;
  }

    // Stampa il progresso a schermo
    std::cout << "\r\e[K" << std::flush;
    std::cout << "Detecting ... [" << std::min(100.0, progress_ * 100.0) << "%]  " <<
    std::flush;
  }

  void update_matrix(double x, double y, double marker_id) {
    bool updated = false; // Variabile per tenere traccia se la matrice è stata aggiornata

    for (int row = 0; row < 4; ++row) {
      if (matrix_[row][0] == 0 && matrix_[row][1] == 0 && matrix_[row][2] == 0) {
        matrix_[row][0] = static_cast<double>(marker_id); // Trasformiamo marker_id in double
        matrix_[row][1] = x;
        matrix_[row][2] = y;
        //RCLCPP_INFO(this->get_logger(), "Matrix updated at row %d: [%f, %f, %f]", row, x, y, static_cast<double>(marker_id));
        updated = true; // Segna che la matrice è stata aggiornata
        break; // Esci dal ciclo dopo aver trovato la prima riga disponibile
      }
    }

    if (!updated) {
        // Se non abbiamo trovato nessuna riga disponibile (la matrice è piena)
        RCLCPP_WARN(this->get_logger(), "No available row to update, matrix is full.");
    }
   }

  void matrix_callback(const std_msgs::msg::String::SharedPtr msg) {
    // Riceve la matrice come stringa e la converte in formato interno
    //RCLCPP_INFO(this->get_logger(), "Received matrix: %s", msg->data.c_str());

    std::istringstream matrix_stream(msg->data);
    std::string line;
    int row = 0;

    while (std::getline(matrix_stream, line) && row < 4) {
      std::istringstream row_stream(line);
      char discard;
      double val;

      row_stream >> discard;  // Salta '['
      for (int col = 0; col < 3; ++col) {
        if (row_stream >> val) {
          matrix_[row][col] = val;
          if (col < 2) row_stream >> discard;  // Salta ','
        }
      }
      ++row;
    }
  }

  void publish_updated_matrix() {
    std::ostringstream matrix_stream;
    matrix_stream << "[\n";

    // Costruisci la stringa della matrice
    for (const auto &row : matrix_) {
      matrix_stream << "    [";
      for (size_t i = 0; i < row.size(); ++i) {
        matrix_stream << row[i];
        if (i < row.size() - 1) matrix_stream << ", ";
      }
      matrix_stream << "],\n";
    }

    matrix_stream << "]";

    // Pubblica la matrice aggiornata
    auto message = std_msgs::msg::String();
    message.data = matrix_stream.str();
    publisher_->on_activate();
    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Published updated matrix: %s", message.data.c_str());

  }

  std::vector<std::vector<double>>  matrix_ = std::vector<std::vector<double>>(4, std::vector<double>(3, 0.0)); 
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr publisher_;
  float progress_;  // Variabile per tenere traccia del progresso dell'azione
  rclcpp::Client<ros2_aruco_interfaces::srv::GetMapData>::SharedPtr service_client_;  // Client per il servizio

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);  // Inizializza il sistema ROS 2
  auto node = std::make_shared<Detect>();  // Crea un oggetto di tipo Detect

  node->set_parameter(rclcpp::Parameter("action_name", "detect"));  // Imposta il nome dell'azione
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);  // Esegue la transizione di configurazione
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);


  rclcpp::spin(node->get_node_base_interface());  // Inizia l'esecuzione del nodo

  rclcpp::shutdown();  // Spegne il sistema ROS 2

  return 0;
}