#include <cstring>
#include <iostream>
#include <memory>
#include <string>

// Bibliotecas nativas do Linux para comunicação via Sockets UDP
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class ESP32ToFlightGearBridge : public rclcpp::Node {
public:
  ESP32ToFlightGearBridge() : Node("esp32_to_flightgear_bridge") {
    // Configura a conexão UDP com o simulador
    setup_udp_socket();

    // Subscreve ao retorno do micro-ROS agent
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "flightgear_attitude_echo", 10,
        std::bind(&ESP32ToFlightGearBridge::echo_callback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "Nó de Ponte (ESP32 -> FlightGear) iniciado.");
    RCLCPP_INFO(this->get_logger(),
                "Escutando eco em /flightgear_attitude_echo e enviando para o "
                "FlightGear.");
  }

  // Garante o fechamento correto do socket ao encerrar o nó
  ~ESP32ToFlightGearBridge() {
    if (sockfd_ != -1) {
      close(sockfd_);
      RCLCPP_INFO(this->get_logger(), "Socket UDP fechado.");
    }
  }

private:
  void setup_udp_socket() {
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0) {
      RCLCPP_FATAL(this->get_logger(), "Falha ao criar o socket UDP.");
      return;
    }

    std::memset(&dest_addr_, 0, sizeof(dest_addr_));
    dest_addr_.sin_family = AF_INET;
    dest_addr_.sin_port = htons(
        5000); // Altere para a porta de entrada configurada no seu FlightGear

    // IP do simulador (localhost se estiver rodando no mesmo computador)
    if (inet_pton(AF_INET, "127.0.0.1", &dest_addr_.sin_addr) <= 0) {
      RCLCPP_FATAL(this->get_logger(),
                   "Endereço IP inválido para o FlightGear.");
    }
  }

  void echo_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    // Garante que o array possui ao menos 3 eixos válidos
    if (msg->axes.size() >= 3) {
      double roll = msg->axes[0];
      double pitch = msg->axes[1];
      double yaw = msg->axes[2];

      // Monta a string de dados conforme o protocolo que você definiu no XML do
      // FlightGear Exemplo comum: valores separados por quebra de linha ou
      // vírgula
      std::string packet = std::to_string(roll) + "," + std::to_string(pitch) +
                           "," + std::to_string(yaw) + "\n";

      // Transmite o pacote via rede
      if (sockfd_ != -1) {
        sendto(sockfd_, packet.c_str(), packet.length(), 0,
               (struct sockaddr *)&dest_addr_, sizeof(dest_addr_));
      }

      RCLCPP_INFO(
          this->get_logger(),
          "Despachado para o Simulador -> Roll: %.3f | Pitch: %.3f | Yaw: %.3f",
          roll, pitch, yaw);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Mensagem recebida do ESP32 com dados incompletos de eixos.");
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  int sockfd_ = -1;
  struct sockaddr_in dest_addr_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ESP32ToFlightGearBridge>());
  rclcpp::shutdown();
  return 0;
}