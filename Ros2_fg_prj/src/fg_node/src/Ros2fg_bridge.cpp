#include <atomic>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

// Bibliotecas nativas do Linux para comunicação via Sockets UDP
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32.hpp"

class Ros2MainToFlightGearBridge : public rclcpp::Node {
public:
  Ros2MainToFlightGearBridge() : Node("Ros2_main_to_flightgear_bridge") {
    // Configura a conexão UDP com o simulador (envio)
    setup_udp_socket();

    // Publishers (Saída para o ESP32)
    rudder_pub_ = this->create_publisher<std_msgs::msg::Float32>("rudder_pos", 10);
    aileron_pub_ = this->create_publisher<std_msgs::msg::Float32>("aileron_pos", 10);
    elevator_pub_ = this->create_publisher<std_msgs::msg::Float32>("elevator_pos", 10);
    throttle_pub_ = this->create_publisher<std_msgs::msg::Float32>("throttle_pos", 10);
    flap_pub_ = this->create_publisher<std_msgs::msg::Float32>("flap_pos", 10);

    // Subscribers (Entrada do py_6dof)
    sub_rudder_ = this->create_subscription<std_msgs::msg::Float32>(
        "rudder_pos_servo_sub", 10,
        [this](const std_msgs::msg::Float32::SharedPtr msg) { rudder_pub_->publish(*msg); });
        
    sub_aileron_ = this->create_subscription<std_msgs::msg::Float32>(
        "aileron_pos_servo_sub", 10,
        [this](const std_msgs::msg::Float32::SharedPtr msg) { aileron_pub_->publish(*msg); });
        
    sub_elevator_ = this->create_subscription<std_msgs::msg::Float32>(
        "elevator_pos_servo_sub", 10,
        [this](const std_msgs::msg::Float32::SharedPtr msg) { elevator_pub_->publish(*msg); });
        
    sub_throttle_ = this->create_subscription<std_msgs::msg::Float32>(
        "throttle_pos_servo_sub", 10,
        [this](const std_msgs::msg::Float32::SharedPtr msg) { throttle_pub_->publish(*msg); });
        
    sub_flap_ = this->create_subscription<std_msgs::msg::Float32>(
        "flap_pos_servo_sub", 10,
        [this](const std_msgs::msg::Float32::SharedPtr msg) { flap_pub_->publish(*msg); });

    // Subscreve ao retorno do micro-ROS agent
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy_processed", 10,
        std::bind(&Ros2MainToFlightGearBridge::echo_callback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "Nó de Ponte (Ros2_main <-> FlightGear) atualizado.");
    RCLCPP_INFO(this->get_logger(),
                "Recebendo de *_servo_sub e repassando para *_pos.");
  }

  // Garante o fechamento correto dos sockets
  ~Ros2MainToFlightGearBridge() {
    if (sockfd_ != -1) {
      close(sockfd_);
      RCLCPP_INFO(this->get_logger(), "Socket UDP de envio fechado.");
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
    dest_addr_.sin_port = htons(5000); 

    // IP do simulador (localhost se estiver rodando no mesmo computador)
    if (inet_pton(AF_INET, "127.0.0.1", &dest_addr_.sin_addr) <= 0) {
      RCLCPP_FATAL(this->get_logger(),
                   "Endereço IP inválido para o FlightGear.");
    }
  }

  void echo_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    if (msg->axes.size() >= 3) {
      double roll = msg->axes[0];
      double pitch = msg->axes[1];
      double yaw = msg->axes[2];

      std::string packet = std::to_string(roll) + "," + std::to_string(pitch) +
                           "," + std::to_string(yaw) + "\n";

      if (sockfd_ != -1) {
        sendto(sockfd_, packet.c_str(), packet.length(), 0,
               (struct sockaddr *)&dest_addr_, sizeof(dest_addr_));
      }

      RCLCPP_INFO(this->get_logger(),
                  "Despachado para o Simulador -> Roll: %.3f | Pitch: %.3f | "
                  "Yaw: %.3f",
                  roll, pitch, yaw);
    } else {
      RCLCPP_WARN(
          this->get_logger(),
          "Mensagem recebida do Ros2_main com dados incompletos de eixos.");
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_rudder_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_aileron_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_elevator_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_throttle_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_flap_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rudder_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr aileron_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr elevator_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr flap_pub_;

  int sockfd_ = -1;
  struct sockaddr_in dest_addr_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ros2MainToFlightGearBridge>());
  rclcpp::shutdown();
  return 0;
}
