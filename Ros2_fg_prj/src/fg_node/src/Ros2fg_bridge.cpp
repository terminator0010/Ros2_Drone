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

    // Configura a conexão UDP com o simulador (recebimento)
    setup_recv_socket();

    // Publisher para a posição do leme recebida do FlightGear
    rudder_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "rudder_pos_servo_sub", 10);
    aileron_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "aileron_pos_servo_sub", 10);
    elevator_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "elevator_pos_servo_sub", 10);
    throttle_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "throttle_pos_servo_sub", 10);
    flap_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "flap_pos_servo_sub", 10);

    // Inicia a thread de recebimento
    running_ = true;
    recv_thread_ = std::thread(&Ros2MainToFlightGearBridge::recv_loop, this);

    // Subscreve ao retorno do micro-ROS agent
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy_processed", 10,
        std::bind(&Ros2MainToFlightGearBridge::echo_callback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "Nó de Ponte (Ros2_main <-> FlightGear) iniciado "
                "bidirecionalmente.");
    RCLCPP_INFO(this->get_logger(),
                "Escutando eco em /joy_processed e enviando para o "
                "FlightGear.");
  }

  // Garante o fechamento correto dos sockets e encerramento da thread
  ~Ros2MainToFlightGearBridge() {
    running_ = false;
    if (recv_thread_.joinable()) {
      recv_thread_.join();
    }

    if (sockfd_ != -1) {
      close(sockfd_);
      RCLCPP_INFO(this->get_logger(), "Socket UDP de envio fechado.");
    }

    if (recv_sockfd_ != -1) {
      close(recv_sockfd_);
      RCLCPP_INFO(this->get_logger(), "Socket UDP de recebimento fechado.");
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

      // Monta a string de dados conforme o protocolo que você definiu no XML
      // do FlightGear Exemplo comum: valores separados por quebra de linha ou
      // vírgula
      std::string packet = std::to_string(roll) + "," + std::to_string(pitch) +
                           "," + std::to_string(yaw) + "\n";

      // Transmite o pacote via rede
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

  void setup_recv_socket() {
    recv_sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (recv_sockfd_ < 0) {
      RCLCPP_FATAL(this->get_logger(),
                   "Falha ao criar o socket UDP de recebimento.");
      return;
    }

    struct sockaddr_in recv_addr;
    std::memset(&recv_addr, 0, sizeof(recv_addr));
    recv_addr.sin_family = AF_INET;
    recv_addr.sin_port = htons(5580);
    recv_addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(recv_sockfd_, (struct sockaddr *)&recv_addr, sizeof(recv_addr)) <
        0) {
      RCLCPP_FATAL(
          this->get_logger(),
          "Falha ao fazer bind no socket UDP de recebimento (porta 5580).");
    }

    // Configura timeout no socket para que a thread possa verificar a flag
    // running_ repetidamente
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000; // 100 ms
    if (setsockopt(recv_sockfd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) <
        0) {
      RCLCPP_WARN(this->get_logger(),
                  "Falha ao configurar timeout no socket de recebimento.");
    }
  }

  void recv_loop() {
    char buffer[1024];
    struct sockaddr_in sender_addr;
    socklen_t sender_len = sizeof(sender_addr);

    while (running_) {
      int n = recvfrom(recv_sockfd_, buffer, sizeof(buffer) - 1, 0,
                       (struct sockaddr *)&sender_addr, &sender_len);
      if (n > 0) {
        buffer[n] = '\0';
        std::string data(buffer);

        try {
          // Converte a string recebida para float
          // Assume-se que a string retornada do FlightGear possa ser lida com
          // std::stof
          float rudder_pos = std::stof(data);

          float aileron_pos = std::stof(data);

          float elevator_pos = std::stof(data);

          float throttle_pos = std::stof(data);

          float flap_pos = std::stof(data);

          std_msgs::msg::Float32 msg;
          msg.data = rudder_pos;
          rudder_pub_->publish(msg);

          std_msgs::msg::Float32 msg;
          msg.data = aileron_pos;
          aileron_pub_->publish(msg);

          std_msgs::msg::Float32 msg;
          msg.data = elevator_pos;
          elevator_pub_->publish(msg);

          std_msgs::msg::Float32 msg;
          msg.data = throttle_pos;
          throttle_pub_->publish(msg);

          std_msgs::msg::Float32 msg;
          msg.data = flap_pos;
          flap_pub_->publish(msg);

        } catch (const std::exception &e) {
          RCLCPP_ERROR(this->get_logger(),
                       "Erro ao converter dado do FG: '%s' (exception: %s)",
                       data.c_str(), e.what());
        }
      }
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rudder_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr aileron_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr elevator_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr flap_pub_;

  int sockfd_ = -1;
  struct sockaddr_in dest_addr_;

  int recv_sockfd_ = -1;
  std::thread recv_thread_;
  std::atomic<bool> running_{false};
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ros2MainToFlightGearBridge>());
  rclcpp::shutdown();
  return 0;
}