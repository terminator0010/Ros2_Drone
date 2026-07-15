#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <memory>

class JoyToESP32Bridge : public rclcpp::Node {
public:
  JoyToESP32Bridge() : Node("joy_to_esp32_bridge") {
    // Declaração de Parâmetros com os valores padrão ideais para o seu cenário
    // Já deixei o Roll como true por padrão para corrigir o seu simulador
    // imediatamente
    this->declare_parameter<bool>("invert_roll", true);
    this->declare_parameter<bool>("invert_pitch", false);
    this->declare_parameter<bool>("invert_yaw", false);
    this->declare_parameter<bool>("invert_throttle", false);

    // Escalas de sensibilidade (1.0 = 100%)
    this->declare_parameter<double>("scale_roll", 1.0);
    this->declare_parameter<double>("scale_pitch", 1.0);
    this->declare_parameter<double>("scale_yaw", 1.0);
    this->declare_parameter<double>("scale_throttle", 1.0);

    // Revertemos para a fila 10 para garantir compatibilidade total com o
    // micro-ROS (ESP32)
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 1,
        std::bind(&JoyToESP32Bridge::joy_callback, this,
                  std::placeholders::_1));

    esp32_pub_ =
        this->create_publisher<sensor_msgs::msg::Joy>("flightgear_attitude", 5);

    RCLCPP_INFO(this->get_logger(),
                "Nó de Ponte (Joystick -> ESP32) iniciado.");
    RCLCPP_INFO(this->get_logger(),
                "Lendo parâmetros de inversão dinamicamente.");
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    auto modified_msg = *msg;

    // LER DINAMICAMENTE
    bool inv_roll = this->get_parameter("invert_roll").as_bool();
    bool inv_pitch = this->get_parameter("invert_pitch").as_bool();
    bool inv_yaw = this->get_parameter("invert_yaw").as_bool();
    bool inv_throttle = this->get_parameter("invert_throttle").as_bool();

    double sc_roll = this->get_parameter("scale_roll").as_double();
    double sc_pitch = this->get_parameter("scale_pitch").as_double();
    double sc_yaw = this->get_parameter("scale_yaw").as_double();
    double sc_throttle = this->get_parameter("scale_throttle").as_double();

    // Aplica as correções progressivamente, sem quebrar se o controle tiver
    // menos eixos
    size_t axes_count = modified_msg.axes.size();

    if (axes_count > 0)
      modified_msg.axes[0] *= (inv_roll ? -1.0 : 1.0) * sc_roll;
    if (axes_count > 1)
      modified_msg.axes[1] *= (inv_pitch ? -1.0 : 1.0) * sc_pitch;
    if (axes_count > 2)
      modified_msg.axes[2] *= (inv_yaw ? -1.0 : 1.0) * sc_yaw;
    if (axes_count > 3)
      modified_msg.axes[3] *= (inv_throttle ? -1.0 : 1.0) * sc_throttle;

    // Repassa para o ESP32
    esp32_pub_->publish(modified_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr esp32_pub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyToESP32Bridge>());
  rclcpp::shutdown();
  return 0;
}