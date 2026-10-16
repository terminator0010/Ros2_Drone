#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c.h" // Incluído para controle do PCA9685
#include "driver/uart.h"
#include "esp_log.h"

// Garante que o compilador C++ não modifique o nome destas funções em C
extern "C" {
#include "esp32_serial_transport.h"
#include "uros_network_interfaces.h"
}

#include "rcl/rcl.h"
#include "rclc/executor.h"
#include "rclc/rclc.h"

#include "rmw_microros/rmw_microros.h"
#include "sensor_msgs/msg/joy.h"
#include <std_msgs/msg/float32.h>

#include "esp_adc/adc_oneshot.h"

#define MICRO_ROS_TASK_STACK_SIZE 8192
#define MICRO_ROS_TASK_PRIORITY 5

// Limites para evitar vazamento de memória (Ghost axes no Linux)
#define MAX_AXES 16
#define MAX_BUTTONS 40
#define MAX_STRING_LEN 30

// --- CONFIGURAÇÕES DO FAILSAFE E WATCHDOG ---
#define FAILSAFE_TIMEOUT_MS 500 // 0.5 segundos sem comandos = Emergência
static TickType_t last_msg_time = 0;
static bool failsafe_active =
    true; // Inicia bloqueado (Emergência) por segurança
static const char *TAG = "HIL_CONTROLLER";

// --- DEFINIÇÕES I2C (PCA9685) ---
#define I2C_MASTER_SCL_IO 2
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define PCA9685_ADDR 0x40

#define PCA9685_MODE1 0x00
#define PCA9685_PRESCALE 0xFE
#define LED0_ON_L 0x06
#define LED0_ON_H 0x07
#define LED0_OFF_L 0x08
#define LED0_OFF_H 0x09

#define RUDDER_FEEDBACK_ADC_CHANNEL ADC_CHANNEL_0
#define AILERON_FEEDBACK_ADC_CHANNEL ADC_CHANNEL_1
#define ELEVATOR_FEEDBACK_ADC_CHANNEL ADC_CHANNEL_3
#define FLAPS_FEEDBACK_ADC_CHANNEL ADC_CHANNEL_5
#define THROTTLE_FEEDBACK_ADC_CHANNEL ADC_CHANNEL_7

// --------------------------------
// ESTRUTURA CENTRALIZADA DOS SERVOS
// --------------------------------
typedef struct {
  uint8_t canal_pca9685;
  float angulo_min;
  float angulo_max;
  float angulo_neutro;
  uint16_t pwm_min;
  uint16_t pwm_max;
  int8_t invertido;
} ServoConfig;

#define NUM_SERVOS 8

ServoConfig servos[NUM_SERVOS] = {
    {0, -24.0f, 24.0f, 0.0f, 150, 600, 1}, // 0: Rudder
    {1, -20.0f, 20.0f, 0.0f, 150, 600, 1}, // 1: Aileron 1
    {2, -20.0f, 20.0f, 0.0f, 150, 600,
     -1}, // 2: Aileron 2 (invertido mecanicamente/logicamente)
    {3, -28.0f, 28.0f, 0.0f, 150, 600, 1},  // 3: Elevator 1
    {4, -28.0f, 28.0f, 0.0f, 150, 600, -1}, // 4: Elevator 2 (invertido)
    {5, -30.0f, 30.0f, 0.0f, 150, 600, 1},  // 5: Flap 1
    {6, -30.0f, 30.0f, 0.0f, 150, 600, -1}, // 6: Flap 2 (invertido)
    {7, -30.0f, 30.0f, 0.0f, 150, 600, 1} // 7: Throttle (Reservado/Inalterado)
};

#define RCSOFTCHECK(fn)                                                        \
  do {                                                                         \
    rcl_ret_t temp_rc = (fn);                                                  \
    (void)temp_rc;                                                             \
  } while (0)

static rcl_subscription_t subscriber;
static rcl_subscription_t rudder_subscriber;
static rcl_subscription_t aileron_subscriber;
static rcl_subscription_t elevator_subscriber;
static rcl_subscription_t flaps_subscriber;
static rcl_subscription_t throttle_subscriber;

static rcl_publisher_t rudder_publisher_servo;
static rcl_publisher_t aileron_publisher_servo;
static rcl_publisher_t elevator_publisher_servo;
static rcl_publisher_t flaps_publisher_servo;
static rcl_publisher_t throttle_publisher_servo;
static rcl_publisher_t echo_publisher;

static sensor_msgs__msg__Joy received_msg;
static sensor_msgs__msg__Joy echo_msg;

static std_msgs__msg__Float32 rudder_msg;
static std_msgs__msg__Float32 aileron_msg;
static std_msgs__msg__Float32 elevator_msg;
static std_msgs__msg__Float32 flaps_msg;
static std_msgs__msg__Float32 throttle_msg;

static adc_oneshot_unit_handle_t adc_handle;

// --- FUNÇÕES DE CONTROLE DO PCA9685 (I2C) ---

esp_err_t i2c_master_init() {
  i2c_config_t conf = {};
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = I2C_MASTER_SDA_IO;
  conf.scl_io_num = I2C_MASTER_SCL_IO;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

  i2c_param_config(I2C_MASTER_NUM, &conf);
  return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void pca9685_write_byte(uint8_t reg, uint8_t data) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (PCA9685_ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_write_byte(cmd, data, true);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
}

void pca9685_init() {
  pca9685_write_byte(PCA9685_MODE1, 0x11); // Wake up
  vTaskDelay(pdMS_TO_TICKS(10));
  pca9685_write_byte(PCA9685_PRESCALE, 0x79); // 50Hz para servos
  pca9685_write_byte(PCA9685_MODE1, 0x01);    // Restart
  vTaskDelay(pdMS_TO_TICKS(10));
}

void pca9685_set_pwm(uint8_t canal, uint16_t on, uint16_t off) {
  uint8_t offset = 4 * canal;
  pca9685_write_byte(LED0_ON_L + offset, on & 0xFF);
  pca9685_write_byte(LED0_ON_H + offset, on >> 8);
  pca9685_write_byte(LED0_OFF_L + offset, off & 0xFF);
  pca9685_write_byte(LED0_OFF_H + offset, off >> 8);
}

// --------------------------------------------
// CONTROLE DE SERVOS UNIFICADO
// --------------------------------------------

void set_servo_angle(uint8_t index, float degrees) {
  if (index >= NUM_SERVOS)
    return;
  ServoConfig *cfg = &servos[index];

  // Limitação de segurança
  if (degrees < cfg->angulo_min)
    degrees = cfg->angulo_min;
  if (degrees > cfg->angulo_max)
    degrees = cfg->angulo_max;

  // Conversão para proporção (0.0 a 1.0)
  float normalized =
      (degrees - cfg->angulo_min) / (cfg->angulo_max - cfg->angulo_min);

  // Inversão se necessário
  if (cfg->invertido < 0) {
    normalized = 1.0f - normalized;
  }

  // Calcula o PWM final
  uint16_t pwm = cfg->pwm_min + normalized * (cfg->pwm_max - cfg->pwm_min);
  pca9685_set_pwm(cfg->canal_pca9685, 0, pwm);
}

// Inicializa ADC genérico para feedback
void adc_feedback_init() {
  adc_oneshot_unit_init_cfg_t init_config = {};
  init_config.unit_id = ADC_UNIT_1;
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

  adc_oneshot_chan_cfg_t config = {};
  config.bitwidth = ADC_BITWIDTH_12;
  config.atten = ADC_ATTEN_DB_12;

  ESP_ERROR_CHECK(adc_oneshot_config_channel(
      adc_handle, RUDDER_FEEDBACK_ADC_CHANNEL, &config));
  ESP_ERROR_CHECK(adc_oneshot_config_channel(
      adc_handle, AILERON_FEEDBACK_ADC_CHANNEL, &config));
  ESP_ERROR_CHECK(adc_oneshot_config_channel(
      adc_handle, ELEVATOR_FEEDBACK_ADC_CHANNEL, &config));
  ESP_ERROR_CHECK(adc_oneshot_config_channel(
      adc_handle, FLAPS_FEEDBACK_ADC_CHANNEL, &config));
  ESP_ERROR_CHECK(adc_oneshot_config_channel(
      adc_handle, THROTTLE_FEEDBACK_ADC_CHANNEL, &config));
}

float read_adc_to_degrees(adc_channel_t channel, float min_deg, float max_deg) {
  int raw = 0;
  if (adc_oneshot_read(adc_handle, channel, &raw) != ESP_OK) {
    return 0.0f;
  }
  float voltage = ((float)raw / 4095.0f) * 3.3f;
  float degrees = min_deg + (voltage / 3.3f) * (max_deg - min_deg);
  if (degrees < min_deg)
    degrees = min_deg;
  if (degrees > max_deg)
    degrees = max_deg;
  return degrees;
}

// --------------------------------------------
// CALLBACKS ROS
// --------------------------------------------

void rudder_subscription_callback(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  last_msg_time = xTaskGetTickCount();
  if (failsafe_active)
    failsafe_active = false;

  set_servo_angle(0, msg->data); // Rudder -> canal 0
}

void aileron_subscription_callback(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  last_msg_time = xTaskGetTickCount();
  if (failsafe_active)
    failsafe_active = false;

  set_servo_angle(1, msg->data); // Aileron 1 -> canal 1
  set_servo_angle(2, msg->data); // Aileron 2 -> canal 2
}

void elevator_subscription_callback(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  last_msg_time = xTaskGetTickCount();
  if (failsafe_active)
    failsafe_active = false;

  set_servo_angle(3, msg->data); // Elevator 1 -> canal 3
  set_servo_angle(4, msg->data); // Elevator 2 -> canal 4
}

void flaps_subscription_callback(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  last_msg_time = xTaskGetTickCount();
  if (failsafe_active)
    failsafe_active = false;

  set_servo_angle(5, msg->data); // Flap 1 -> canal 5
  set_servo_angle(6, msg->data); // Flap 2 -> canal 6
}

void throttle_subscription_callback(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;

  // Atualiza apenas o watchdog para evitar perda de conexão
  last_msg_time = xTaskGetTickCount();
  if (failsafe_active)
    failsafe_active = false;

  // O canal de throttle permanece sem alterações funcionais,
  // reservado para futuras lógicas de controle do motor DC.
  // set_servo_angle(7, msg->data);
}

void flightgear_subscription_callback(const void *msgin) {
  const sensor_msgs__msg__Joy *incoming_msg =
      (const sensor_msgs__msg__Joy *)msgin;

  size_t axes_to_copy = incoming_msg->axes.size;
  if (axes_to_copy > MAX_AXES)
    axes_to_copy = MAX_AXES;

  echo_msg.axes.size = axes_to_copy;
  for (size_t i = 0; i < axes_to_copy; i++) {
    echo_msg.axes.data[i] = incoming_msg->axes.data[i];
  }

  size_t buttons_to_copy = incoming_msg->buttons.size;
  if (buttons_to_copy > MAX_BUTTONS)
    buttons_to_copy = MAX_BUTTONS;

  echo_msg.buttons.size = buttons_to_copy;
  for (size_t i = 0; i < buttons_to_copy; i++) {
    echo_msg.buttons.data[i] = incoming_msg->buttons.data[i];
  }

  RCSOFTCHECK(rcl_publish(&echo_publisher, &echo_msg, NULL));
}

void micro_ros_task(void *arg) {
  (void)arg;

  // --- ALOCAÇÃO DE MEMÓRIA DINÂMICA SEGURA ---
  received_msg.axes.capacity = MAX_AXES;
  received_msg.axes.data =
      (float *)malloc(received_msg.axes.capacity * sizeof(float));
  received_msg.axes.size = 0;

  echo_msg.axes.capacity = MAX_AXES;
  echo_msg.axes.data = (float *)malloc(echo_msg.axes.capacity * sizeof(float));
  echo_msg.axes.size = 0;

  received_msg.buttons.capacity = MAX_BUTTONS;
  received_msg.buttons.data =
      (int32_t *)malloc(received_msg.buttons.capacity * sizeof(int32_t));
  received_msg.buttons.size = 0;

  echo_msg.buttons.capacity = MAX_BUTTONS;
  echo_msg.buttons.data =
      (int32_t *)malloc(echo_msg.buttons.capacity * sizeof(int32_t));
  echo_msg.buttons.size = 0;

  received_msg.header.frame_id.capacity = MAX_STRING_LEN;
  received_msg.header.frame_id.data =
      (char *)malloc(MAX_STRING_LEN * sizeof(char));
  received_msg.header.frame_id.size = 0;

  echo_msg.header.frame_id.capacity = MAX_STRING_LEN;
  echo_msg.header.frame_id.data = (char *)malloc(MAX_STRING_LEN * sizeof(char));
  echo_msg.header.frame_id.size = 0;
  // ---------------------------------------------

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;
  rcl_node_t node;
  rclc_executor_t executor;

  while (1) {
    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
      vTaskDelay(2000 / portTICK_PERIOD_MS);
      continue;
    }

    node = rcl_get_zero_initialized_node();
    if (rclc_node_init_default(&node, "esp32_control_node", "", &support) !=
        RCL_RET_OK) {
      RCSOFTCHECK(rclc_support_fini(&support));
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }

    // Inicializa Publishers (Echo)
    echo_publisher = rcl_get_zero_initialized_publisher();
    if (rclc_publisher_init_default(
            &echo_publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
            "flightgear_attitude_echo") != RCL_RET_OK)
      goto cleanup_node;

    rudder_publisher_servo = rcl_get_zero_initialized_publisher();
    if (rclc_publisher_init_default(
            &rudder_publisher_servo, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "rudder_pos_servo_echo") != RCL_RET_OK)
      goto cleanup_node;

    aileron_publisher_servo = rcl_get_zero_initialized_publisher();
    if (rclc_publisher_init_default(
            &aileron_publisher_servo, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "aileron_pos_servo_echo") != RCL_RET_OK)
      goto cleanup_node;

    elevator_publisher_servo = rcl_get_zero_initialized_publisher();
    if (rclc_publisher_init_default(
            &elevator_publisher_servo, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "elevator_pos_servo_echo") != RCL_RET_OK)
      goto cleanup_node;

    flaps_publisher_servo = rcl_get_zero_initialized_publisher();
    if (rclc_publisher_init_default(
            &flaps_publisher_servo, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "flaps_pos_servo_echo") != RCL_RET_OK)
      goto cleanup_node;

    throttle_publisher_servo = rcl_get_zero_initialized_publisher();
    if (rclc_publisher_init_default(
            &throttle_publisher_servo, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "throttle_pos_servo_echo") != RCL_RET_OK)
      goto cleanup_node;

    // Inicializa Subscribers
    subscriber = rcl_get_zero_initialized_subscription();
    if (rclc_subscription_init_default(
            &subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
            "flightgear_attitude") != RCL_RET_OK)
      goto cleanup_node;

    rudder_subscriber = rcl_get_zero_initialized_subscription();
    if (rclc_subscription_init_default(
            &rudder_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "rudder_pos") != RCL_RET_OK)
      goto cleanup_node;

    aileron_subscriber = rcl_get_zero_initialized_subscription();
    if (rclc_subscription_init_default(
            &aileron_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "aileron_pos") != RCL_RET_OK)
      goto cleanup_node;

    elevator_subscriber = rcl_get_zero_initialized_subscription();
    if (rclc_subscription_init_default(
            &elevator_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "elevator_pos") != RCL_RET_OK)
      goto cleanup_node;

    flaps_subscriber = rcl_get_zero_initialized_subscription();
    if (rclc_subscription_init_default(
            &flaps_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "flap_pos") != RCL_RET_OK)
      goto cleanup_node;

    throttle_subscriber = rcl_get_zero_initialized_subscription();
    if (rclc_subscription_init_default(
            &throttle_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "throttle_pos") != RCL_RET_OK)
      goto cleanup_node;

    // Executor com 6 handles (1 joy + 5 floats)
    executor = rclc_executor_get_zero_initialized_executor();
    if (rclc_executor_init(&executor, &support.context, 6, &allocator) !=
        RCL_RET_OK)
      goto cleanup_node;

    if (rclc_executor_add_subscription(&executor, &subscriber, &received_msg,
                                       &flightgear_subscription_callback,
                                       ON_NEW_DATA) != RCL_RET_OK)
      goto cleanup_executor;
    if (rclc_executor_add_subscription(
            &executor, &rudder_subscriber, &rudder_msg,
            &rudder_subscription_callback, ON_NEW_DATA) != RCL_RET_OK)
      goto cleanup_executor;
    if (rclc_executor_add_subscription(
            &executor, &aileron_subscriber, &aileron_msg,
            &aileron_subscription_callback, ON_NEW_DATA) != RCL_RET_OK)
      goto cleanup_executor;
    if (rclc_executor_add_subscription(
            &executor, &elevator_subscriber, &elevator_msg,
            &elevator_subscription_callback, ON_NEW_DATA) != RCL_RET_OK)
      goto cleanup_executor;
    if (rclc_executor_add_subscription(&executor, &flaps_subscriber, &flaps_msg,
                                       &flaps_subscription_callback,
                                       ON_NEW_DATA) != RCL_RET_OK)
      goto cleanup_executor;
    if (rclc_executor_add_subscription(
            &executor, &throttle_subscriber, &throttle_msg,
            &throttle_subscription_callback, ON_NEW_DATA) != RCL_RET_OK)
      goto cleanup_executor;

    last_msg_time = xTaskGetTickCount();

    while (1) {
      rcl_ret_t rc = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

      rudder_msg.data =
          read_adc_to_degrees(RUDDER_FEEDBACK_ADC_CHANNEL, servos[0].angulo_min,
                              servos[0].angulo_max);
      aileron_msg.data =
          read_adc_to_degrees(AILERON_FEEDBACK_ADC_CHANNEL,
                              servos[1].angulo_min, servos[1].angulo_max);
      elevator_msg.data =
          read_adc_to_degrees(ELEVATOR_FEEDBACK_ADC_CHANNEL,
                              servos[3].angulo_min, servos[3].angulo_max);
      flaps_msg.data =
          read_adc_to_degrees(FLAPS_FEEDBACK_ADC_CHANNEL, servos[5].angulo_min,
                              servos[5].angulo_max);
      throttle_msg.data =
          read_adc_to_degrees(THROTTLE_FEEDBACK_ADC_CHANNEL,
                              servos[7].angulo_min, servos[7].angulo_max);

      RCSOFTCHECK(rcl_publish(&rudder_publisher_servo, &rudder_msg, NULL));
      RCSOFTCHECK(rcl_publish(&aileron_publisher_servo, &aileron_msg, NULL));
      RCSOFTCHECK(rcl_publish(&elevator_publisher_servo, &elevator_msg, NULL));
      RCSOFTCHECK(rcl_publish(&flaps_publisher_servo, &flaps_msg, NULL));
      RCSOFTCHECK(rcl_publish(&throttle_publisher_servo, &throttle_msg, NULL));

      if (rc != RCL_RET_OK && rc != RCL_RET_TIMEOUT) {
        break;
      }

      // --- LÓGICA DO WATCHDOG ---
      TickType_t current_time = xTaskGetTickCount();
      uint32_t time_since_last_msg =
          (current_time - last_msg_time) * portTICK_PERIOD_MS;

      if (time_since_last_msg > FAILSAFE_TIMEOUT_MS) {
        if (!failsafe_active) {
          // Centraliza apenas os servos de controle de voo (0 a 6)
          for (int i = 0; i <= 6; i++) {
            set_servo_angle(i, servos[i].angulo_neutro);
          }
          failsafe_active = true;
        }
      }
      // --------------------------

      vTaskDelay(10 / portTICK_PERIOD_MS);
    }

  cleanup_executor:
    RCSOFTCHECK(rclc_executor_fini(&executor));
  cleanup_node:
    RCSOFTCHECK(rcl_publisher_fini(&echo_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&rudder_publisher_servo, &node));
    RCSOFTCHECK(rcl_publisher_fini(&aileron_publisher_servo, &node));
    RCSOFTCHECK(rcl_publisher_fini(&elevator_publisher_servo, &node));
    RCSOFTCHECK(rcl_publisher_fini(&flaps_publisher_servo, &node));
    RCSOFTCHECK(rcl_publisher_fini(&throttle_publisher_servo, &node));
    RCSOFTCHECK(rcl_subscription_fini(&rudder_subscriber, &node));
    RCSOFTCHECK(rcl_subscription_fini(&aileron_subscriber, &node));
    RCSOFTCHECK(rcl_subscription_fini(&elevator_subscriber, &node));
    RCSOFTCHECK(rcl_subscription_fini(&flaps_subscriber, &node));
    RCSOFTCHECK(rcl_subscription_fini(&throttle_subscriber, &node));
    RCSOFTCHECK(rcl_subscription_fini(&subscriber, &node));
    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rclc_support_fini(&support));

    // Failsafe de desconexão física
    if (!failsafe_active) {
      for (int i = 0; i <= 6; i++) {
        set_servo_angle(i, servos[i].angulo_neutro);
      }
      failsafe_active = true;
    }

    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

extern "C" void app_main(void) {
  static uart_port_t uart_port = UART_NUM_0;

  rmw_uros_set_custom_transport(true, (void *)&uart_port, esp32_serial_open,
                                esp32_serial_close, esp32_serial_write,
                                esp32_serial_read);

  ESP_ERROR_CHECK(i2c_master_init());
  pca9685_init();
  adc_feedback_init();

  // Inicia com os servos 0 a 6 fisicamente travados no centro
  for (int i = 0; i <= 6; i++) {
    set_servo_angle(i, servos[i].angulo_neutro);
  }

  // set_servo_angle(0, 0);    // Rudder
  // set_servo_angle(1, 15);   // Aileron 1
  // set_servo_angle(2, 15);   // Aileron 2

  xTaskCreatePinnedToCore(micro_ros_task, "uros_task",
                          MICRO_ROS_TASK_STACK_SIZE, NULL,
                          MICRO_ROS_TASK_PRIORITY, NULL, 1);
}