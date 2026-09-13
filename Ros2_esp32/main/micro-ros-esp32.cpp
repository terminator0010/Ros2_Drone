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

#define RUDDER_MIN_DEG -24.0f
#define RUDDER_CENTER_DEG 0.0f
#define RUDDER_MAX_DEG 24.0f

#define RUDDER_PWM_MIN 150
#define RUDDER_PWM_CENTER 300
#define RUDDER_PWM_MAX 600

#define AILERON_MIN_DEG -20.0f
#define AILERON_CENTER_DEG 0.0f
#define AILERON_MAX_DEG 20.0f

#define AILERON_PWM_MIN 150
#define AILERON_PWM_CENTER 300
#define AILERON_PWM_MAX 600

#define ELEVATOR_MIN_DEG -28.0f
#define ELEVATOR_CENTER_DEG 0.0f
#define ELEVATOR_MAX_DEG 28.0f

#define ELEVATOR_PWM_MIN 150
#define ELEVATOR_PWM_CENTER 300
#define ELEVATOR_PWM_MAX 600

#define FLAPS_MIN_DEG -30.0f
#define FLAPS_CENTER_DEG 0.0f
#define FLAPS_MAX_DEG 30.0f

#define FLAPS_PWM_MIN 150
#define FLAPS_PWM_CENTER 300
#define FLAPS_PWM_MAX 600

#define THROTTLE_MIN_DEG -30.0f
#define THROTTLE_CENTER_DEG 0.0f
#define THROTTLE_MAX_DEG 30.0f

#define THROTTLE_PWM_MIN 150
#define THROTTLE_PWM_CENTER 300
#define THROTTLE_PWM_MAX 600

#define RUDDER_FEEDBACK_ADC_CHANNEL ADC_CHANNEL_0
#define AILERON_FEEDBACK_ADC_CHANNEL ADC_CHANNEL_1
#define ELEVATOR_FEEDBACK_ADC_CHANNEL ADC_CHANNEL_2
#define FLAPS_FEEDBACK_ADC_CHANNEL ADC_CHANNEL_3
#define THROTTLE_FEEDBACK_ADC_CHANNEL ADC_CHANNEL_4

// --------------------------------

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

void rudder_feedback_adc_init() {
  adc_oneshot_unit_init_cfg_t init_config = {};

  init_config.unit_id = ADC_UNIT_1;

  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

  adc_oneshot_chan_cfg_t config = {};

  config.bitwidth = ADC_BITWIDTH_12;
  config.atten = ADC_ATTEN_DB_12;

  ESP_ERROR_CHECK(adc_oneshot_config_channel(
      adc_handle, RUDDER_FEEDBACK_ADC_CHANNEL, &config));
}

uint16_t rudder_degrees_to_pwm(float degrees) {
  if (degrees < RUDDER_MIN_DEG)
    degrees = RUDDER_MIN_DEG;

  if (degrees > RUDDER_MAX_DEG)
    degrees = RUDDER_MAX_DEG;

  float normalized =
      (degrees - RUDDER_MIN_DEG) / (RUDDER_MAX_DEG - RUDDER_MIN_DEG);

  float pwm = RUDDER_PWM_MIN + normalized * (RUDDER_PWM_MAX - RUDDER_PWM_MIN);

  return (uint16_t)pwm;
}

uint16_t aileron_degrees_to_pwm(float degrees) {
  if (degrees < AILERON_MIN_DEG)
    degrees = AILERON_MIN_DEG;

  if (degrees > AILERON_MAX_DEG)
    degrees = AILERON_MAX_DEG;

  float normalized =
      (degrees - AILERON_MIN_DEG) / (AILERON_MAX_DEG - AILERON_MIN_DEG);

  float pwm =
      AILERON_PWM_MIN + normalized * (AILERON_PWM_MAX - AILERON_PWM_MIN);

  return (uint16_t)pwm;
}

uint16_t elevator_degrees_to_pwm(float degrees) {
  if (degrees < ELEVATOR_MIN_DEG)
    degrees = ELEVATOR_MIN_DEG;

  if (degrees > ELEVATOR_MAX_DEG)
    degrees = ELEVATOR_MAX_DEG;

  float normalized =
      (degrees - ELEVATOR_MIN_DEG) / (ELEVATOR_MAX_DEG - ELEVATOR_MIN_DEG);

  float pwm =
      ELEVATOR_PWM_MIN + normalized * (ELEVATOR_PWM_MAX - ELEVATOR_PWM_MIN);

  return (uint16_t)pwm;
}

uint16_t flaps_degrees_to_pwm(float degrees) {
  if (degrees < FLAPS_MIN_DEG)
    degrees = FLAPS_MIN_DEG;

  if (degrees > FLAPS_MAX_DEG)
    degrees = FLAPS_MAX_DEG;

  float normalized =
      (degrees - FLAPS_MIN_DEG) / (FLAPS_MAX_DEG - FLAPS_MIN_DEG);

  float pwm = FLAPS_PWM_MIN + normalized * (FLAPS_PWM_MAX - FLAPS_PWM_MIN);

  return (uint16_t)pwm;
}

uint16_t throttle_degrees_to_pwm(float degrees) {
  if (degrees < THROTTLE_MIN_DEG)
    degrees = THROTTLE_MIN_DEG;

  if (degrees > THROTTLE_MAX_DEG)
    degrees = THROTTLE_MAX_DEG;

  float normalized =
      (degrees - THROTTLE_MIN_DEG) / (THROTTLE_MAX_DEG - THROTTLE_MIN_DEG);

  float pwm =
      THROTTLE_PWM_MIN + normalized * (THROTTLE_PWM_MAX - THROTTLE_PWM_MIN);

  return (uint16_t)pwm;
}

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

void rudder_subscription_callback(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;

  last_msg_time = xTaskGetTickCount();

  if (failsafe_active)
    failsafe_active = false;

  float rudder_degrees = msg->data;

  uint16_t pwm_off = rudder_degrees_to_pwm(rudder_degrees);

  pca9685_set_pwm(0, 0, pwm_off);
}

void aileron_subscription_callback(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;

  last_msg_time = xTaskGetTickCount();

  if (failsafe_active)
    failsafe_active = false;

  float aileron_degrees = msg->data;

  uint16_t pwm_off = aileron_degrees_to_pwm(aileron_degrees);

  pca9685_set_pwm(0, 0, pwm_off);
}

float read_rudder_position_degrees() {
  int raw = 0;

  if (adc_oneshot_read(adc_handle, RUDDER_FEEDBACK_ADC_CHANNEL, &raw) !=
      ESP_OK) {
    return 0.0f;
  }

  float voltage = ((float)raw / 4095.0f) * 3.3f;

  float degrees =
      RUDDER_MIN_DEG + (voltage / 3.3f) * (RUDDER_MAX_DEG - RUDDER_MIN_DEG);

  if (degrees < RUDDER_MIN_DEG)
    degrees = RUDDER_MIN_DEG;

  if (degrees > RUDDER_MAX_DEG)
    degrees = RUDDER_MAX_DEG;

  return degrees;
}

float read_aileron_position_degrees() {
  int raw = 0;

  if (adc_oneshot_read(adc_handle, AILERON_FEEDBACK_ADC_CHANNEL, &raw) !=
      ESP_OK) {
    return 0.0f;
  }

  float voltage = ((float)raw / 4095.0f) * 3.3f;

  float degrees =
      AILERON_MIN_DEG + (voltage / 3.3f) * (AILERON_MAX_DEG - AILERON_MIN_DEG);

  if (degrees < AILERON_MIN_DEG)
    degrees = AILERON_MIN_DEG;

  if (degrees > AILERON_MAX_DEG)
    degrees = AILERON_MAX_DEG;

  return degrees;
}

float read_elevator_position_degrees() {
  int raw = 0;

  if (adc_oneshot_read(adc_handle, ELEVATOR_FEEDBACK_ADC_CHANNEL, &raw) !=
      ESP_OK) {
    return 0.0f;
  }

  float voltage = ((float)raw / 4095.0f) * 3.3f;

  float degrees = ELEVATOR_MIN_DEG +
                  (voltage / 3.3f) * (ELEVATOR_MAX_DEG - ELEVATOR_MIN_DEG);

  if (degrees < ELEVATOR_MIN_DEG)
    degrees = ELEVATOR_MIN_DEG;

  if (degrees > ELEVATOR_MAX_DEG)
    degrees = ELEVATOR_MAX_DEG;

  return degrees;
}

float read_flaps_position_degrees() {
  int raw = 0;

  if (adc_oneshot_read(adc_handle, FLAPS_FEEDBACK_ADC_CHANNEL, &raw) !=
      ESP_OK) {
    return 0.0f;
  }

  float voltage = ((float)raw / 4095.0f) * 3.3f;

  float degrees =
      FLAPS_MIN_DEG + (voltage / 3.3f) * (FLAPS_MAX_DEG - FLAPS_MIN_DEG);

  if (degrees < FLAPS_MIN_DEG)
    degrees = FLAPS_MIN_DEG;

  if (degrees > FLAPS_MAX_DEG)
    degrees = FLAPS_MAX_DEG;

  return degrees;
}

float read_throttle_position_degrees() {
  int raw = 0;

  if (adc_oneshot_read(adc_handle, THROTTLE_FEEDBACK_ADC_CHANNEL, &raw) !=
      ESP_OK) {
    return 0.0f;
  }

  float voltage = ((float)raw / 4095.0f) * 3.3f;

  float degrees = THROTTLE_MIN_DEG +
                  (voltage / 3.3f) * (THROTTLE_MAX_DEG - THROTTLE_MIN_DEG);

  if (degrees < THROTTLE_MIN_DEG)
    degrees = THROTTLE_MIN_DEG;

  if (degrees > THROTTLE_MAX_DEG)
    degrees = THROTTLE_MAX_DEG;

  return degrees;
}

void rudder_publisher_servo_callback(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;

  // Atualiza o Watchdog: Um sinal vital válido foi recebido
  last_msg_time = xTaskGetTickCount();
  if (failsafe_active) {
    // ESP_LOGI(TAG, "SINAL RECUPERADO: Controle nominal retomado.");
    failsafe_active = false;
  }

  // O valor recebido varia de -1.0 a 1.0.
  float rudder_val = msg->data;

  // Limitação de segurança (clamp)
  if (rudder_val < -1.0f)
    rudder_val = -1.0f;
  if (rudder_val > 1.0f)
    rudder_val = 1.0f;

  // Conversão para o PCA9685 (resolução de 12 bits, 0-4095) a 50Hz.
  // 1ms (0 graus) ~= 204. 2ms (180 graus) ~= 409. Centro (90 graus) = 307.
  // Para abranger um arco ligeiramente maior com segurança, usaremos o range de
  // 150 a 600 (centro 00).
  uint16_t pwm_off = 300 + (uint16_t)(rudder_val * 300);

  // Envia o comando I2C para o PCA9685 no Canal 0
  pca9685_set_pwm(0, 0, pwm_off);
}

void flightgear_subscription_callback(const void *msgin) {
  const sensor_msgs__msg__Joy *incoming_msg =
      (const sensor_msgs__msg__Joy *)msgin;

  // Copia os eixos (limitado ao tamanho máximo)
  size_t axes_to_copy = incoming_msg->axes.size;
  if (axes_to_copy > MAX_AXES)
    axes_to_copy = MAX_AXES;

  echo_msg.axes.size = axes_to_copy;
  for (size_t i = 0; i < axes_to_copy; i++) {
    echo_msg.axes.data[i] = incoming_msg->axes.data[i];
  }

  // Copia os botões (limitado ao tamanho máximo)
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
    // ESP_LOGI(TAG, "Tentando conectar ao micro-ROS agent...");

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

    echo_publisher = rcl_get_zero_initialized_publisher();
    if (rclc_publisher_init_default(
            &echo_publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
            "flightgear_attitude_echo") != RCL_RET_OK) {
      RCSOFTCHECK(rcl_node_fini(&node));
      RCSOFTCHECK(rclc_support_fini(&support));
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }

    subscriber = rcl_get_zero_initialized_subscription();
    if (rclc_subscription_init_default(
            &subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
            "flightgear_attitude") != RCL_RET_OK) {
      RCSOFTCHECK(rcl_publisher_fini(&echo_publisher, &node));
      RCSOFTCHECK(rcl_node_fini(&node));
      RCSOFTCHECK(rclc_support_fini(&support));
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }

    rudder_publisher_servo = rcl_get_zero_initialized_publisher();
    if (rclc_publisher_init_default(
            &rudder_publisher_servo, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "rudder_pos_servo_echo") != RCL_RET_OK) {
      RCSOFTCHECK(rcl_node_fini(&node));
      RCSOFTCHECK(rclc_support_fini(&support));
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }

    rudder_subscriber = rcl_get_zero_initialized_subscription();
    if (rclc_subscription_init_default(
            &rudder_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "rudder_pos") != RCL_RET_OK) {
      RCSOFTCHECK(rcl_publisher_fini(&rudder_publisher_servo, &node));
      RCSOFTCHECK(rcl_node_fini(&node));
      RCSOFTCHECK(rclc_support_fini(&support));
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }

    aileron_subscriber = rcl_get_zero_initialized_subscription();
    if (rclc_subscription_init_default(
            &aileron_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "aileron_pos") != RCL_RET_OK) {
      RCSOFTCHECK(rcl_publisher_fini(&aileron_publisher_servo, &node));
      RCSOFTCHECK(rcl_node_fini(&node));
      RCSOFTCHECK(rclc_support_fini(&support));
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }

    elevator_subscriber = rcl_get_zero_initialized_subscription();
    if (rclc_subscription_init_default(
            &elevator_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "elevator_pos") != RCL_RET_OK) {
      RCSOFTCHECK(rcl_publisher_fini(&elevator_publisher_servo, &node));
      RCSOFTCHECK(rcl_node_fini(&node));
      RCSOFTCHECK(rclc_support_fini(&support));
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }

    flaps_subscriber = rcl_get_zero_initialized_subscription();
    if (rclc_subscription_init_default(
            &flaps_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "flaps_pos") != RCL_RET_OK) {
      RCSOFTCHECK(rcl_publisher_fini(&flaps_publisher_servo, &node));
      RCSOFTCHECK(rcl_node_fini(&node));
      RCSOFTCHECK(rclc_support_fini(&support));
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }

    throttle_subscriber = rcl_get_zero_initialized_subscription();
    if (rclc_subscription_init_default(
            &throttle_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "throttle_pos") != RCL_RET_OK) {
      RCSOFTCHECK(rcl_publisher_fini(&throttle_publisher_servo, &node));
      RCSOFTCHECK(rcl_node_fini(&node));
      RCSOFTCHECK(rclc_support_fini(&support));
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }

    executor = rclc_executor_get_zero_initialized_executor();
    if (rclc_executor_init(&executor, &support.context, 2, &allocator) !=
        RCL_RET_OK) {
      RCSOFTCHECK(rcl_subscription_fini(&rudder_subscriber, &node));
      RCSOFTCHECK(rcl_subscription_fini(&subscriber, &node));
      RCSOFTCHECK(rcl_publisher_fini(&echo_publisher, &node));
      RCSOFTCHECK(rcl_subscription_fini(&aileron_subscriber, &node));
      RCSOFTCHECK(rcl_subscription_fini(&elevator_subscriber, &node));
      RCSOFTCHECK(rcl_subscription_fini(&flaps_subscriber, &node));
      RCSOFTCHECK(rcl_subscription_fini(&throttle_subscriber, &node));
      RCSOFTCHECK(rcl_node_fini(&node));
      RCSOFTCHECK(rclc_support_fini(&support));
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }

    if (rclc_executor_add_subscription(&executor, &subscriber, &received_msg,
                                       &flightgear_subscription_callback,
                                       ON_NEW_DATA) != RCL_RET_OK) {
      RCSOFTCHECK(rclc_executor_fini(&executor));
      RCSOFTCHECK(rcl_subscription_fini(&rudder_subscriber, &node));
      RCSOFTCHECK(rcl_subscription_fini(&aileron_subscriber, &node));
      RCSOFTCHECK(rcl_subscription_fini(&elevator_subscriber, &node));
      RCSOFTCHECK(rcl_subscription_fini(&flaps_subscriber, &node));
      RCSOFTCHECK(rcl_subscription_fini(&throttle_subscriber, &node));
      RCSOFTCHECK(rcl_subscription_fini(&subscriber, &node));
      RCSOFTCHECK(rcl_publisher_fini(&echo_publisher, &node));
      RCSOFTCHECK(rcl_node_fini(&node));
      RCSOFTCHECK(rclc_support_fini(&support));
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }

    if (rclc_executor_add_subscription(
            &executor, &rudder_subscriber, &rudder_msg,
            &rudder_publisher_servo_callback, ON_NEW_DATA) != RCL_RET_OK) {
      RCSOFTCHECK(rclc_executor_fini(&executor));
      RCSOFTCHECK(rcl_subscription_fini(&rudder_subscriber, &node));
      RCSOFTCHECK(rcl_subscription_fini(&aileron_subscriber, &node));
      RCSOFTCHECK(rcl_subscription_fini(&elevator_subscriber, &node));
      RCSOFTCHECK(rcl_subscription_fini(&flaps_subscriber, &node));
      RCSOFTCHECK(rcl_subscription_fini(&throttle_subscriber, &node));
      RCSOFTCHECK(rcl_subscription_fini(&subscriber, &node));
      RCSOFTCHECK(rcl_publisher_fini(&echo_publisher, &node));
      RCSOFTCHECK(rcl_node_fini(&node));
      RCSOFTCHECK(rclc_support_fini(&support));
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }

    // ESP_LOGI(TAG, "Conectado! Aguardando comandos...");

    // Inicia a contagem do Watchdog no momento da conexão bem-sucedida
    last_msg_time = xTaskGetTickCount();

    while (1) {
      rcl_ret_t rc = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

      rudder_msg.data = read_rudder_position_degrees();
      aileron_msg.data = read_aileron_position_degrees();
      elevator_msg.data = read_elevator_position_degrees();
      flaps_msg.data = read_flaps_position_degrees();
      throttle_msg.data = read_throttle_position_degrees();

      RCSOFTCHECK(rcl_publish(&rudder_publisher_servo, &rudder_msg, NULL));
      RCSOFTCHECK(rcl_publish(&aileron_publisher_servo, &aileron_msg, NULL));
      RCSOFTCHECK(rcl_publish(&elevator_publisher_servo, &elevator_msg, NULL));
      RCSOFTCHECK(rcl_publish(&flaps_publisher_servo, &flaps_msg, NULL));
      RCSOFTCHECK(rcl_publish(&throttle_publisher_servo, &throttle_msg, NULL));

      if (rc != RCL_RET_OK && rc != RCL_RET_TIMEOUT) {
        // ESP_LOGE(TAG, "Conexão perdida (Erro Executor). Reiniciando nós...");
        break;
      }

      // --- LÓGICA DO WATCHDOG ---
      TickType_t current_time = xTaskGetTickCount();
      uint32_t time_since_last_msg =
          (current_time - last_msg_time) * portTICK_PERIOD_MS;

      if (time_since_last_msg > FAILSAFE_TIMEOUT_MS) {
        if (!failsafe_active) {
          /* ESP_LOGE(TAG,
                   "ALERTA CRÍTICO: Timeout da rede (>%d ms)! Acionando Modo "
                   "Failsafe.",
                   FAILSAFE_TIMEOUT_MS); */

          // Centraliza o leme (posição segura 375 = ~90 graus)
          pca9685_set_pwm(0, 0, 300);

          failsafe_active = true;
        }
      }
      // --------------------------

      vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    RCSOFTCHECK(rcl_publisher_fini(&echo_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&rudder_publisher_servo, &node));
    RCSOFTCHECK(rcl_subscription_fini(&rudder_subscriber, &node));
    RCSOFTCHECK(rcl_subscription_fini(&aileron_subscriber, &node));
    RCSOFTCHECK(rcl_subscription_fini(&elevator_subscriber, &node));
    RCSOFTCHECK(rcl_subscription_fini(&flaps_subscriber, &node));
    RCSOFTCHECK(rcl_subscription_fini(&throttle_subscriber, &node));
    RCSOFTCHECK(rcl_subscription_fini(&subscriber, &node));
    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rclc_support_fini(&support));

    // Failsafe de desconexão física: ativado imediatamente se o loop do
    // executor for quebrado
    if (!failsafe_active) {
      /* ESP_LOGE(TAG, "ALERTA CRÍTICO: Conexão USB/Serial interrompida! "
                    "Acionando Failsafe."); */
      pca9685_set_pwm(0, 0, 300);
      failsafe_active = true;
    }

    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

// O extern "C" é obrigatório aqui para o ESP-IDF encontrar a função

extern "C" void app_main(void) {
  static uart_port_t uart_port = UART_NUM_0;

  // Configura o transporte para usar a porta serial nativa do ESP32
  rmw_uros_set_custom_transport(true, (void *)&uart_port, esp32_serial_open,
                                esp32_serial_close, esp32_serial_write,
                                esp32_serial_read);

  // Inicialização do barramento I2C e configuração do Driver PCA9685
  ESP_ERROR_CHECK(i2c_master_init());
  pca9685_init();

  // Inicia com o servo fisicamente travado no centro até a primeira mensagem de
  // rede
  pca9685_set_pwm(0, 0, 300);

  xTaskCreatePinnedToCore(micro_ros_task, "uros_task",
                          MICRO_ROS_TASK_STACK_SIZE, NULL,
                          MICRO_ROS_TASK_PRIORITY, NULL, 1);
}