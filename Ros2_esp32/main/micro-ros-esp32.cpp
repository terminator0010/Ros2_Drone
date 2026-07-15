#include <stdio.h>
#include <stdlib.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "esp32_serial_transport.h"

#include "rcl/rcl.h"
#include "rclc/executor.h"
#include "rclc/rclc.h"

#include "rmw_microros/rmw_microros.h"
#include "sensor_msgs/msg/joy.h"

#define MICRO_ROS_TASK_STACK_SIZE 8192
#define MICRO_ROS_TASK_PRIORITY 5

// Aumentamos os limites para cobrir qualquer eixo fantasma que o Linux mapeie
#define MAX_AXES 16
#define MAX_BUTTONS 40
#define MAX_STRING_LEN 30

#define RCSOFTCHECK(fn)                                                        \
  do {                                                                         \
    rcl_ret_t temp_rc = (fn);                                                  \
    (void)temp_rc;                                                             \
  } while (0)

static rcl_subscription_t subscriber;
static rcl_publisher_t echo_publisher;

static sensor_msgs__msg__Joy received_msg;
static sensor_msgs__msg__Joy echo_msg;

void flightgear_subscription_callback(const void *msgin) {
  const sensor_msgs__msg__Joy *incoming_msg =
      (const sensor_msgs__msg__Joy *)msgin;

  // Copia os eixos
  echo_msg.axes.size = incoming_msg->axes.size;
  for (size_t i = 0; i < incoming_msg->axes.size; i++) {
    echo_msg.axes.data[i] = incoming_msg->axes.data[i];
  }

  // Copia os botões
  echo_msg.buttons.size = incoming_msg->buttons.size;
  for (size_t i = 0; i < incoming_msg->buttons.size; i++) {
    echo_msg.buttons.data[i] = incoming_msg->buttons.data[i];
  }

  RCSOFTCHECK(rcl_publish(&echo_publisher, &echo_msg, NULL));
}

void micro_ros_task(void *arg) {
  (void)arg;

  // --- ALOCAÇÃO DE MEMÓRIA DINÂMICA SEGURA ---
  // Eixos
  received_msg.axes.capacity = MAX_AXES;
  received_msg.axes.data =
      (float *)malloc(received_msg.axes.capacity * sizeof(float));
  received_msg.axes.size = 0;

  echo_msg.axes.capacity = MAX_AXES;
  echo_msg.axes.data = (float *)malloc(echo_msg.axes.capacity * sizeof(float));
  echo_msg.axes.size = 0;

  // Botões
  received_msg.buttons.capacity = MAX_BUTTONS;
  received_msg.buttons.data =
      (int32_t *)malloc(received_msg.buttons.capacity * sizeof(int32_t));
  received_msg.buttons.size = 0;

  echo_msg.buttons.capacity = MAX_BUTTONS;
  echo_msg.buttons.data =
      (int32_t *)malloc(echo_msg.buttons.capacity * sizeof(int32_t));
  echo_msg.buttons.size = 0;

  // Cabeçalho (frame_id) - Evita que strings enviadas pelo ROS 2 corrompam o
  // pacote
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
    printf("Tentando conectar ao micro-ROS agent...\n");

    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
      vTaskDelay(2000 / portTICK_PERIOD_MS);
      continue;
    }

    node = rcl_get_zero_initialized_node();
    if (rclc_node_init_default(&node, "esp32_control_node", "", &support) !=
        RCL_RET_OK) {
      rclc_support_fini(&support);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }

    echo_publisher = rcl_get_zero_initialized_publisher();
    if (rclc_publisher_init_default(
            &echo_publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
            "flightgear_attitude_echo") != RCL_RET_OK) {
      rcl_node_fini(&node);
      rclc_support_fini(&support);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }

    subscriber = rcl_get_zero_initialized_subscription();
    if (rclc_subscription_init_default(
            &subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
            "flightgear_attitude") != RCL_RET_OK) {
      rcl_publisher_fini(&echo_publisher, &node);
      rcl_node_fini(&node);
      rclc_support_fini(&support);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }

    executor = rclc_executor_get_zero_initialized_executor();
    if (rclc_executor_init(&executor, &support.context, 1, &allocator) !=
        RCL_RET_OK) {
      rcl_subscription_fini(&subscriber, &node);
      rcl_publisher_fini(&echo_publisher, &node);
      rcl_node_fini(&node);
      rclc_support_fini(&support);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }

    if (rclc_executor_add_subscription(&executor, &subscriber, &received_msg,
                                       &flightgear_subscription_callback,
                                       ON_NEW_DATA) != RCL_RET_OK) {
      rclc_executor_fini(&executor);
      rcl_subscription_fini(&subscriber, &node);
      rcl_publisher_fini(&echo_publisher, &node);
      rcl_node_fini(&node);
      rclc_support_fini(&support);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }

    printf("Conectado! Aguardando comandos...\n");

    while (1) {
      rcl_ret_t rc = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

      if (rc != RCL_RET_OK && rc != RCL_RET_TIMEOUT) {
        printf("Conexão perdida. Reiniciando nós...\n");
        break;
      }
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    RCSOFTCHECK(rcl_publisher_fini(&echo_publisher, &node));
    RCSOFTCHECK(rcl_subscription_fini(&subscriber, &node));
    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rclc_support_fini(&support));

    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }

  // A título de boas práticas C
  free(received_msg.axes.data);
  free(received_msg.buttons.data);
  free(echo_msg.axes.data);
  free(echo_msg.buttons.data);
  free(received_msg.header.frame_id.data);
  free(echo_msg.header.frame_id.data);
  vTaskDelete(NULL);
}

extern "C" void app_main() {
  static uart_port_t uart_port = UART_NUM_0;

  rmw_uros_set_custom_transport(true, (void *)&uart_port, esp32_serial_open,
                                esp32_serial_close, esp32_serial_write,
                                esp32_serial_read);

  xTaskCreatePinnedToCore(micro_ros_task, "uros_task",
                          MICRO_ROS_TASK_STACK_SIZE, NULL,
                          MICRO_ROS_TASK_PRIORITY, NULL, 1);
}