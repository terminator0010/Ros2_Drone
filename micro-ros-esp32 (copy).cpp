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

#define MAX_AXES 5
#define MAX_BUTTONS 20

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

  echo_msg.axes.size = incoming_msg->axes.size;
  echo_msg.buttons.size = incoming_msg->buttons.size;

  for (size_t i = 0; i < incoming_msg->axes.size; i++) {
    echo_msg.axes.data[i] = incoming_msg->axes.data[i];
  }

  for (size_t i = 0; i < incoming_msg->buttons.size; i++) {
    echo_msg.buttons.data[i] = incoming_msg->buttons.data[i];
  }

  RCSOFTCHECK(rcl_publish(&echo_publisher, &echo_msg, NULL));
}

void micro_ros_task(void *arg) {
  (void)arg;

  // 1. Alocação de memória ESTÁTICA na Heap (Feita apenas uma vez na vida da
  // Task)
  received_msg.axes.capacity = MAX_AXES;
  received_msg.axes.data =
      (float *)malloc(received_msg.axes.capacity * sizeof(float));
  received_msg.axes.size = 0;

  received_msg.buttons.capacity = MAX_BUTTONS;
  received_msg.buttons.data =
      (int32_t *)malloc(received_msg.buttons.capacity * sizeof(int32_t));
  received_msg.buttons.size = 0;

  echo_msg.axes.capacity = MAX_AXES;
  echo_msg.axes.data = (float *)malloc(echo_msg.axes.capacity * sizeof(float));
  echo_msg.axes.size = 0;

  echo_msg.buttons.capacity = MAX_BUTTONS;
  echo_msg.buttons.data =
      (int32_t *)malloc(echo_msg.buttons.capacity * sizeof(int32_t));
  echo_msg.buttons.size = 0;

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;
  rcl_node_t node;
  rclc_executor_t executor;

  // LOOP EXTERNO: Trata a conexão e reconexão com o Agent
  while (1) {
    printf("Tentando conectar ao micro-ROS agent...\n");

    // Tenta inicializar o suporte básico
    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
      printf("Agent não encontrado. Nova tentativa em 2 segundos...\n");
      vTaskDelay(2000 / portTICK_PERIOD_MS);
      continue; // Falhou, volta para o início do loop externo
    }

    // Tenta inicializar o Nó
    node = rcl_get_zero_initialized_node();
    if (rclc_node_init_default(&node, "esp32_control_node", "", &support) !=
        RCL_RET_OK) {
      rclc_support_fini(&support);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }

    // Tenta inicializar o Publisher de Eco
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

    // Tenta inicializar o Subscriber do Joystick
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

    // Tenta inicializar o Executor
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

    // Tenta vincular o subscriber ao executor
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

    printf("Conectado com sucesso ao micro-ROS agent! Iniciando loop de "
           "controle.\n");

    // LOOP INTERNO: Execução contínua enquanto a sessão estiver saudável
    while (1) {
      rcl_ret_t rc = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

      // Se a resposta do spin indicar falha crônica na sessão atual
      if (rc != RCL_RET_OK && rc != RCL_RET_TIMEOUT) {
        printf("Sessão perdida com o agente! Destruindo nós antigos...\n");
        break; // Sai do loop interno e vai para a rotina de limpeza abaixo
      }

      vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    // LIMPEZA DA SESSÃO ATUAL: Desfaz as entidades criadas antes de tentar uma
    // nova conexão
    RCSOFTCHECK(rcl_publisher_fini(&echo_publisher, &node));
    RCSOFTCHECK(rcl_subscription_fini(&subscriber, &node));
    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rclc_support_fini(&support));

    printf("Estruturas antigas limpas da memória. Aguardando para tentar "
           "reconexão...\n");
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }

  // Linhas abaixo nunca serão alcançadas pelo loop eterno, mas são mantidas por
  // conformidade de compilação
  free(received_msg.axes.data);
  free(received_msg.buttons.data);
  free(echo_msg.axes.data);
  free(echo_msg.buttons.data);
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