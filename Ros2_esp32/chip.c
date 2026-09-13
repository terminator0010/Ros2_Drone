#include "wokwi-api.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define I2C_ADDRESS 0x40

#define PCA9685_MODE1 0x00
#define PCA9685_PRESCALE 0xFE
#define LED0_ON_L 0x06
#define LED0_ON_H 0x07
#define LED0_OFF_L 0x08
#define LED0_OFF_H 0x09

#define I2C_MASTER_SCL_IO 2  // Alterado de 22 para 2
#define I2C_MASTER_SDA_IO 21 // Alterado de 21 para 1

// O oscilador interno do PCA9685 roda a 25MHz
#define OSC_CLOCK 25000000.0

typedef struct {
  uint8_t registers[256];
  uint8_t current_reg;
  bool first_byte;
  pin_t pin_pwm0;

  // Controle do Timer PWM
  timer_t pwm_timer;
  uint16_t counter; // O PCA9685 tem um contador de 12 bits (0 a 4095)
} chip_state_t;

// Função executada ciclicamente pelo Timer (simula o relógio interno do chip)
void timer_tick(void *user_data) {
  chip_state_t *chip = (chip_state_t *)user_data;

  // Verifica se o chip está em Sleep (bit 4 do MODE1). Se sim, o oscilador
  // para.
  if (chip->registers[PCA9685_MODE1] & (1 << 4)) {
    return;
  }

  // Lê os valores exatos enviados pelo ESP32 para ligar e desligar o pulso
  uint16_t on_val =
      chip->registers[LED0_ON_L] | (chip->registers[LED0_ON_H] << 8);
  uint16_t off_val =
      chip->registers[LED0_OFF_L] | (chip->registers[LED0_OFF_H] << 8);

  // Lógica do contador de 12 bits do PCA9685
  if (chip->counter == on_val) {
    pin_write(chip->pin_pwm0, HIGH); // Sobe o pulso PWM
  }

  if (chip->counter == off_val) {
    pin_write(chip->pin_pwm0, LOW); // Desce o pulso PWM
  }

  chip->counter++;
  if (chip->counter >= 4096) {
    chip->counter = 0; // Reinicia o ciclo (um período completo do PWM)
  }
}

// Recalcula a velocidade do timer baseado no PRE_SCALE enviado pelo ESP32
void update_timer_interval(chip_state_t *chip) {
  uint8_t prescale = chip->registers[PCA9685_PRESCALE];
  // Fórmula padrão do datasheet do PCA9685 para calcular a frequência de
  // atualização Freq_PWM = 25MHz / (4096 * (prescale + 1)) O Wokwi pede o
  // intervalo em microssegundos (us) para cada incremento (1/4096 do ciclo)

  float update_rate_hz = OSC_CLOCK / (prescale + 1.0);
  uint32_t interval_us =
      (uint32_t)((1000000.0 / update_rate_hz) + 0.5); // Arredondado

  // No Wokwi, o intervalo mínimo do timer geralmente não pode ser menor que 1us
  if (interval_us < 1)
    interval_us = 1;

  timer_start(chip->pwm_timer, interval_us, true);
}

bool on_i2c_connect(void *user_data, uint32_t address, bool connect) {
  return true;
}

uint8_t on_i2c_read(void *user_data) {
  chip_state_t *chip = (chip_state_t *)user_data;
  uint8_t data = chip->registers[chip->current_reg];
  chip->current_reg++;
  return data;
}

bool on_i2c_write(void *user_data, uint8_t data) {
  chip_state_t *chip = (chip_state_t *)user_data;

  if (chip->first_byte) {
    chip->current_reg = data;
    chip->first_byte = false;
  } else {
    chip->registers[chip->current_reg] = data;

    // Se o ESP32 alterar o PRE_SCALE ou sair do Sleep Mode (MODE1), precisamos
    // recalcular o relógio
    if (chip->current_reg == PCA9685_PRESCALE ||
        chip->current_reg == PCA9685_MODE1) {
      update_timer_interval(chip);
    }

    chip->current_reg++;
  }
  return true;
}

void on_i2c_disconnect(void *user_data) {
  chip_state_t *chip = (chip_state_t *)user_data;
  chip->first_byte = true;
}

void chip_init(void) {
  chip_state_t *chip = malloc(sizeof(chip_state_t));
  chip->first_byte = true;
  chip->counter = 0;

  // Valores de Reset de fábrica
  chip->registers[PCA9685_MODE1] = 0x11;
  chip->registers[PCA9685_PRESCALE] = 0x1E;

  chip->pin_pwm0 = pin_init("PWM0", OUTPUT);

  // Inicializa o Timer, mas ele só começa a rodar de verdade quando o ESP32
  // envia os comandos iniciais
  const timer_config_t timer_config = {
      .callback = timer_tick,
      .user_data = chip,
  };
  chip->pwm_timer = timer_init(&timer_config);
  update_timer_interval(chip);

  const i2c_config_t i2c_config = {
      .user_data = chip,
      .address = I2C_ADDRESS,
      .scl = pin_init("SCL", INPUT),
      .sda = pin_init("SDA", INPUT),
      .connect = on_i2c_connect,
      .read = on_i2c_read,
      .write = on_i2c_write,
      .disconnect = on_i2c_disconnect,
  };

  i2c_init(&i2c_config);
  printf("Custom Chip PCA9685 (Canal 0 + Timer) Inicializado!\n");
}