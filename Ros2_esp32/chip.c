#include "wokwi-api.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define I2C_ADDRESS 0x40

// PCA9685 registers
#define PCA9685_MODE1 0x00
#define PCA9685_PRESCALE 0xFE

// First PWM channel registers
#define LED0_ON_L 0x06
#define LED0_ON_H 0x07
#define LED0_OFF_L 0x08
#define LED0_OFF_H 0x09

// Internal oscillator
#define OSC_CLOCK 25000000.0

// PCA9685
#define PCA9685_CHANNELS 8
#define PCA9685_RESOLUTION 4096

typedef struct {
  uint8_t registers[256];

  uint8_t current_reg;
  bool first_byte;

  // PWM output pins
  pin_t pin_pwm0;
  pin_t pin_pwm1;
  pin_t pin_pwm2;
  pin_t pin_pwm3;
  pin_t pin_pwm4;
  pin_t pin_pwm5;
  pin_t pin_pwm6;
  pin_t pin_pwm7;

  // PWM timer
  timer_t pwm_timer;

  // 12-bit PWM counter
  uint16_t counter;

} chip_state_t;


// ============================================================
// TIMER PWM
// ============================================================

void timer_tick(void *user_data) {

  chip_state_t *chip = (chip_state_t *)user_data;

  // ----------------------------------------------------------
  // Se o PCA9685 estiver em Sleep, o oscilador fica parado.
  // Bit 4 do MODE1 = SLEEP
  // ----------------------------------------------------------

  if (chip->registers[PCA9685_MODE1] & (1 << 4)) {
    return;
  }

  // ----------------------------------------------------------
  // Lista dos pinos PWM
  // ----------------------------------------------------------

  pin_t pwm_pins[PCA9685_CHANNELS] = {
    chip->pin_pwm0,
    chip->pin_pwm1,
    chip->pin_pwm2,
    chip->pin_pwm3,
    chip->pin_pwm4,
    chip->pin_pwm5,
    chip->pin_pwm6,
    chip->pin_pwm7
  };

  // ----------------------------------------------------------
  // Cada canal do PCA9685 possui 4 registros:
  //
  // Canal 0:
  //   0x06 = ON_L
  //   0x07 = ON_H
  //   0x08 = OFF_L
  //   0x09 = OFF_H
  //
  // Canal 1:
  //   0x0A - 0x0D
  //
  // Canal 2:
  //   0x0E - 0x11
  //
  // etc.
  //
  // Portanto:
  //
  // base = 0x06 + (canal * 4)
  // ----------------------------------------------------------

  for (int channel = 0;
       channel < PCA9685_CHANNELS;
       channel++) {

    uint8_t base = LED0_ON_L + (channel * 4);

    // --------------------------------------------------------
    // Valor ON
    // --------------------------------------------------------

    uint16_t on_val =
        (uint16_t)chip->registers[base] |
        ((uint16_t)chip->registers[base + 1] << 8);

    // --------------------------------------------------------
    // Valor OFF
    // --------------------------------------------------------

    uint16_t off_val =
        (uint16_t)chip->registers[base + 2] |
        ((uint16_t)chip->registers[base + 3] << 8);

    // Garante que estamos trabalhando apenas com 12 bits.
    on_val &= 0x0FFF;
    off_val &= 0x0FFF;

    // --------------------------------------------------------
    // Liga o PWM deste canal
    // --------------------------------------------------------

    if (chip->counter == on_val) {
      pin_write(pwm_pins[channel], HIGH);
    }

    // --------------------------------------------------------
    // Desliga o PWM deste canal
    // --------------------------------------------------------

    if (chip->counter == off_val) {
      pin_write(pwm_pins[channel], LOW);
    }
  }

  // ----------------------------------------------------------
  // Incrementa contador de 12 bits
  // ----------------------------------------------------------

  chip->counter++;

  if (chip->counter >= PCA9685_RESOLUTION) {
    chip->counter = 0;
  }
}


// ============================================================
// ATUALIZA INTERVALO DO TIMER
// ============================================================

void update_timer_interval(chip_state_t *chip) {

  uint8_t prescale =
      chip->registers[PCA9685_PRESCALE];

  // ----------------------------------------------------------
  // Frequência de PWM do PCA9685:
  //
  // F_PWM =
  //     25MHz / (4096 * (PRE_SCALE + 1))
  //
  // O timer do Wokwi precisa executar uma vez para cada
  // incremento do contador de 12 bits.
  // ----------------------------------------------------------

  float pwm_frequency =
      OSC_CLOCK /
      (PCA9685_RESOLUTION *
       (prescale + 1.0f));

  // ----------------------------------------------------------
  // Cada período PWM possui 4096 ticks.
  // ----------------------------------------------------------

  float tick_frequency =
      pwm_frequency * PCA9685_RESOLUTION;

  uint32_t interval_us =
      (uint32_t)(
        (1000000.0f / tick_frequency) + 0.5f
      );

  if (interval_us < 1) {
    interval_us = 1;
  }

  timer_start(
      chip->pwm_timer,
      interval_us,
      true
  );
}


// ============================================================
// I2C CONNECT
// ============================================================

bool on_i2c_connect(
    void *user_data,
    uint32_t address,
    bool connect) {

  // PCA9685 responde somente no endereço configurado.
  if (address != I2C_ADDRESS) {
    return false;
  }

  return true;
}


// ============================================================
// I2C READ
// ============================================================

uint8_t on_i2c_read(void *user_data) {

  chip_state_t *chip =
      (chip_state_t *)user_data;

  uint8_t data =
      chip->registers[chip->current_reg];

  chip->current_reg++;

  return data;
}


// ============================================================
// I2C WRITE
// ============================================================

bool on_i2c_write(
    void *user_data,
    uint8_t data) {

  chip_state_t *chip =
      (chip_state_t *)user_data;

  // ----------------------------------------------------------
  // Primeiro byte da transmissão I2C:
  //
  // indica qual registro será acessado.
  // ----------------------------------------------------------

  if (chip->first_byte) {

    chip->current_reg = data;
    chip->first_byte = false;

  } else {

    // --------------------------------------------------------
    // Escreve o dado no registro atual
    // --------------------------------------------------------

    chip->registers[chip->current_reg] = data;

    // --------------------------------------------------------
    // Se alterou PRESCALE ou MODE1, recalcula o timer.
    // --------------------------------------------------------

    if (chip->current_reg == PCA9685_PRESCALE ||
        chip->current_reg == PCA9685_MODE1) {

      update_timer_interval(chip);
    }

    // --------------------------------------------------------
    // Auto incremento do endereço do registro.
    // --------------------------------------------------------

    chip->current_reg++;
  }

  return true;
}


// ============================================================
// I2C DISCONNECT
// ============================================================

void on_i2c_disconnect(void *user_data) {

  chip_state_t *chip =
      (chip_state_t *)user_data;

  chip->first_byte = true;
}


// ============================================================
// CHIP INITIALIZATION
// ============================================================

void chip_init(void) {

  // ----------------------------------------------------------
  // calloc inicializa toda a memória com zero.
  //
  // Isso é importante porque registers[] contém os registros
  // internos do PCA9685.
  // ----------------------------------------------------------

  chip_state_t *chip =
      calloc(1, sizeof(chip_state_t));

  if (chip == NULL) {
    printf("ERRO: Falha ao alocar memoria do PCA9685\n");
    return;
  }

  chip->first_byte = true;
  chip->counter = 0;

  // ----------------------------------------------------------
  // Valores iniciais dos registros
  // ----------------------------------------------------------

  chip->registers[PCA9685_MODE1] = 0x11;

  // Valor inicial padrão.
  // O ESP32 posteriormente escreve 0x79 para aproximadamente
  // 50 Hz.
  chip->registers[PCA9685_PRESCALE] = 0x1E;

  // ----------------------------------------------------------
  // Inicialização dos oito canais PWM
  // ----------------------------------------------------------

  chip->pin_pwm0 =
      pin_init("PWM0", OUTPUT);

  chip->pin_pwm1 =
      pin_init("PWM1", OUTPUT);

  chip->pin_pwm2 =
      pin_init("PWM2", OUTPUT);

  chip->pin_pwm3 =
      pin_init("PWM3", OUTPUT);

  chip->pin_pwm4 =
      pin_init("PWM4", OUTPUT);

  chip->pin_pwm5 =
      pin_init("PWM5", OUTPUT);

  chip->pin_pwm6 =
      pin_init("PWM6", OUTPUT);

  chip->pin_pwm7 =
      pin_init("PWM7", OUTPUT);

  // ----------------------------------------------------------
  // Inicializa timer PWM
  // ----------------------------------------------------------

  const timer_config_t timer_config = {
    .callback = timer_tick,
    .user_data = chip,
  };

  chip->pwm_timer =
      timer_init(&timer_config);

  update_timer_interval(chip);

  // ----------------------------------------------------------
  // Inicialização I2C
  // ----------------------------------------------------------

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

  printf(
      "Custom Chip PCA9685 "
      "(8 canais PWM) Inicializado!\n"
  );
}