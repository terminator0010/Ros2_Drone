#include "wokwi-api.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define PWM_MIN_US       1000.0f
#define PWM_CENTER_US    1500.0f
#define PWM_MAX_US       2000.0f

#define FEEDBACK_MIN_V   0.0f
#define FEEDBACK_MAX_V   3.3f

typedef struct {
  pin_t pin_pwm;
  pin_t pin_feedback;

  uint64_t pulse_start_ns;
  bool pulse_active;
} chip_state_t;


/*
 * Converte a largura do pulso PWM em tensão de feedback.
 *
 * 1000 us -> 0.00 V
 * 1500 us -> 1.65 V
 * 2000 us -> 3.30 V
 */
float pulse_to_voltage(float pulse_us) {

  if (pulse_us < PWM_MIN_US)
    pulse_us = PWM_MIN_US;

  if (pulse_us > PWM_MAX_US)
    pulse_us = PWM_MAX_US;

  float position =
      (pulse_us - PWM_MIN_US) /
      (PWM_MAX_US - PWM_MIN_US);

  return FEEDBACK_MIN_V +
         position * (FEEDBACK_MAX_V - FEEDBACK_MIN_V);
}


/*
 * Detecta as bordas do sinal PWM.
 */
void pwm_changed(void *user_data, pin_t pin, uint32_t value) {

  (void)pin;

  chip_state_t *chip =
      (chip_state_t *)user_data;

  uint64_t now = get_sim_nanos();

  /*
   * Início do pulso.
   */
  if (value == HIGH) {

    chip->pulse_start_ns = now;
    chip->pulse_active = true;

    return;
  }

  /*
   * Final do pulso.
   */
  if (value == LOW && chip->pulse_active) {

    uint64_t pulse_ns =
        now - chip->pulse_start_ns;

    float pulse_us =
        (float)pulse_ns / 1000.0f;

    float feedback_voltage =
        pulse_to_voltage(pulse_us);

    pin_dac_write(
        chip->pin_feedback,
        feedback_voltage
    );

    chip->pulse_active = false;
  }
}


void chip_init(void) {

  chip_state_t *chip =
      static_cast<chip_state_t *>(calloc(1, sizeof(chip_state_t)));

  /*
   * Entrada PWM.
   */
  chip->pin_pwm =
      pin_init("PWM", INPUT);

  /*
   * Saída analógica de feedback.
   */
  chip->pin_feedback =
      pin_init("FEEDBACK", ANALOG);

  /*
   * Monitora subida e descida do PWM.
   */
  const pin_watch_config_t watch_config = {
      .edge = BOTH,
      .pin_change = pwm_changed,
      .user_data = chip,
  };

  pin_watch(
      chip->pin_pwm,
      &watch_config
  );

  /*
   * Inicialmente o servo fica centralizado.
   */
  pin_dac_write(
      chip->pin_feedback,
      1.65f
  );

  printf(
      "Servo Feedback virtual inicializado!\n"
  );
}