#include "pico/multicore.h"
#include "pico/stdlib.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"

// RPM SENSOR CONFIG
#define LEFT_SENSOR_PIN 9
#define RIGHT_SENSOR_PIN 8
#define M_PER_TICK 0.0243
#define VELOCITY_TIMEOUT_US                                                    \
  50000 // 50ms timeout to reset velocity to 0 (to account for lack of
        // interrupt)
#define FILTER_SIZE 10 // Number of samples for rolling average

// 5 kHz frequency
#define WRAPVAL 5000
#define CLKDIV 5.0f

// GPIO output for PWM
#define PWM_OUT_A 18
#define PWM_OUT_B 19

#define BRAKE_A 16
#define ENABLE_A 17

#define BRAKE_B 14
#define ENABLE_B 15

#define THROTTLE_ADC_PIN 26

#define MAX_DUTY_CYCLE 1.0f

#define MAX_THROTTLE_CHANGE_PER_SECOND_UP 4.0f
#define MAX_THROTTLE_CHANGE_PER_SECOND_DOWN 8.0f

// ENCODER VARS
volatile uint32_t last_left_time = 0;
volatile uint32_t last_right_time = 0;
volatile float left_velocity = 0.0;
volatile float right_velocity = 0.0;

volatile int light_on = 0;

float left_velocity_buffer[FILTER_SIZE] = {0};
float right_velocity_buffer[FILTER_SIZE] = {0};
int left_index = 0, right_index = 0;

// PWM VARS
uint slice_num = 1;

volatile float last_throttle = 0;
volatile float throttle = 0;

volatile uint16_t last_control = 0;
volatile uint16_t control = 0;
volatile uint32_t last_time_micro = 0;

float min_scaling_vel = 1.0;
float max_scaling_vel = 5.0;
int min_duty_cycle = 2000;

// Maintain a rolling average on RPM sensing to smooth velocity
// and account for noise
float rolling_average(float *buffer, int size) {
  float sum = 0.0;
  for (int i = 0; i < size; i++) {
    sum += buffer[i];
  }
  return sum / size;
}

void left_wheel_isr(uint gpio, uint32_t events) {
  uint32_t current_time = time_us_32();
  if (last_left_time != 0) {
    uint32_t delta_time = current_time - last_left_time;
    float new_velocity =
        M_PER_TICK / (delta_time / 1e6); // Convert to meters per second
    left_velocity_buffer[left_index] = new_velocity;
    left_index = (left_index + 1) % FILTER_SIZE;

    float avg = rolling_average(left_velocity_buffer, FILTER_SIZE);
    if (avg < left_velocity + 1) {
      left_velocity = avg;
    }
  }
  last_left_time = current_time;
}

void right_wheel_isr(uint gpio, uint32_t events) {
  uint32_t current_time = time_us_32();
  if (last_right_time != 0) {
    uint32_t delta_time = current_time - last_right_time;
    float new_velocity =
        M_PER_TICK / (delta_time / 1e6); // Convert to meters per second
    right_velocity_buffer[right_index] = new_velocity;
    right_index = (right_index + 1) % FILTER_SIZE;

    float avg = rolling_average(right_velocity_buffer, FILTER_SIZE);
    if (avg < right_velocity + 1) {
      right_velocity = avg;
    }
  }
  last_right_time = current_time;
}

// Triggers every PWM Period
void on_pwm_wrap() {
  pwm_clear_irq(slice_num);

  // ---- Calculate throttle delta limits ----

  // Calculate change in time
  uint32_t current_time_micro = time_us_32();
  float delta_time_s = ((float)(current_time_micro - last_time_micro)) / 1e6;
  last_time_micro = current_time_micro;

  // Limit maximum and minimum throttle based on change in time and configured
  // limits
  float max_control =
      last_throttle + MAX_THROTTLE_CHANGE_PER_SECOND_UP * delta_time_s;
  float min_control =
      last_throttle - MAX_THROTTLE_CHANGE_PER_SECOND_DOWN * delta_time_s;

  // Bound new throttle
  if (max_control > 1.0) {
    max_control = 1.0;
  }

  if (min_control < 0.0) {
    min_control = 0.0;
  }

  if (throttle > max_control) {
    throttle = max_control;
  }

  if (throttle < min_control) {
    throttle = min_control;
  }

  last_throttle = throttle;

  // Out of maximum duty cycle approximately
  int multiplier = 5400;

  // Set duty cycle for both motor outputs
  control = (int)(throttle * multiplier);
  pwm_set_chan_level(slice_num, PWM_CHAN_A, control);
  pwm_set_chan_level(slice_num, PWM_CHAN_B, control);
}

int main() {
  stdio_init_all();

  // Initialize throttle
  adc_init();
  adc_gpio_init(THROTTLE_ADC_PIN);
  adc_select_input(0);

  // Active BRAKE and ENABLE pins to both motors
  gpio_init(BRAKE_A);
  gpio_init(ENABLE_A);
  gpio_init(BRAKE_B);
  gpio_init(ENABLE_B);

  gpio_set_dir(BRAKE_A, GPIO_OUT);
  gpio_set_dir(ENABLE_A, GPIO_OUT);
  gpio_set_dir(BRAKE_B, GPIO_OUT);
  gpio_set_dir(ENABLE_B, GPIO_OUT);

  gpio_put(BRAKE_A, 1);
  gpio_put(ENABLE_A, 1);
  gpio_put(BRAKE_B, 1);
  gpio_put(ENABLE_B, 1);

  // LED
  gpio_init(25);
  gpio_set_dir(25, GPIO_OUT);
  gpio_put(25, 1);

  gpio_set_function(PWM_OUT_A, GPIO_FUNC_PWM);
  gpio_set_function(PWM_OUT_B, GPIO_FUNC_PWM);

  // Schedule PWM setting interrupt
  pwm_clear_irq(slice_num);
  pwm_set_irq_enabled(slice_num, true);
  irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
  irq_set_enabled(PWM_IRQ_WRAP, true);

  pwm_set_wrap(slice_num, WRAPVAL);
  pwm_set_clkdiv(slice_num, CLKDIV);

  // Write PWM 0 to begin
  pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);
  pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);

  // Switch on PWM slice
  pwm_set_mask_enabled((1u << slice_num));

  // Configure GPIO for sensors
  gpio_init(LEFT_SENSOR_PIN);
  gpio_set_dir(LEFT_SENSOR_PIN, GPIO_IN);
  gpio_pull_up(LEFT_SENSOR_PIN);
  gpio_set_irq_enabled_with_callback(LEFT_SENSOR_PIN,
                                     GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                     true, &left_wheel_isr);

  gpio_init(RIGHT_SENSOR_PIN);
  gpio_set_dir(RIGHT_SENSOR_PIN, GPIO_IN);
  gpio_pull_up(RIGHT_SENSOR_PIN);
  gpio_set_irq_enabled_with_callback(RIGHT_SENSOR_PIN,
                                     GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                     true, &right_wheel_isr);

  while (true) {
    static int test_in;
    while (1) {
      uint32_t current_time = time_us_32();

      // Check for timeout on left wheel velocity interrupt
      if ((current_time - last_left_time) > VELOCITY_TIMEOUT_US) {
        left_velocity = 0.0;
      }

      // Check for timeout on right wheel velocity interrupt
      if ((current_time - last_right_time) > VELOCITY_TIMEOUT_US) {
        right_velocity = 0.0;
      }

      // Send velocity output through serial (temporary, for debugging)
      printf("Left Velocity: %.2f m/s, Right Velocity: %.2f m/s\n",
             left_velocity, right_velocity);

      // Read throttle input
      uint16_t val = adc_read();

      if (val < 1500) // Stop if too small value (safety)
      {
        throttle = 0;
      } else {
        throttle = (((val - 1500) * (MAX_DUTY_CYCLE * 3))) / 5500;
      }
    }
  }
}
