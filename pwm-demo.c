#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/i2c.h"
#include "pt_cornell_rp2040_v1_3.h"
// RPM SENSOR CONFIG
#define LEFT_SENSOR_PIN 6          //TODO : Change pin according to set-up
#define RIGHT_SENSOR_PIN 7        //TODO : Change pin according to set-up
#define M_PER_TICK 0.0243         // TODO: Empirically tune
#define VELOCITY_TIMEOUT_US 50000 // 50ms timeout to reset velocity to 0 (to account for lack of interrupt)
#define FILTER_SIZE 10            // Number of samples for rolling average
// I2C Configuration
#define I2C_PORT i2c0                  // I2C port to use
#define I2C_SDA_PIN 8               //TODO: Change pin according to set-up
#define I2C_SCL_PIN 9               //TODO: Change pin according to set-up
#define I2C_ADDR 0x55 // I2C device address
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
float rolling_average(float *buffer, int size)
{
    float sum = 0.0;
    for (int i = 0; i < size; i++)
    {
        sum += buffer[i];
    }
    return sum / size;
}
int calculate_rpm(float vel){
    return (int)(vel *60)/M_PER_TICK;
}
void left_wheel_isr(uint gpio, uint32_t events)
{
    gpio_put(25, 1);
    uint32_t current_time = time_us_32();
    if (last_left_time != 0)
    {
        uint32_t delta_time = current_time - last_left_time;
        float new_velocity = M_PER_TICK / (delta_time / 1e6); // Convert to meters per second
        left_velocity_buffer[left_index] = new_velocity;
        left_index = (left_index + 1) % FILTER_SIZE;
        float avg = rolling_average(left_velocity_buffer, FILTER_SIZE);
        if (avg < left_velocity + 1)
        {
            left_velocity = avg;
        }
    }
    last_left_time = current_time;
}
void right_wheel_isr(uint gpio, uint32_t events)
{
    gpio_put(25, 1);
    uint32_t current_time = time_us_32();
    if (last_right_time != 0)
    {
        uint32_t delta_time = current_time - last_right_time;
        float new_velocity = M_PER_TICK / (delta_time / 1e6); // Convert to meters per second
        right_velocity_buffer[right_index] = new_velocity;
        right_index = (right_index + 1) % FILTER_SIZE;
        float avg = rolling_average(right_velocity_buffer, FILTER_SIZE);
        if (avg < right_velocity + 1)
        {
            right_velocity = avg;
        }
    }
    last_right_time = current_time;
}
void on_pwm_wrap()
{
    // Clear the interrupt flag that brought us here
    pwm_clear_irq(slice_num);
    // Get current time
    uint32_t current_time_micro = time_us_32();
    float delta_time_s = ((float)(current_time_micro - last_time_micro)) / 1e6;
    last_time_micro = current_time_micro;
    float max_control = last_throttle + MAX_THROTTLE_CHANGE_PER_SECOND_UP * delta_time_s;
    float min_control = last_throttle - MAX_THROTTLE_CHANGE_PER_SECOND_DOWN * delta_time_s;
    if (max_control > 1.0)
    {
        max_control = 1.0;
    }
    if (min_control < 0.0)
    {
        min_control = 0.0;
    }
    if (throttle > max_control)
    {
        throttle = max_control;
    }
    if (throttle < min_control)
    {
        throttle = min_control;
    }
    last_throttle = throttle;
    int multiplier = 5400;
    // if (right_velocity < min_scaling_vel)
    // {
    //     multiplier = min_duty_cycle;
    // }
    // else if (right_velocity < max_scaling_vel)
    // {
    //     float diff = right_velocity - min_scaling_vel;
    //     float mult_diff = multiplier - min_duty_cycle;
    //     multiplier = min_duty_cycle + (diff / (max_scaling_vel - min_scaling_vel)) * mult_diff;
    // }
    // Update duty cycle if control input changed
    // Both channels correspond to one GPIO pin
    control = (int)(throttle * multiplier);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, control);
    pwm_set_chan_level(slice_num, PWM_CHAN_B, control);
}
static PT_THREAD(protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt);
    static uint32_t last_print_time = 0;
    static int test_in;
    while (1)
    {
        uint32_t current_time = time_us_32();
        // Check for timeout on left wheel
        if ((current_time - last_left_time) > VELOCITY_TIMEOUT_US)
        {
            left_velocity = 0.0;
        }
        // Check for timeout on right wheel
        if ((current_time - last_right_time) > VELOCITY_TIMEOUT_US)
        {
            right_velocity = 0.0;
        }

        if (current_time-last_print_time > 1000000) // Print every second
        {
            last_print_time = current_time;
            //printf("Left Velocity: %.2f m/s, Right Velocity: %.2f m/s\n", left_velocity, right_velocity);
            printf("Left RPM: %d , Right RPM: %d\n", calculate_rpm(left_velocity), calculate_rpm(right_velocity));
        }
        //printf("Left Velocity: %.2f m/s, Right Velocity: %.2f m/s\n", left_velocity, right_velocity);
        // Read throttle
        uint16_t val = adc_read();
        if (val < 1500) // Stop if too small value (safety)
        {
            throttle = 0;
        }
        else
        {
            // throttle = (val - 1500) * (3.0 / 5500.0);
            throttle = (((val - 1500) * (MAX_DUTY_CYCLE * 3))) / 5500;
        }
    }
    PT_END(pt);
}
void i2c_init_pico() {
    i2c_init(I2C_PORT, 100000); // 1 MHz I2C clock speed
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
}

void send_i2c_data(float left_velocity, float right_velocity) {
    // ---- ADD I2C SEND FUNCTIONALITY ----
    uint8_t data[8];
    // Convert left_velocity to int (same format as receive function expects)
    int leftRPM = calculate_rpm(left_velocity);  // Convert m/s to RPM equivalent scale
    int rightRPM = calculate_rpm(right_velocity); // Convert m/s to RPM equivalent scale
    data[0] = (leftRPM >> 24) & 0xFF;
    data[1] = (leftRPM >> 16) & 0xFF;
    data[2] = (leftRPM >> 8) & 0xFF;
    data[3] = leftRPM & 0xFF;
    data[4] = (rightRPM >> 24) & 0xFF;
    data[5] = (rightRPM >> 16) & 0xFF;
    data[6] = (rightRPM >> 8) & 0xFF;
    data[7] = rightRPM & 0xFF;
    // Send RPM data via I2CI2C_PORT
    i2c_write_blocking(I2C_PORT, I2C_ADDR, data, 8, true);
    printf("Left RPM: %d , Right RPM: %d\n", leftRPM, rightRPM);

}
int main()
{
    stdio_init_all();
    i2c_init_pico();
    // Initialize throttle
    adc_init();
    adc_gpio_init(THROTTLE_ADC_PIN);
    adc_select_input(0);
    // GPIO Switch On Enable and Brake for both channels
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
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);
    // Switch on PWM slice
    pwm_set_mask_enabled((1u << slice_num));
    // Configure GPIO for sensors
    gpio_init(LEFT_SENSOR_PIN);
    gpio_set_dir(LEFT_SENSOR_PIN, GPIO_IN);
    gpio_pull_up(LEFT_SENSOR_PIN);
    gpio_set_irq_enabled_with_callback(LEFT_SENSOR_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &left_wheel_isr);
    gpio_init(RIGHT_SENSOR_PIN);
    gpio_set_dir(RIGHT_SENSOR_PIN, GPIO_IN);
    gpio_pull_up(RIGHT_SENSOR_PIN);
    gpio_set_irq_enabled_with_callback(RIGHT_SENSOR_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &right_wheel_isr);
    send_i2c_data(left_velocity, right_velocity); 
    pt_add_thread(protothread_serial);
    pt_schedule_start;
}