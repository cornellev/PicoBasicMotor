#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"
#include "hardware/uart.h"
#include "hardware/spi.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
// RPM SENSOR CONFIG
#define LEFT_RPM_SENSOR_PIN 26
#define RIGHT_RPM_SENSOR_PIN 27
#define M_PER_TICK 0.0243 // 22 inch diameter wheel over 72 rising and falling edges per revolution
#define VELOCITY_TIMEOUT_US 500000
#define FILTER_SIZE 10
// SPI CONFIG
#define SPI_PORT        spi1
#define SPI_SCK_PIN     10
#define SPI_MOSI_PIN    11
#define SPI_MISO_PIN    12   // not used here, but good to wire for full-duplex
#define SPI_CS_PIN      13
#define SPI_BAUDRATE    1000000
// PWM CONFIG
#define WRAPVAL 5000
#define CLKDIV 5.0f
// GPIO output for PWM
#define LEFT_MOTOR_PWM 17
#define RIGHT_MOTOR_PWM 15
#define MOTOR_BRAKE 18
#define MOTOR_ENABLE 19
// THROTTLE INPUT
#define THROTTLE_ADC_PIN 27
// THROTTLE CONFIG
#define MAX_DUTY_CYCLE 1.0f
#define MAX_THROTTLE_CHANGE_PER_SECOND_UP .5f
#define MAX_THROTTLE_CHANGE_PER_SECOND_DOWN 2.0f
// STEER SENSOR CONFIG
#define STEER_ADC_PIN 28
// STEER VARS
float steering = 0.0;
// ENCODER VARS
volatile uint32_t last_left_time = 0;
volatile uint32_t last_right_time = 0;
float left_velocity_buffer[FILTER_SIZE] = {0};
float right_velocity_buffer[FILTER_SIZE] = {0};
int left_index = 0, right_index = 0;
volatile float left_velocity = 0.0;
volatile float right_velocity = 0.0;
// PWM VARS
int slice_num_a, slice_num_b;
volatile float last_throttle = 0;
volatile float throttle = 0;
volatile uint16_t control = 0;
volatile uint32_t last_time_micro = 0;
float min_scaling_vel = 0.0;
float max_scaling_vel = 5.0;
int min_duty_cycle = 825;

// TIMESTAMP EXTENSION
volatile uint64_t extended_timestamp = 0;
volatile uint32_t last_timestamp = 0;

float rolling_average(float *buffer, int size)
{
    float sum = 0.0;
    for (int i = 0; i < size; i++)
    {
        sum += buffer[i];
    }
    return sum / size;
}
int calculate_rpm(float vel) { return (vel / M_PER_TICK) * 60.0; }

uint32_t safe_time_difference(uint32_t newer, uint32_t older) {
    if (newer >= older) {
        return newer - older;
    } else {
        return (UINT32_MAX - older) + newer + 1;
    }
}

uint64_t get_extended_timestamp() {
    uint32_t current = time_us_32();
    uint32_t delta = safe_time_difference(current, last_timestamp);
    extended_timestamp += delta;
    last_timestamp = current;
    return extended_timestamp;
}

void left_wheel_isr(uint gpio, uint32_t events)
{
    gpio_put(25, 1);
    uint32_t current_time = time_us_32();
    if (last_left_time != 0)
    {
        uint32_t delta_time = safe_time_difference(current_time, last_left_time);
        float new_velocity = M_PER_TICK / (delta_time / 1e6);
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
        uint32_t delta_time = safe_time_difference(current_time, last_right_time);
        float new_velocity = M_PER_TICK / (delta_time / 1e6);
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
    pwm_clear_irq(slice_num_a);
    uint32_t current_time_micro = time_us_32();
    float delta_time_s = ((float)(safe_time_difference(current_time_micro, last_time_micro))) / 1e6;
    last_time_micro = current_time_micro;
    float max_control =
        last_throttle + MAX_THROTTLE_CHANGE_PER_SECOND_UP * delta_time_s;
    float min_control =
        last_throttle - MAX_THROTTLE_CHANGE_PER_SECOND_DOWN * delta_time_s;
    if (max_control > 1.0)
        max_control = 1.0;
    if (min_control < 0.0)
        min_control = 0.0;
    if (throttle > max_control)
        throttle = max_control;
    if (throttle < min_control)
        throttle = min_control;
    last_throttle = throttle;
    int multiplier = 5400;
    // if (left_velocity < min_scaling_vel)
    //     multiplier = min_duty_cycle;
    // else if (left_velocity < max_scaling_vel)
    // {
    //     float diff = left_velocity - min_scaling_vel;
    //     float mult_diff = multiplier - min_duty_cycle;
    //     multiplier = min_duty_cycle +
    //                  (diff / (max_scaling_vel - min_scaling_vel)) * mult_diff;
    // }
    control = (int)(throttle * multiplier);
    pwm_set_chan_level(slice_num_a, PWM_CHAN_B, control);
    pwm_set_chan_level(slice_num_b, PWM_CHAN_B, control);
}
// void uart_init_pico()
// {
//     uart_init(UART_PORT, BAUD_RATE);
//     gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
//     gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
// }
// helper, but is redundant...
// void send_spi_data(uint32_t timestamp,
//     int left_rpm,
//     float duty_frac,
//     float steering)
// {
// char buf[64];
// int len = snprintf(buf, sizeof(buf), "%u %d %f %f\n",
//         timestamp, left_rpm, duty_frac, steering);
// // assert chip‐select
// gpio_put(SPI_CS_PIN, 0);
// // master writes out the bytes
// spi_write_blocking(SPI_PORT, (uint8_t*)buf, len);
// // deassert CS
// gpio_put(SPI_CS_PIN, 1);
// }
float read_throttle()
{
    // adc_select_input(1);
    uint16_t val = adc_read();
    return (val < 1500) ? 0 : (((val - 1500.0) * (MAX_DUTY_CYCLE * 2.18))) / 5500.0;
}
float read_steering()
{
    adc_select_input(2);
    uint16_t val = adc_read();
    return (float)val;
}
int main()
{
    stdio_init_all();
    // uart_init_pico();
    // ↓ SPI-slave initialization ↓
    spi_init(SPI_PORT, SPI_BAUDRATE);
    spi_set_slave(SPI_PORT, true);
    gpio_set_function(SPI_SCK_PIN,  GPIO_FUNC_SPI);
    gpio_set_function(SPI_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_MISO_PIN, GPIO_FUNC_SPI);
    // leave CS as SIO input so we can poll it, or try FUNC_SPI if your board supports it:
    gpio_init(SPI_CS_PIN);
    gpio_set_dir(SPI_CS_PIN, GPIO_IN);
    gpio_pull_up(SPI_CS_PIN);
    adc_init();
    adc_gpio_init(THROTTLE_ADC_PIN);
    adc_select_input(1);
    // adc_gpio_init(STEER_ADC_PIN);
    gpio_init(MOTOR_BRAKE);
    gpio_init(MOTOR_ENABLE);
    gpio_set_dir(MOTOR_BRAKE, GPIO_OUT);
    gpio_set_dir(MOTOR_ENABLE, GPIO_OUT);
    gpio_put(MOTOR_BRAKE, 1);
    gpio_put(MOTOR_ENABLE, 1);
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    gpio_put(25, 1);
    gpio_set_function(LEFT_MOTOR_PWM, GPIO_FUNC_PWM);
    gpio_set_function(RIGHT_MOTOR_PWM, GPIO_FUNC_PWM);
    slice_num_a = pwm_gpio_to_slice_num(LEFT_MOTOR_PWM);
    slice_num_b = pwm_gpio_to_slice_num(RIGHT_MOTOR_PWM);
    pwm_clear_irq(slice_num_a);
    pwm_set_irq_enabled(slice_num_a, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);
    pwm_set_wrap(slice_num_a, WRAPVAL);
    pwm_set_wrap(slice_num_b, WRAPVAL);
    pwm_set_clkdiv(slice_num_a, CLKDIV);
    pwm_set_clkdiv(slice_num_b, CLKDIV);
    pwm_set_chan_level(slice_num_a, PWM_CHAN_B, 0);
    pwm_set_chan_level(slice_num_b, PWM_CHAN_B, 0);
    pwm_set_mask_enabled((1u << slice_num_a) | (1u << slice_num_b));
    // Configure GPIO for sensors
    gpio_init(LEFT_RPM_SENSOR_PIN);
    gpio_set_dir(LEFT_RPM_SENSOR_PIN, GPIO_IN);
    gpio_pull_up(LEFT_RPM_SENSOR_PIN);
    gpio_set_irq_enabled_with_callback(LEFT_RPM_SENSOR_PIN,
                                       GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                       true, &left_wheel_isr);
    gpio_init(RIGHT_RPM_SENSOR_PIN);
    gpio_set_dir(RIGHT_RPM_SENSOR_PIN, GPIO_IN);
    gpio_pull_up(RIGHT_RPM_SENSOR_PIN);
    gpio_set_irq_enabled_with_callback(RIGHT_RPM_SENSOR_PIN,
       GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
       true, &right_wheel_isr);
    last_timestamp = time_us_32();
    extended_timestamp = last_timestamp; 
    
    uint32_t last_print_time = 0;
    uint64_t last_ext_print_time = 0;
    while (1)
    {
        uint64_t current_ext_time = get_extended_timestamp();
        uint32_t current_time = last_timestamp;
        if (safe_time_difference(current_time, last_left_time) > VELOCITY_TIMEOUT_US)
            left_velocity = 0.0;
        if (safe_time_difference(current_time, last_right_time) > VELOCITY_TIMEOUT_US)
            right_velocity = 0.0;
        throttle = read_throttle();
        steering = 0.0;
        // steering = read_steering();
        float left_rpm = calculate_rpm(left_velocity);
        float right_rpm = calculate_rpm(right_velocity);
        if ((current_ext_time - last_ext_print_time) > 10000)
        {
            // Timestamp in s, left rpm, right rpm, throttle, steering
            // printf("%d %f %f %f\n", current_time, left_rpm, ((float)control) / 5500.0, steering);
            // Prepare ASCII telemetry
            char txbuf[64];
            int len = snprintf(txbuf, sizeof(txbuf),
                            "%llu %d %f %f\n",
                            current_ext_time, left_rpm, right_rpm,
                            ((float)control)/5500.0f,
                            steering);
            // Wait for Pi to pull CS low (start of transaction)
            while (gpio_get(SPI_CS_PIN)) tight_loop_contents();
            // Clock our TX FIFO out while Pi drives SCLK
            // spi_write_blocking(SPI_PORT, (uint8_t*)txbuf, len);
            // Shift out via SPI-peripheral API
            spi_write_read_blocking(SPI_PORT,
                (const uint8_t*)txbuf,
                NULL,
                len);
            // Wait for CS to go high again (end of transaction)
            while (!gpio_get(SPI_CS_PIN)) tight_loop_contents();
            last_ext_print_time = current_ext_time;
            last_print_time = current_time;
        }
    }
    return 0;
}









