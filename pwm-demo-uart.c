#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"
#include "hardware/uart.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pt_cornell_rp2040_v1_3.h"

// RPM SENSOR CONFIG
#define LEFT_SENSOR_PIN 26
#define RIGHT_SENSOR_PIN 27
#define M_PER_TICK 0.0243
#define VELOCITY_TIMEOUT_US 50000
#define FILTER_SIZE 10

// UART Configuration
#define UART_PORT uart0
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define BAUD_RATE 115200

// PWM CONFIG
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
float left_velocity_buffer[FILTER_SIZE] = {0};
float right_velocity_buffer[FILTER_SIZE] = {0};
int left_index = 0, right_index = 0;

// PWM VARS
uint slice_num = 1;
volatile float last_throttle = 0;
volatile float throttle = 0;
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

int calculate_rpm(float vel) { return (int)(vel * 60) / M_PER_TICK; }

void left_wheel_isr(uint gpio, uint32_t events)
{   
    gpio_put(25, 1);
    uint32_t current_time = time_us_32();
    if (last_left_time != 0)
    {
        uint32_t delta_time = current_time - last_left_time;
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
{   gpio_put(25, 1);
    uint32_t current_time = time_us_32();
    if (last_right_time != 0)
    {
        uint32_t delta_time = current_time - last_right_time;
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
    pwm_clear_irq(slice_num);
    uint32_t current_time_micro = time_us_32();
    float delta_time_s = ((float)(current_time_micro - last_time_micro)) / 1e6;
    last_time_micro = current_time_micro;

    float max_control = last_throttle + MAX_THROTTLE_CHANGE_PER_SECOND_UP * delta_time_s;
    float min_control = last_throttle - MAX_THROTTLE_CHANGE_PER_SECOND_DOWN * delta_time_s;

    if (max_control > 1.0) max_control = 1.0;
    if (min_control < 0.0) min_control = 0.0;

    if (throttle > max_control) throttle = max_control;
    if (throttle < min_control) throttle = min_control;

    last_throttle = throttle;
    int multiplier = 5400;

    if (right_velocity < min_scaling_vel)
    {
        multiplier = min_duty_cycle;
    }
    else if (right_velocity < max_scaling_vel)
    {
        float diff = right_velocity - min_scaling_vel;
        float mult_diff = multiplier - min_duty_cycle;
        multiplier = min_duty_cycle + (diff / (max_scaling_vel - min_scaling_vel)) * mult_diff;
    }

    control = (int)(throttle * multiplier);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, control);
    pwm_set_chan_level(slice_num, PWM_CHAN_B, control);
}

void uart_init_pico()
{
    uart_init(UART_PORT, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
}


/* 
Format for sending packets: 
<L_RPM:1200,R_RPM:1150>\n
*/
void send_uart_data(float left_velocity, float right_velocity)
{
    char buffer[64];
    int leftRPM = calculate_rpm(left_velocity);
    int rightRPM = calculate_rpm(right_velocity);

    // Above data format with fixed-length RPM values
    snprintf(buffer, sizeof(buffer), "<L_RPM:%04d,R_RPM:%04d>\n", leftRPM, rightRPM);

    // Print to console for debugging
    printf("Sending via UART: %s", buffer);

    // Send formatted data over UART
    uart_puts(UART_PORT, buffer);
}


int main()
{
    stdio_init_all();
    uart_init_pico();
    
    adc_init();
    adc_gpio_init(THROTTLE_ADC_PIN);
    adc_select_input(0);

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

    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    gpio_put(25, 1);

    gpio_set_function(PWM_OUT_A, GPIO_FUNC_PWM);
    gpio_set_function(PWM_OUT_B, GPIO_FUNC_PWM);

    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);
    pwm_set_wrap(slice_num, WRAPVAL);
    pwm_set_clkdiv(slice_num, CLKDIV);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);
    pwm_set_mask_enabled((1u << slice_num));

    uint32_t last_print_time = 0;

    while (1)
    {
        uint32_t current_time = time_us_32();
        if ((current_time - last_left_time) > VELOCITY_TIMEOUT_US) left_velocity = 0.0;
        if ((current_time - last_right_time) > VELOCITY_TIMEOUT_US) right_velocity = 0.0;

        if ((current_time - last_print_time) > 1000000)
        {
            last_print_time = current_time;
            send_uart_data(left_velocity, right_velocity);
        }

        uint16_t val = adc_read();
        throttle = (val < 1500) ? 0 : (((val - 1500) * (MAX_DUTY_CYCLE * 3))) / 5500;
    }
    return 0;
}
