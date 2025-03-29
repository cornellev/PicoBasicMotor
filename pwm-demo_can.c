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
#include "hardware/spi.h"

#include "mcp2515.h"
#include "pt_cornell_rp2040_v1_3.h"


// CAN Configuration (Using SPI0, not the RPM sensor pins)
#define CAN_SPI_PORT spi0
#define CAN_MISO 0  // SPI0 RX
#define CAN_MOSI 3  // SPI0 TX
#define CAN_SCK 2   // SPI0 SCK
#define CAN_CS 1    // SPI0 CSn
#define CAN_INT 20   // Interrupt Pin

MCP2515 can(CAN_CS);

// RPM SENSOR CONFIG
#define LEFT_SENSOR_PIN 9
#define RIGHT_SENSOR_PIN 8
#define M_PER_TICK 0.0243         // TODO: Empirically tune
#define VELOCITY_TIMEOUT_US 50000 // 50ms timeout to reset velocity to 0 (to account for lack of interrupt)
#define FILTER_SIZE 10            // Number of samples for rolling average

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

//CAN VARS
uint32_t last_can_send_time = 0;
const uint32_t CAN_SEND_INTERVAL_MS = 100;  //TODO: Adjust as needed

#define QUEUE_SIZE 10
volatile float left_velocity_queue;
volatile float right_velocity_queue;
volatile bool can_data_ready = false;

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

int velocity_to_rpm(float velocity) {
    return (int)(velocity * 60.0 / M_PER_TICK);
}

// TODO : fix to get proper RPM data
void send_rpm_can() {
    struct can_frame frame;
    frame.can_id = 0x15; // RPM Message ID
    frame.can_dlc = 4;

    int left_rpm = velocity_to_rpm(left_velocity);
    int right_rpm = velocity_to_rpm(right_velocity);

    frame.data[0] = (left_rpm >> 8) & 0xFF;
    frame.data[1] = left_rpm & 0xFF;
    frame.data[2] = (right_rpm >> 8) & 0xFF;
    frame.data[3] = right_rpm & 0xFF;

    if (can.sendMessage(&frame) == MCP2515::ERROR_OK) {
        printf("Sent CAN RPM: Left: %d, Right: %d\n", left_rpm, right_rpm);
    } else {
        printf("CAN transmission failed!\n");
    }
}

static PT_THREAD(protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt);
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

        printf("Left Velocity: %.2f m/s, Right Velocity: %.2f m/s\n", left_velocity, right_velocity);

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

// Core 1 Task (Handles CAN Processing)
void can_core_task() {
    // Initialize SPI0 for MCP2515
    spi_init(CAN_SPI_PORT, 1 * 1000 * 1000);
    gpio_set_function(CAN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(CAN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(CAN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(CAN_CS, GPIO_FUNC_SIO);
    gpio_set_dir(CAN_CS, GPIO_OUT);
    gpio_put(CAN_CS, 1);

    // Initialize MCP2515
    can.reset();
    if (can.setBitrate(CAN_500KBPS) != MCP2515::ERROR_OK) {
        printf("Failed to set CAN bitrate\n");
        while (1);
    }
    can.setNormalMode();
    printf("CAN initialized successfully on Core 1\n");

    while (1) {
        if (can_data_ready) {
            send_rpm_can();
            can_data_ready = false;
        }
        sleep_ms(10);
    }
}


int main()
{
    stdio_init_all();
    sleep_ms(2000); // Allow USB connection

    // Initialize SPI0 for MCP2515 CAN Controller
    // spi_init(CAN_SPI_PORT, 1 * 1000 * 1000); // 1 MHz
    // gpio_set_function(CAN_MISO, GPIO_FUNC_SPI);
    // gpio_set_function(CAN_MOSI, GPIO_FUNC_SPI);
    // gpio_set_function(CAN_SCK, GPIO_FUNC_SPI);
    // gpio_init(CAN_CS);
    // gpio_set_dir(CAN_CS, GPIO_OUT);
    // gpio_put(CAN_CS, 1);

    // //Initialize MCP2515
    // can.reset();
    // if (can.setBitrate(CAN_500KBPS) != MCP2515::ERROR_OK) {
    //     printf("Failed to set CAN bitrate\n");
    //     while (1);
    // }
    // can.setNormalMode();
    // printf("CAN initialized successfully\n");

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

    multicore_launch_core1(can_core_task);

    while (1) {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());

        // Reset velocity if no pulses for timeout duration
        if ((current_time - last_left_time) > VELOCITY_TIMEOUT_US / 1000) {
            left_velocity = 0.0;
        }
        if ((current_time - last_right_time) > VELOCITY_TIMEOUT_US / 1000) {
            right_velocity = 0.0;
        }

        // Update CAN queue every 100ms
        if ((current_time - last_can_send_time) >= CAN_SEND_INTERVAL_MS) {
            left_velocity_queue = left_velocity;
            right_velocity_queue = right_velocity;
            can_data_ready = true;
            last_can_send_time = current_time;
        }

        sleep_ms(10);
    }

    // pt_add_thread(protothread_serial);
    // pt_schedule_start;
}
