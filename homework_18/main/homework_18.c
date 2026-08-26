#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_PORT I2C_NUM_0
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9
#define I2C_FREQUENCY_HZ 400000
#define MPU6050_ADDRESS 0x68
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_WHO_AM_I 0x75
#define UART_PORT UART_NUM_0
#define UART_RX_BUFFER_SIZE 256
#define COMMAND_BUFFER_SIZE 64
#define DEFAULT_PERIOD_MS 500
#define MIN_PERIOD_MS 100
#define MAX_PERIOD_MS 5000

static volatile bool measurement_due = false;
static uint32_t period_ms = DEFAULT_PERIOD_MS;
static esp_timer_handle_t measurement_timer;

static void uart_send(const char *text)
{
    uart_write_bytes(UART_PORT, text, strlen(text));
}

static void measurement_timer_callback(void *argument)
{
    (void)argument;
    measurement_due = true;
}

static esp_err_t mpu6050_write_register(uint8_t reg, uint8_t value)
{
    const uint8_t data[] = {reg, value};
    return i2c_master_write_to_device(I2C_PORT, MPU6050_ADDRESS, data,
                                      sizeof(data), pdMS_TO_TICKS(100));
}

static esp_err_t mpu6050_read_registers(uint8_t reg, uint8_t *data,
                                        size_t data_size)
{
    return i2c_master_write_read_device(I2C_PORT, MPU6050_ADDRESS, &reg, 1,
                                        data, data_size, pdMS_TO_TICKS(100));
}

static void init_i2c(void)
{
    const i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQUENCY_HZ,
        .clk_flags = 0,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));
}

static void init_mpu6050(void)
{
    uint8_t identity = 0;
    ESP_ERROR_CHECK(mpu6050_read_registers(MPU6050_WHO_AM_I, &identity, 1));
    if (identity != MPU6050_ADDRESS) {
        printf("error unexpected_mpu6050_id=0x%02x\r\n", identity);
        abort();
    }
    ESP_ERROR_CHECK(mpu6050_write_register(MPU6050_PWR_MGMT_1, 0x00));
}

static esp_err_t read_acceleration(float *ax, float *ay, float *az)
{
    uint8_t raw[6];
    ESP_RETURN_ON_ERROR(
        mpu6050_read_registers(MPU6050_ACCEL_XOUT_H, raw, sizeof(raw)),
        "mpu6050", "failed to read acceleration");

    const int16_t x = (int16_t)(((uint16_t)raw[0] << 8) | raw[1]);
    const int16_t y = (int16_t)(((uint16_t)raw[2] << 8) | raw[3]);
    const int16_t z = (int16_t)(((uint16_t)raw[4] << 8) | raw[5]);
    *ax = x / 16384.0f;
    *ay = y / 16384.0f;
    *az = z / 16384.0f;
    return ESP_OK;
}

static void restart_measurement_timer(void)
{
    ESP_ERROR_CHECK(esp_timer_stop(measurement_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(
        measurement_timer, (uint64_t)period_ms * 1000));
}

static void process_command(char *command)
{
    while (isspace((unsigned char)*command)) {
        ++command;
    }

    if (command[0] == 'p' && isspace((unsigned char)command[1])) {
        char *end = NULL;
        const long requested_period = strtol(command + 2, &end, 10);
        while (end != NULL && isspace((unsigned char)*end)) {
            ++end;
        }
        if (end != NULL && *end == '\0' &&
            requested_period >= MIN_PERIOD_MS &&
            requested_period <= MAX_PERIOD_MS) {
            period_ms = (uint32_t)requested_period;
            restart_measurement_timer();
            char reply[48];
            snprintf(reply, sizeof(reply), "ok period=%lu ms\r\n",
                     (unsigned long)period_ms);
            uart_send(reply);
            return;
        }
    }
    char reply[COMMAND_BUFFER_SIZE + 48];
    snprintf(reply, sizeof(reply),
             "error command='%s' use: p <100..5000>\r\n", command);
    uart_send(reply);
}

static void poll_uart_commands(void)
{
    static char command[COMMAND_BUFFER_SIZE];
    static size_t command_length = 0;
    uint8_t byte = 0;

    while (uart_read_bytes(UART_PORT, &byte, 1, 0) == 1) {
        if (byte == '\r' || byte == '\n') {
            if (command_length > 0) {
                command[command_length] = '\0';
                process_command(command);
                command_length = 0;
            }
        } else if (command_length + 1 < sizeof(command)) {
            command[command_length++] = (char)byte;
        } else {
            command_length = 0;
            uart_send("error command_too_long\r\n");
        }
    }
}

static void init_uart(void)
{
    const uart_config_t config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
    if (!uart_is_driver_installed(UART_PORT)) {
        ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_RX_BUFFER_SIZE,
                                            0, 0, NULL, 0));
    }
}

void app_main(void)
{
    init_uart();
    init_i2c();
    init_mpu6050();

    const esp_timer_create_args_t timer_config = {
        .callback = measurement_timer_callback,
        .name = "measurement",
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_config, &measurement_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(
        measurement_timer, (uint64_t)period_ms * 1000));
    uart_send("ready command: p <100..5000>\r\n");

    while (true) {
        poll_uart_commands();
        if (measurement_due) {
            measurement_due = false;
            float ax = 0.0f;
            float ay = 0.0f;
            float az = 0.0f;
            char report[112];
            if (read_acceleration(&ax, &ay, &az) == ESP_OK) {
                snprintf(report, sizeof(report),
                         "t=%lld ms ax=%.2f ay=%.2f az=%.2f mode=periodic\r\n",
                         esp_timer_get_time() / 1000, ax, ay, az);
            } else {
                snprintf(report, sizeof(report),
                         "t=%lld ms sensor=error mode=periodic\r\n",
                         esp_timer_get_time() / 1000);
            }
            uart_send(report);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
