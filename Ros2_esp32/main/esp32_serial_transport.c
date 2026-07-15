#include "esp32_serial_transport.h"

#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MICRO_ROS_UART_BAUDRATE 115200
#define MICRO_ROS_UART_RX_BUFFER_SIZE 2048
#define MICRO_ROS_UART_TX_BUFFER_SIZE 2048

static uart_port_t get_uart_port(struct uxrCustomTransport *transport)
{
    if (transport == NULL || transport->args == NULL) {
        return UART_NUM_0;
    }

    return *((uart_port_t *)transport->args);
}

bool esp32_serial_open(struct uxrCustomTransport *transport)
{
    uart_port_t uart_port = get_uart_port(transport);

    uart_config_t uart_config = {
        .baud_rate = MICRO_ROS_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
#if ESP_IDF_VERSION_MAJOR >= 5
        .source_clk = UART_SCLK_DEFAULT,
#endif
    };

    esp_err_t ret;

    ret = uart_param_config(uart_port, &uart_config);
    if (ret != ESP_OK) {
        return false;
    }

    /*
     * Para UART_NUM_0, UART_PIN_NO_CHANGE usa os pinos padrão da placa.
     * No ESP32-S3 normalmente são GPIO43 TX e GPIO44 RX quando a console está em UART.
     */
    ret = uart_set_pin(
        uart_port,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    );
    if (ret != ESP_OK) {
        return false;
    }

    /*
     * Se o driver já estiver instalado, uart_driver_install pode falhar.
     * Então tentamos remover antes e instalar de novo.
     */
    uart_driver_delete(uart_port);

    ret = uart_driver_install(
        uart_port,
        MICRO_ROS_UART_RX_BUFFER_SIZE,
        MICRO_ROS_UART_TX_BUFFER_SIZE,
        0,
        NULL,
        0
    );

    return ret == ESP_OK;
}

bool esp32_serial_close(struct uxrCustomTransport *transport)
{
    uart_port_t uart_port = get_uart_port(transport);

    esp_err_t ret = uart_driver_delete(uart_port);

    return ret == ESP_OK;
}

size_t esp32_serial_write(
    struct uxrCustomTransport *transport,
    const uint8_t *buf,
    size_t len,
    uint8_t *errcode)
{
    uart_port_t uart_port = get_uart_port(transport);

    int written = uart_write_bytes(
        uart_port,
        (const char *)buf,
        len
    );

    if (written < 0) {
        if (errcode != NULL) {
            *errcode = 1;
        }
        return 0;
    }

    uart_wait_tx_done(uart_port, pdMS_TO_TICKS(100));

    if (errcode != NULL) {
        *errcode = 0;
    }

    return (size_t)written;
}

size_t esp32_serial_read(
    struct uxrCustomTransport *transport,
    uint8_t *buf,
    size_t len,
    int timeout,
    uint8_t *errcode)
{
    uart_port_t uart_port = get_uart_port(transport);

    TickType_t ticks_to_wait;

    if (timeout < 0) {
        ticks_to_wait = portMAX_DELAY;
    } else {
        ticks_to_wait = pdMS_TO_TICKS(timeout);
    }

    int read = uart_read_bytes(
        uart_port,
        buf,
        len,
        ticks_to_wait
    );

    if (read < 0) {
        if (errcode != NULL) {
            *errcode = 1;
        }
        return 0;
    }

    if (errcode != NULL) {
        *errcode = 0;
    }

    return (size_t)read;
}