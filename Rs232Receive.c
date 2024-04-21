#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include <esp_err.h>
#include <string.h>

static const int BUF_SIZE = 1024;

#define UART_0_TX 1
#define UART_0_RX 3

void init(){
    QueueHandle_t uart_queue;
    const uart_config_t uartconfig = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uartconfig));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0,UART_0_TX, UART_0_RX,UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0,BUF_SIZE,BUF_SIZE,10,&uart_queue,0));
}

static void task_rx(){
    int len = 0;
    uint8_t data[128];
    while (1)
    {
        uart_get_buffered_data_len(UART_NUM_0,(size_t *)&len);
        uart_read_bytes(UART_NUM_0,data,len,100);
        printf("data - %.*s", len, data);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(){
    printf("Receive data:\n");
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    init();
    xTaskCreate(task_rx, "uart_task_rx", 1024 * 2, NULL, configMAX_PRIORITIES - 1, NULL);
}