#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include <esp_err.h>
#include <string.h>

#define BUF_SIZE (1024)

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
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0,UART_0_TX, UART_0_RX,22, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0,BUF_SIZE,BUF_SIZE,10,&uart_queue,0));
}

static void task_tx(){
    char *data = "AB";
    while (1)
    {
        int len = strlen(data);
        int txBytes= uart_write_bytes(UART_NUM_0, data, len);
        printf("\n");    
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
    
}

void app_main(void)
{
    printf("Send data: \n");
    vTaskDelay(1000/portTICK_PERIOD_MS);
    init();
    xTaskCreate(task_tx,"uart_task_tx",1024*2,NULL,configMAX_PRIORITIES - 1, NULL);
}
