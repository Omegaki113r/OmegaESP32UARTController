/**
 * @file main.c
 * @author Omegaki113r
 * @date Thursday, 17th October 2024 3:50:30 pm
 * @copyright Copyright 2024 - 2024 0m3g4ki113r, Xtronic
 * */
/*
 * Project: UART Trdansmit Example
 * File Name: main.c
 * File Created: Thursday, 17th October 2024 3:50:30 pm
 * Author: Omegaki113r (omegaki113r@gmail.com)
 * -----
 * Last Modified: Monday, 21st October 2024 10:41:43 pm
 * Modified By: Omegaki113r (omegaki113r@gmail.com)
 * -----
 * Copyright 2024 - 2024 0m3g4ki113r, Xtronic
 * -----
 * HISTORY:
 * Date      	By	Comments
 * ----------	---	---------------------------------------------------------
 */

#include <stdio.h>

#include <driver/gpio.h>

#include "OmegaESP32UARTController.h"

OmegaESP32UARTHandle handle;

void app_main(void)
{
    handle = OmegaESP32UARTController_init(UART_NUM_1);
    OmegaESP32UARTController_set_gpio(handle, (OmegaGPIO_t){.pin = GPIO_NUM_35}, (OmegaGPIO_t){.pin = GPIO_NUM_36});
    OmegaStatus state = OmegaESP32UARTController_start(handle);
    if (state == eFAILED)
    {
        printf("failed %d\n", __LINE__);
    }
    printf("amount sent: %d\n", OmegaESP32UARTController_transmit(handle, (uint8_t *)"hello\n", 5, 1000));
    char received[10] = {0};
    OmegaESP32UARTController_receive(handle, (uint8_t *)received, 100, 1000);
    printf("received: %s\n", received);
}
