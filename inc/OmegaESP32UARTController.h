/**
 * @file OmegaESP32UARTController.h
 * @author Omegaki113r
 * @date Thursday, 17th October 2024 3:19:11 pm
 * @copyright Copyright 2024 - 2024 0m3g4ki113r, Xtronic
 * */
/*
 * Project: OmegaESP32UARTController
 * File Name: OmegaESP32UARTController.h
 * File Created: Thursday, 17th October 2024 3:19:11 pm
 * Author: Omegaki113r (omegaki113r@gmail.com)
 * -----
 * Last Modified: Monday, 21st October 2024 10:39:52 pm
 * Modified By: Omegaki113r (omegaki113r@gmail.com)
 * -----
 * Copyright 2024 - 2024 0m3g4ki113r, Xtronic
 * -----
 * HISTORY:
 * Date      	By	Comments
 * ----------	---	---------------------------------------------------------
 */

#ifndef __OMEGA_ESP32_UART_CONTROLLER_H__
#define __OMEGA_ESP32_UART_CONTROLLER_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <driver/uart.h>

#include "OmegaUARTController.h"
#include "OmegaUtilityDriver.h"

    typedef uint64_t OmegaESP32UARTHandle;
    typedef struct
    {

        OmegaESP32UARTHandle m_handle;
        uart_port_t m_uart_num;
        QueueHandle_t m_uart_queue;
        OmegaUARTController_t m_uart_controller;
    } OmegaESP32Controller_t;

    OmegaESP32UARTHandle OmegaESP32UARTController_init(uart_port_t);
    OmegaStatus OmegaESP32UARTController_set_baudrate(OmegaESP32UARTHandle, uint32_t);
    OmegaStatus OmegaESP32UARTController_set_databits(OmegaESP32UARTHandle, OmegaUARTDataBits);
    OmegaStatus OmegaESP32UARTController_set_parity(OmegaESP32UARTHandle, OmegaUARTParity);
    OmegaStatus OmegaESP32UARTController_set_stopbits(OmegaESP32UARTHandle, OmegaUARTStopBits);
    OmegaStatus OmegaESP32UARTController_set_gpio(OmegaESP32UARTHandle, OmegaGPIO_t, OmegaGPIO_t);
    OmegaStatus OmegaESP32UARTController_start(OmegaESP32UARTHandle);

    size_t OmegaESP32UARTController_transmit(OmegaESP32UARTHandle, uint8_t *, size_t, uint32_t);
    size_t OmegaESP32UARTController_receive(OmegaESP32UARTHandle, uint8_t *, size_t, uint32_t);
#ifdef __cplusplus
}
#endif

#endif // __OMEGA_ESP32_UART_CONTROLLER_H__