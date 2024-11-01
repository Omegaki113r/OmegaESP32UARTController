/**
 * @file OmegaESP32UARTController.c
 * @author Omegaki113r
 * @date Thursday, 17th October 2024 3:29:37 pm
 * @copyright Copyright 2024 - 2024 0m3g4ki113r, Xtronic
 * */
/*
 * Project: OmegaESP32UARTController
 * File Name: OmegaESP32UARTController.c
 * File Created: Thursday, 17th October 2024 3:29:37 pm
 * Author: Omegaki113r (omegaki113r@gmail.com)
 * -----
 * Last Modified: Monday, 21st October 2024 10:39:10 pm
 * Modified By: Omegaki113r (omegaki113r@gmail.com)
 * -----
 * Copyright 2024 - 2024 0m3g4ki113r, Xtronic
 * -----
 * HISTORY:
 * Date      	By	Comments
 * ----------	---	---------------------------------------------------------
 */
#include <string.h>

#include "OmegaESP32UARTController.h"
#include "OmegaUtilityDriver.h"

#define OMEGA_ESP32_UART_TX_RING_BUFFER_SIZE (1024 * 1)
#define OMEGA_ESP32_UART_RX_RING_BUFFER_SIZE (1024 * 1)
#define OMEGA_ESP32_UART_QUEUE_SIZE (10)

typedef enum
{
    eESP32_PARITY_DISABLE = 0,
    eESP32_PARITY_EVEN = 2,
    eESP32_PARITY_ODD = 3,
} ESP32Parity;

internal OmegaESP32Controller_t s_controller[UART_NUM_MAX];

OmegaESP32UARTHandle OmegaESP32UARTController_init(uart_port_t in_uart_num)
{
    memset(&s_controller, 0, sizeof(s_controller));
    s_controller[in_uart_num] = (OmegaESP32Controller_t){
        .m_handle = 1,
        .m_uart_num = in_uart_num,
        .m_uart_controller = {
            .m_started = false,
            .m_baud_rate = 115200,
            .m_data_bits = eDATA_BITS_8,
            .m_parity = ePARITY_DISABLE,
            .m_stop_bits = eSTOP_BITS_1,
            .m_tx_pin = {.port = NULL, .pin = OMEGA_GPIO_PIN_NC},
            .m_rx_pin = {.port = NULL, .pin = OMEGA_GPIO_PIN_NC},
        },
    };
    return s_controller[in_uart_num].m_handle;
}

OmegaStatus OmegaESP32UARTController_set_baudrate(OmegaESP32UARTHandle in_handle, uint32_t in_baud_rate)
{
    OmegaStatus status = eFAILED;
    if (in_handle <= 0)
    {
        goto ret;
    }
    for (uint8_t controller_idx = 0; controller_idx < sizeof(s_controller); ++controller_idx)
    {
        if (s_controller[controller_idx].m_handle == in_handle)
        {
            s_controller[controller_idx].m_uart_controller.m_baud_rate = in_baud_rate;
            if (s_controller[controller_idx].m_uart_controller.m_started)
            {
                if (ESP_OK != uart_set_baudrate(s_controller[controller_idx].m_uart_num, in_baud_rate))
                {
                    goto ret;
                }
            }
            status = eSUCCESS;
            goto ret;
        }
    }
ret:
    return status;
}

OmegaStatus OmegaESP32UARTController_set_databits(OmegaESP32UARTHandle in_handle, OmegaUARTDataBits in_data_bits)
{
    OmegaStatus status = eFAILED;
    if (in_handle <= 0)
    {
        goto ret;
    }
    if (in_data_bits >= eDATA_BITS_MAX)
    {
        goto ret;
    }
    for (uint8_t controller_idx = 0; controller_idx < sizeof(s_controller); ++controller_idx)
    {
        if (s_controller[controller_idx].m_handle == in_handle)
        {
            s_controller[controller_idx].m_uart_controller.m_data_bits = in_data_bits;
            if (s_controller[controller_idx].m_uart_controller.m_started)
            {
                if (ESP_OK != uart_set_word_length(s_controller[controller_idx].m_uart_num, in_data_bits))
                {
                    goto ret;
                }
            }
            status = eSUCCESS;
            goto ret;
        }
    }
ret:
    return status;
}

OmegaStatus OmegaESP32UARTController_set_parity(OmegaESP32UARTHandle in_handle, OmegaUARTParity in_parity)
{
    OmegaStatus status = eFAILED;
    if (in_handle <= 0)
    {
        goto ret;
    }
    if (in_parity >= ePARITY_MAX)
    {
        goto ret;
    }
    for (uint8_t controller_idx = 0; controller_idx < sizeof(s_controller); ++controller_idx)
    {
        if (s_controller[controller_idx].m_handle == in_handle)
        {
            switch (in_parity)
            {
            case ePARITY_DISABLE:
            {
                in_parity = eESP32_PARITY_DISABLE;
                break;
            }
            case ePARITY_EVEN:
            {
                in_parity = eESP32_PARITY_EVEN;
                break;
            }
            case ePARITY_ODD:
            {
                in_parity = eESP32_PARITY_ODD;
                break;
            }
            default:
            {
                goto ret;
            }
            }
            s_controller[controller_idx].m_uart_controller.m_parity = in_parity;
            if (s_controller[controller_idx].m_uart_controller.m_started)
            {
                if (ESP_OK != uart_set_parity(s_controller[controller_idx].m_uart_num, in_parity))
                {
                    goto ret;
                }
            }
            status = eSUCCESS;
            goto ret;
        }
    }
ret:
    return status;
}

OmegaStatus OmegaESP32UARTController_set_stopbits(OmegaESP32UARTHandle in_handle, OmegaUARTStopBits in_stopbits)
{
    OmegaStatus status = eFAILED;
    if (in_handle <= 0)
    {
        goto ret;
    }
    if (in_stopbits >= eSTOP_BITS_MAX)
    {
        goto ret;
    }
    for (uint8_t controller_idx = 0; controller_idx < sizeof(s_controller); ++controller_idx)
    {
        if (s_controller[controller_idx].m_handle == in_handle)
        {
            s_controller[controller_idx].m_uart_controller.m_stop_bits = in_stopbits;
            if (s_controller[controller_idx].m_uart_controller.m_started)
            {
                if (ESP_OK != uart_set_stop_bits(s_controller[controller_idx].m_uart_num, in_stopbits))
                {
                    goto ret;
                }
            }
            status = eSUCCESS;
            goto ret;
        }
    }
ret:
    return status;
}

OmegaStatus OmegaESP32UARTController_set_gpio(OmegaESP32UARTHandle in_handle, OmegaGPIO_t in_tx_pin, OmegaGPIO_t in_rx_pin)
{
    OmegaStatus status = eFAILED;
    if (in_handle <= 0)
    {
        goto ret;
    }
    if (in_tx_pin.pin == OMEGA_GPIO_PIN_NC && in_rx_pin.pin == OMEGA_GPIO_PIN_NC)
    {
        goto ret;
    }
    for (uint8_t controller_idx = 0; controller_idx < sizeof(s_controller); ++controller_idx)
    {
        if (s_controller[controller_idx].m_handle == in_handle)
        {
            s_controller[controller_idx].m_uart_controller.m_tx_pin = in_tx_pin;
            s_controller[controller_idx].m_uart_controller.m_rx_pin = in_rx_pin;
            if (s_controller[controller_idx].m_uart_controller.m_started)
            {
                if (ESP_OK != uart_set_pin(s_controller[controller_idx].m_uart_num,
                                           s_controller[controller_idx].m_uart_controller.m_tx_pin.pin, s_controller[controller_idx].m_uart_controller.m_rx_pin.pin, OMEGA_GPIO_PIN_NC, OMEGA_GPIO_PIN_NC))
                {
                    goto ret;
                }
            }
            status = eSUCCESS;
            goto ret;
        }
    }
ret:
    return status;
}

OmegaStatus OmegaESP32UARTController_start(OmegaESP32UARTHandle in_handle)
{
    OmegaStatus status = eFAILED;
    if (in_handle <= 0)
    {
        goto ret;
    }
    for (uint8_t controller_idx = 0; controller_idx < sizeof(s_controller); ++controller_idx)
    {
        if (s_controller[controller_idx].m_handle == in_handle)
        {
            if (!s_controller[controller_idx].m_uart_controller.m_started)
            {
                s_controller[controller_idx].m_uart_controller.m_started = true;
                uart_config_t uart_config = {
                    .baud_rate = s_controller[controller_idx].m_uart_controller.m_baud_rate,
                    .data_bits = s_controller[controller_idx].m_uart_controller.m_data_bits,
                    .parity = s_controller[controller_idx].m_uart_controller.m_parity,
                    .stop_bits = s_controller[controller_idx].m_uart_controller.m_stop_bits,
                };
                if (ESP_OK != uart_param_config(s_controller[controller_idx].m_uart_num, &uart_config))
                {
                    goto ret;
                }
                if (s_controller[controller_idx].m_uart_controller.m_tx_pin.pin == OMEGA_GPIO_PIN_NC && s_controller[controller_idx].m_uart_controller.m_tx_pin.pin == OMEGA_GPIO_PIN_NC)
                {
                    goto ret;
                }
                if (ESP_OK != uart_set_pin(s_controller[controller_idx].m_uart_num,
                                           s_controller[controller_idx].m_uart_controller.m_tx_pin.pin, s_controller[controller_idx].m_uart_controller.m_rx_pin.pin, OMEGA_GPIO_PIN_NC, OMEGA_GPIO_PIN_NC))
                {
                    goto ret;
                }
                if (ESP_OK != uart_driver_install(s_controller[controller_idx].m_uart_num,
                                                  OMEGA_ESP32_UART_TX_RING_BUFFER_SIZE, OMEGA_ESP32_UART_RX_RING_BUFFER_SIZE,
                                                  OMEGA_ESP32_UART_QUEUE_SIZE, &s_controller[controller_idx].m_uart_queue, 0))
                {
                    goto ret;
                }
                status = eSUCCESS;
                goto ret;
            }
            status = eALREADY_STARTED;
            goto ret;
        }
    }
ret:
    return status;
}

size_t OmegaESP32UARTController_transmit(OmegaESP32UARTHandle in_handle, uint8_t *in_data, size_t in_data_size, uint32_t in_timeout_ms)
{
    size_t transmitted_byte_count = 0;
    if (0 >= in_handle)
    {
        goto ret;
    }
    if (NULL == in_data)
    {
        goto ret;
    }
    if (0 >= in_data_size)
    {
        goto ret;
    }
    for (uint8_t controller_idx = 0; controller_idx < sizeof(s_controller); ++controller_idx)
    {
        if (s_controller[controller_idx].m_handle == in_handle)
        {
            if (!s_controller[controller_idx].m_uart_controller.m_started)
            {
                goto ret;
            }
            if (ESP_OK != uart_wait_tx_done(s_controller[controller_idx].m_uart_num, pdMS_TO_TICKS(in_timeout_ms)))
            {
                goto ret;
            }
            if (in_data_size != (transmitted_byte_count = uart_write_bytes(s_controller[controller_idx].m_uart_num, in_data, in_data_size)))
            {
                goto ret;
            }
            goto ret;
        }
    }
ret:
    return transmitted_byte_count;
}

size_t OmegaESP32UARTController_receive(OmegaESP32UARTHandle in_handle, uint8_t *out_data, size_t in_data_size, uint32_t in_timeout_ms)
{
    size_t received_byte_count = 0;
    if (in_handle <= 0)
    {
        goto ret;
    }
    if (NULL == out_data)
    {
        goto ret;
    }
    if (0 >= in_data_size)
    {
        goto ret;
    }
    for (uint8_t controller_idx = 0; controller_idx < sizeof(s_controller); ++controller_idx)
    {
        if (s_controller[controller_idx].m_handle == in_handle)
        {
            if (!s_controller[controller_idx].m_uart_controller.m_started)
            {
                goto ret;
            }
            if (in_data_size != (received_byte_count = uart_read_bytes(s_controller[controller_idx].m_uart_num, out_data, in_data_size, pdMS_TO_TICKS(in_timeout_ms))))
            {
                goto ret;
            }
            goto ret;
        }
    }
ret:
    return received_byte_count;
}
