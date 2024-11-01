/**
 * @file OmegaUtilityDriver.h
 * @author Omegaki113r
 * @date Tuesday, 2nd July 2024 12:59:59 pm
 * @copyright Copyright 2024 - 2024 0m3g4ki113r, Xtronic
 * */
/*
 * Project: OmegaUtilityDriver
 * File Name: OmegaUtilityDriver.h
 * File Created: Tuesday, 2nd July 2024 12:59:59 pm
 * Author: Omegaki113r (omegaki113r@gmail.com)
 * -----
 * Last Modified: Wednesday, 11th September 2024 12:27:48 am
 * Modified By: Omegaki113r (omegaki113r@gmail.com)
 * -----
 * Copyright 2024 - 2024 0m3g4ki113r, Xtronic
 * -----
 * HISTORY:
 * Date      	By	Comments
 * ----------	---	---------------------------------------------------------
 *
 * 06-07-2024	0m3g4	adding OMEGA_STR2MAC implemented
 *
 * 05-07-2024	0m3g4	adding MAC_ADDRESS_BUFFER_LENGTH and MAC2STR
 */

#ifndef __OMEGA_UTILITY_DRIVER_H__
#define __OMEGA_UTILITY_DRIVER_H__

#ifdef __cplusplus
extern "C"
{
#endif

#if ESP_PLATFORM
#include <esp_heap_caps.h>
#endif

#if __has_include("OmegaBaseLoggingController.h")
#include "OmegaBaseLoggingController.h"
#else
#define OMEGA_LOGE
#endif

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

#define internal static
#define UNUSED(arg) (void)arg

#define GPIO_LEVEL_LOW 0
#define GPIO_LEVEL_HIGH 1

#define MAC_ADDRESS_BUFFER_LENGTH 6
#define MAC_ADDRESS_STRING_BUFFER_LENGTH 17
#define MAC2STR_STRING "%02x:%02x:%02x:%02x:%02x:%02x"
#define STR2MAC_STRING "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx"
#define BUFFER_EXPAND(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define OMEGA_MAC2STR(addr, str)                                \
    do                                                          \
    {                                                           \
        if (addr == NULL)                                       \
        {                                                       \
            OMEGA_LOGE("Bluetooth address was NULL");           \
            return;                                             \
        }                                                       \
        if (str == NULL)                                        \
        {                                                       \
            OMEGA_LOGE("Provided buffer was NULL");             \
            return;                                             \
        }                                                       \
        snprintf(str, 18, MAC2STR_STRING, BUFFER_EXPAND(addr)); \
    } while (0)
#define OMEGA_STR2MAC(str, addr)                                                                 \
    do                                                                                           \
    {                                                                                            \
        if (str == NULL)                                                                         \
        {                                                                                        \
            OMEGA_LOGE("String buffer was NULL");                                                \
            return;                                                                              \
        }                                                                                        \
        if (addr == NULL)                                                                        \
        {                                                                                        \
            OMEGA_LOGE("Destination buffer was NULL");                                           \
            return;                                                                              \
        }                                                                                        \
        sscanf(str, STR2MAC_STRING, &addr[0], &addr[1], &addr[2], &addr[3], &addr[4], &addr[5]); \
    } while (0)

#if ESP_PLATFORM
#if CONFIG_SPIRAM
#define omega_malloc(size) heap_caps_malloc(size, MALLOC_CAP_SPIRAM)
#define omega_realloc(prev_ptr, new_size) heap_caps_realloc(prev_ptr, new_size, MALLOC_CAP_SPIRAM)
#define omega_free(ptr)      \
    do                       \
    {                        \
        heap_caps_free(ptr); \
        ptr = NULL;          \
    } while (0)
#else
#define omega_malloc(size) malloc(size)
#define omega_realloc(prev_ptr, new_size) realloc(prev_ptr, new_size)
#define omega_free(ptr) \
    do                  \
    {                   \
        free(ptr);      \
        ptr = NULL;     \
    } while (0)
#endif
#endif

    typedef uint8_t u8;
    typedef int8_t i8;
    typedef uint16_t u16;
    typedef int16_t i16;
    typedef uint64_t u64;
    typedef int64_t i64;

#define REGISTER_WRITE_DATA(name, ...) u8 name[] = {__VA_ARGS__}
#define REGISTER_READ_DATA(name, size) u8 name[size] = {0}

typedef enum
{
    eSUCCESS,
    eFAILED,
    eTIMEOUT,
    eNOT_INITED,
    eALREADY_INITED,
    eNOT_STARTED,
    eALREADY_STARTED,
}OmegaStatus;

#define OMEGA_GPIO_PORT_NC NULL
#define OMEGA_GPIO_PIN_NC -1
#define OMEGA_GPIO_NC ((OmegaGPIO_t){.port = NULL,.pin = -1})
typedef struct
{
    void* port;
    int32_t pin;
}OmegaGPIO_t;

#ifdef __cplusplus
}
#endif

#endif
