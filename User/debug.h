/**
  ******************************************************************************
  * @file    debug.h
  * @brief   Lightweight debug output over a dedicated UART.
  *
  * Self-contained ASCII output (no printf / no C-library retargeting needed),
  * so it compiles cleanly under Keil AC5 regardless of MicroLIB settings.
  *
 * Hardware mapping: USART1 (on-board CH340 USB-UART), TX = PA9, RX = PA10,
 * 115200 8N1. This is the SAME UART shared by the host frame protocol and the
 * interactive shell. Debug_Init() owns the USART1 hardware and RX interrupt.
  ******************************************************************************
  */

#ifndef __DEBUG_H
#define __DEBUG_H

#include "stm32f10x.h"

/* Master debug switch: set to 0 to strip all debug output from the build. */
#define DEBUG_ENABLE  1u

/* Console UART mapping (USART1, TX = PA9, RX = PA10, on-board CH340). */
#define DEBUG_USART         USART1
#define DEBUG_USART_CLK     RCC_APB2Periph_USART1
#define DEBUG_TX_GPIO       GPIOA
#define DEBUG_TX_GPIO_CLK   RCC_APB2Periph_GPIOA
#define DEBUG_TX_PIN        GPIO_Pin_9
#define DEBUG_RX_GPIO       GPIOA
#define DEBUG_RX_GPIO_CLK   RCC_APB2Periph_GPIOA
#define DEBUG_RX_PIN        GPIO_Pin_10

void Debug_Init(void);
void Debug_SendByte(uint8_t b);
void Debug_SendString(const char *str);
void Debug_SendHex8(uint8_t v);
void Debug_SendHex16(uint16_t v);
void Debug_SendDecU16(uint16_t v);

/* Convenience macros that compile to nothing when debug is disabled. */
#if DEBUG_ENABLE
    #define DBG_STR(s)      Debug_SendString((s))
    #define DBG_BYTE(b)     Debug_SendByte((uint8_t)(b))
    #define DBG_HEX8(v)     Debug_SendHex8((v))
    #define DBG_HEX16(v)    Debug_SendHex16((v))
#else
    #define DBG_STR(s)      ((void)0)
    #define DBG_BYTE(b)     ((void)0)
    #define DBG_HEX8(v)     ((void)0)
    #define DBG_HEX16(v)    ((void)0)
#endif

#endif /* __DEBUG_H */
