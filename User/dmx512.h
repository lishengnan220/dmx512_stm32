/**
  ******************************************************************************
  * @file    dmx512.h
  * @brief   DMX512 driver layer - public interface
  *
  * Implements DMX512 output:
  *   - Break / MAB timing generation (hardware timer, microsecond accuracy)
  *   - 250 kbps, 8 data bits + 2 stop bits serial transmission
  *   - channel mapping and output buffer management
  *
  * Hardware mapping (centralized macros, adjust to match your PCB):
  *   - DMX TX UART   : USART2  (TX = PA2)
  *   - RS485 DE/RE   : PA1     (DE/RE tied together, high = TX, low = RX)
  *   - Delay timer   : TIM4    (72 MHz, 1 us per tick)
  ******************************************************************************
  */

#ifndef __DMX512_H
#define __DMX512_H

#include "stm32f10x.h"

/* ============================ Basic constants ============================ */
#define DMX512_CHANNEL_NUM       512u    /* Max channels on a single DMX512 link */
#define DMX512_SEGMENT_CHANNEL   4u      /* Channels per segment (RGBW) */
#define DMX512_SEGMENT_COUNT     20u     /* Actual number of light segments */
#define DMX512_START_CODE        0x00u   /* DMX512 start code */

/* ============================ Hardware mapping ============================ */
/* --- DMX TX UART (USART2, TX = PA2) --- */
#define DMX512_USART             USART2
#define DMX512_USART_CLK         RCC_APB1Periph_USART2
#define DMX512_TX_GPIO           GPIOA
#define DMX512_TX_GPIO_CLK       RCC_APB2Periph_GPIOA
#define DMX512_TX_PIN            GPIO_Pin_2

/* --- RS485 direction control (PA1, DE/RE tied together) --- */
#define DMX512_DIR_GPIO          GPIOA
#define DMX512_DIR_GPIO_CLK      RCC_APB2Periph_GPIOA
#define DMX512_DIR_PIN           GPIO_Pin_1

/* --- Delay timer (TIM4) --- */
#define DMX512_TIM               TIM4
#define DMX512_TIM_CLK           RCC_APB1Periph_TIM4
#define DMX512_TIM_PSC           (72u - 1u)   /* 72 MHz -> 1 us per tick */

/* ============================ Timing parameters ============================ */
#define DMX512_BREAK_US          100u   /* Break low level duration (>= 92 us) */
#define DMX512_MAB_US            12u    /* Mark-After-Break high level (>= 12 us) */

/* ============================ Public interface ============================ */
void    DMX512_Init(void);
void    DMX512_SetSegment(uint8_t segment,
                          uint8_t r, uint8_t g, uint8_t b, uint8_t w);
void    DMX512_SetChannel(uint16_t ch, uint8_t value);
void    DMX512_SendFrame(void);
uint8_t DMX512_IsBusy(void);

#endif /* __DMX512_H */
