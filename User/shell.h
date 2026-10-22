/**
  ******************************************************************************
  * @file    shell.h
  * @brief   Interactive debug command line over the console UART (USART3).
  *
  * Lets the host type ASCII commands into the external USB-TTL console UART
  * (USART3 RX = PB11) and receive the parsed result + program output back on
  * the same UART (USART3 TX = PB10). RX bytes are latched into a ring buffer
  * by the console UART interrupt (debug.c); Shell_Poll() parses whole lines
  * and executes commands in the main loop so DMX512 timing stays deterministic.
  * The console ISR only forwards non-0xAA (text) bytes here.
  ******************************************************************************
  */

#ifndef __SHELL_H
#define __SHELL_H

#include "stm32f10x.h"

void Shell_Init(void);               /* Reset buffers and print the banner   */
void Shell_RxByte(uint8_t byte);     /* Push one RX byte (ISR context)        */
void Shell_Poll(void);               /* Parse a line / advance test animation */

#endif /* __SHELL_H */
