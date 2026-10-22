/**
  ******************************************************************************
  * @file    debug.c
  * @brief   Console UART (USART3, external USB-TTL) - implementation.
  *
  * Owns the USART3 hardware: full-duplex 115200 8N1 (TX = PB10, RX = PB11),
  * the RXNE interrupt and the NVIC setup. The RX interrupt demultiplexes the
  * incoming byte stream: a 0xAA frame head routes into the host frame parser
  * (protocol.c), any other byte routes into the interactive shell (shell.c).
  *
  * Note: Debug_SendByte() blocks on TXE. In a high-rate ISR this would stall
  * the interrupt; acceptable for debugging, but the debug calls should be
  * removed or gated by DEBUG_ENABLE=0 in production builds.
  ******************************************************************************
  */

#include "debug.h"
#include "protocol.h"
#include "shell.h"

/* 1 while a binary host frame is being received, 0 while in text mode. */
static volatile uint8_t d_debug_binary = 0u;

/* --------------------------------------------------------------------------
 * USART3 RX interrupt: demultiplex the incoming byte stream.
 *   0xAA        -> binary host frame (protocol.c state machine)
 *   any other   -> interactive shell text (shell.c ring buffer)
 * ------------------------------------------------------------------------ */
void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(DEBUG_USART, USART_IT_RXNE) != RESET)
    {
        uint8_t b = (uint8_t)USART_ReceiveData(DEBUG_USART);

        if (b == PROTO_FRAME_HEAD1)
        {
            d_debug_binary = 1u;
            Protocol_RxByte(b);
        }
        else if (d_debug_binary != 0u)
        {
            Protocol_RxByte(b);
            if (Protocol_InFrame() == 0u)
            {
                d_debug_binary = 0u;
            }
        }
        else
        {
            Shell_RxByte(b);
        }
    }
}

/* --------------------------------------------------------------------------
 * Public: initialize the console UART (USART3 TX = PB10 / RX = PB11, 115200).
 * ------------------------------------------------------------------------ */
void Debug_Init(void)
{
    GPIO_InitTypeDef  gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef  nvic;

    /* USART3 is on APB1, GPIOB is on APB2. */
    RCC_APB2PeriphClockCmd(DEBUG_TX_GPIO_CLK | DEBUG_RX_GPIO_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(DEBUG_USART_CLK, ENABLE);

    /* TX pin as AF push-pull (USART3_TX = PB10). */
    gpio.GPIO_Pin   = DEBUG_TX_PIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(DEBUG_TX_GPIO, &gpio);

    /* RX pin as floating input (USART3_RX = PB11, external USB-TTL). */
    gpio.GPIO_Pin   = DEBUG_RX_PIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(DEBUG_RX_GPIO, &gpio);

    usart.USART_BaudRate            = 115200u;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(DEBUG_USART, &usart);
    USART_Cmd(DEBUG_USART, ENABLE);

    /* RX-not-empty interrupt: demultiplex into protocol parser / shell. */
    USART_ITConfig(DEBUG_USART, USART_IT_RXNE, ENABLE);

    /* NVIC: group 2, preemption priority 1. */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    nvic.NVIC_IRQChannel                   = USART3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    d_debug_binary = 0u;
}

/* --------------------------------------------------------------------------
 * Public: send one byte, blocking until the TX register is empty.
 * ------------------------------------------------------------------------ */
void Debug_SendByte(uint8_t b)
{
    while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET)
    {
    }
    USART_SendData(DEBUG_USART, b);
}

/* --------------------------------------------------------------------------
 * Public: send a NUL-terminated string.
 * ------------------------------------------------------------------------ */
void Debug_SendString(const char *str)
{
    while (*str != '\0')
    {
        Debug_SendByte((uint8_t)(*str));
        str++;
    }
}

/* --------------------------------------------------------------------------
 * Internal: convert a 4-bit value to an uppercase hex ASCII character.
 * ------------------------------------------------------------------------ */
static char Debug_NibbleToHex(uint8_t nibble)
{
    nibble &= 0x0Fu;
    return (nibble < 10u) ? (char)('0' + nibble) : (char)('A' + nibble - 10u);
}

/* --------------------------------------------------------------------------
 * Public: send one byte as two hex characters ("XX").
 * ------------------------------------------------------------------------ */
void Debug_SendHex8(uint8_t v)
{
    Debug_SendByte((uint8_t)Debug_NibbleToHex((uint8_t)(v >> 4)));
    Debug_SendByte((uint8_t)Debug_NibbleToHex((uint8_t)(v & 0x0Fu)));
}

/* --------------------------------------------------------------------------
 * Public: send two bytes as four hex characters ("XXXX").
 * ------------------------------------------------------------------------ */
void Debug_SendHex16(uint16_t v)
{
    Debug_SendHex8((uint8_t)(v >> 8));
    Debug_SendHex8((uint8_t)(v & 0xFFu));
}

/* --------------------------------------------------------------------------
 * Public: send an unsigned 16-bit value as decimal ASCII ("0".."65535").
 * ------------------------------------------------------------------------ */
void Debug_SendDecU16(uint16_t v)
{
    char tmp[5];
    uint8_t len = 0u;

    if (v == 0u)
    {
        Debug_SendByte('0');
        return;
    }

    while (v > 0u)
    {
        tmp[len] = (char)('0' + (v % 10u));
        len++;
        v /= 10u;
    }

    while (len > 0u)
    {
        len--;
        Debug_SendByte((uint8_t)tmp[len]);
    }
}
