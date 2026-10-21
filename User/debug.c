/**
  ******************************************************************************
  * @file    debug.c
  * @brief   Lightweight debug output implementation (USART3 TX).
  *
  * Note: Debug_SendByte() blocks on TXE. In a high-rate ISR this would stall
  * the interrupt; acceptable for debugging, but the debug calls should be
  * removed or gated by DEBUG_ENABLE=0 in production builds.
  ******************************************************************************
  */

#include "debug.h"

/* --------------------------------------------------------------------------
 * Public: initialize the debug UART (USART3 TX, 115200 8N1).
 * ------------------------------------------------------------------------ */
void Debug_Init(void)
{
    GPIO_InitTypeDef  gpio;
    USART_InitTypeDef usart;

    RCC_APB2PeriphClockCmd(DEBUG_TX_GPIO_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(DEBUG_USART_CLK, ENABLE);

    gpio.GPIO_Pin   = DEBUG_TX_PIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(DEBUG_TX_GPIO, &gpio);

    usart.USART_BaudRate            = 115200u;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_Mode                = USART_Mode_Tx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(DEBUG_USART, &usart);
    USART_Cmd(DEBUG_USART, ENABLE);
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
