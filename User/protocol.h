/**
  ******************************************************************************
  * @file    protocol.h
  * @brief   Private frame protocol parser - public interface
  *
  * Parses a fixed 7-byte frame from the host over UART:
  *   [0] 0xAA  Frame head 1
  *   [1] 0x55  Frame head 2
  *   [2] Segment (0x00 = broadcast all segments)
  *   [3] Red
  *   [4] Green
  *   [5] Blue
  *   [6] White
  *
  * Byte-by-byte parsing is done inside the UART RX interrupt (no blocking,
  * no queue, no heap). A 3-state machine filters noise and tolerates a
  * repeated frame-head byte.
  ******************************************************************************
  */

#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include "stm32f10x.h"

/* ============================ Frame constants ============================ */
#define PROTO_FRAME_HEAD1   0xAAu   /* Frame head first byte */
#define PROTO_FRAME_HEAD2   0x55u   /* Frame head second byte */
#define PROTO_FRAME_LEN     7u      /* Total frame length (2 head + 5 payload) */
#define PROTO_PAYLOAD_LEN   5u      /* Segment + RGBW */

/* ============================ State machine ============================ */
typedef enum
{
    STATE_IDLE     = 0,   /* Waiting for the first frame head byte */
    STATE_HEAD1    = 1,   /* 0xAA received, waiting for 0x55 */
    STATE_PLAYLOAD = 2    /* Head verified, receiving 5 payload bytes */
} ProtocolState_t;

/* ============================ Public interface ============================ */
void Protocol_Init(void);
void Protocol_RxByte(uint8_t byte);

/* Main-loop poll: drains the RX result cache, writes DMX512, prints the
 * parsed frame over the debug UART and updates the status LED. Call this
 * repeatedly from the main loop (never from the ISR). */
void Protocol_Poll(void);

#endif /* __PROTOCOL_H */
