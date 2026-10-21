/**
  ******************************************************************************
  * @file    protocol.c
  * @brief   Private frame protocol parser - implementation
  *
  * Receives the 7-byte host frame on USART1 (115200, 8N1) and drives a
  * 3-state machine inside the RX interrupt. A fully received frame is written
  * directly to the DMX512 output buffer via DMX512_SetSegment().
  ******************************************************************************
  */

#include "protocol.h"
#include "dmx512.h"
#include "debug.h"
#include "status.h"

/* The USART1 hardware (GPIO, UART, NVIC, RX interrupt) is owned by the console
 * module (debug.c). This file only parses the byte stream, so no UART macros
 * are needed here. */

/* ============================ Parser state ============================ */
static ProtocolState_t state = STATE_IDLE;
static uint8_t payload_index = 0u;
static uint8_t payload[PROTO_PAYLOAD_LEN];   /* [0]=segment, [1]=R, [2]=G, [3]=B, [4]=W */

/* Result cache shared between the RX ISR (writer) and Protocol_Poll (reader).
 * The ISR only latches data here so it stays light; printing / DMX write /
 * LED update happen in the main loop to keep DMX timing deterministic. */
static volatile uint8_t rx_ready = 0u;         /* 1 = a full frame is cached  */
static volatile uint8_t rx_segment;
static volatile uint8_t rx_r, rx_g, rx_b, rx_w;

static volatile uint8_t rx_error_ready = 0u;   /* 1 = an invalid byte cached  */
static volatile uint8_t rx_error_byte;

/* --------------------------------------------------------------------------
 * Public: initialize uplink UART, NVIC and reset the state machine.
 * ------------------------------------------------------------------------ */
void Protocol_Init(void)
{
    /* USART1 hardware (GPIO, UART, NVIC, RX interrupt) is owned by the
     * console module (Debug_Init in debug.c). Here we only reset the parser
     * state so a stale partial frame from a previous run is discarded. */
    state          = STATE_IDLE;
    payload_index  = 0u;
    rx_ready       = 0u;
    rx_error_ready = 0u;

    DBG_STR("Protocol Init OK\r\n");
}

/* --------------------------------------------------------------------------
 * Public: feed one received byte into the state machine.
 * Must be called from the UART RX interrupt context.
 * ------------------------------------------------------------------------ */
void Protocol_RxByte(uint8_t byte)
{
    switch (state)
    {
        case STATE_IDLE:
            /* Wait for the first frame head byte. */
            if (byte == PROTO_FRAME_HEAD1)
            {
                state = STATE_HEAD1;
            }
            break;

        case STATE_HEAD1:
            if (byte == PROTO_FRAME_HEAD2)
            {
                /* Frame head verified, start payload. */
                state         = STATE_PLAYLOAD;
                payload_index = 0u;
            }
            else if (byte == PROTO_FRAME_HEAD1)
            {
                /* Tolerate a repeated 0xAA (line glitch); stay in HEAD1. */
            }
            else
            {
                /* Unexpected byte -> latch error, back to IDLE. */
                rx_error_byte  = byte;
                rx_error_ready = 1u;
                state = STATE_IDLE;
            }
            break;

        case STATE_PLAYLOAD:
            payload[payload_index] = byte;
            payload_index++;
            if (payload_index >= PROTO_PAYLOAD_LEN)
            {
                /* Frame complete: latch the result, leave ISR as fast as
                 * possible. The main-loop poll applies it to DMX + LED. */
                rx_segment = payload[0];
                rx_r       = payload[1];
                rx_g       = payload[2];
                rx_b       = payload[3];
                rx_w       = payload[4];
                rx_ready   = 1u;
                state = STATE_IDLE;
            }
            break;

        default:
            state = STATE_IDLE;
            break;
    }
}

/* --------------------------------------------------------------------------
 * Public: return 1 while the state machine is mid-frame (state != IDLE).
 * Used by the console UART ISR to decide between binary-frame and shell modes.
 * ------------------------------------------------------------------------ */
uint8_t Protocol_InFrame(void)
{
    return (state != STATE_IDLE) ? 1u : 0u;
}

/* --------------------------------------------------------------------------
 * Public: main-loop poll. Drains the cached RX result and prints it over the
 * debug UART, then updates the status LED. Never call from the ISR.
 * ------------------------------------------------------------------------ */
void Protocol_Poll(void)
{
    /* Error takes priority over a normal frame. */
    if (rx_error_ready != 0u)
    {
        rx_error_ready = 0u;
        DBG_STR("ERR: bad byte 0x");
        DBG_HEX8(rx_error_byte);
        DBG_STR(" in HEAD1, reset\r\n");
        Status_Set(STATUS_ERROR);
    }
    else if (rx_ready != 0u)
    {
        uint16_t ch;

        rx_ready = 0u;

        /* Apply the parsed frame to the DMX512 output buffer. */
        DMX512_SetSegment(rx_segment, rx_r, rx_g, rx_b, rx_w);

        /* Print the program output over the debug UART. */
        DBG_STR("RX seg=");
        DBG_HEX8(rx_segment);
        DBG_STR(" R="); DBG_HEX8(rx_r);
        DBG_STR(" G="); DBG_HEX8(rx_g);
        DBG_STR(" B="); DBG_HEX8(rx_b);
        DBG_STR(" W="); DBG_HEX8(rx_w);

        if (rx_segment == 0u)
        {
            DBG_STR(" -> broadcast all segments\r\n");
            Status_Set(STATUS_BROADCAST);
        }
        else
        {
            ch = (uint16_t)(rx_segment - 1u) * DMX512_SEGMENT_CHANNEL + 1u;
            DBG_STR(" -> DMX start ch=");
            DBG_HEX16(ch);
            DBG_STR("\r\n");
            Status_Set(STATUS_SINGLE);
        }
    }

    /* Always refresh the LED so the latest state is visible. */
    Status_Refresh();
}
