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

/* ============================ Uplink UART mapping ============================
 * Host link: USART1, RX = PA10, 115200 8N1, receive only.
 * Adjust these macros if the host UART uses different pins.
 * =========================================================================== */
#define PROTO_USART         USART1
#define PROTO_USART_CLK     RCC_APB2Periph_USART1
#define PROTO_RX_GPIO       GPIOA
#define PROTO_RX_GPIO_CLK   RCC_APB2Periph_GPIOA
#define PROTO_RX_PIN        GPIO_Pin_10

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
    GPIO_InitTypeDef  gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef  nvic;

    /* 1. Enable clocks: GPIOA + USART1 (both on APB2). */
    RCC_APB2PeriphClockCmd(PROTO_RX_GPIO_CLK | PROTO_USART_CLK, ENABLE);

    /* 2. RX pin as floating input. */
    gpio.GPIO_Pin   = PROTO_RX_PIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(PROTO_RX_GPIO, &gpio);

    /* 3. UART: 115200, 8 data bits, 1 stop bit, no parity, RX only. */
    usart.USART_BaudRate            = 115200u;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_Mode                = USART_Mode_Rx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(PROTO_USART, &usart);

    /* 4. Enable RX-not-empty interrupt. */
    USART_ITConfig(PROTO_USART, USART_IT_RXNE, ENABLE);

    /* 5. NVIC: group 2, preemption priority 1. */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    nvic.NVIC_IRQChannel                   = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    /* 6. Enable UART and reset the state machine. */
    USART_Cmd(PROTO_USART, ENABLE);
    state         = STATE_IDLE;
    payload_index = 0u;

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
 * UART1 receive interrupt handler.
 * ------------------------------------------------------------------------ */
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(PROTO_USART, USART_IT_RXNE) != RESET)
    {
        Protocol_RxByte((uint8_t)USART_ReceiveData(PROTO_USART));
    }
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
