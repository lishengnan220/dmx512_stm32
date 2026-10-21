/**
  ******************************************************************************
  * @file    status.c
  * @brief   System status indication via the on-board RGB LED.
  *
  * Reuses the bsp_led driver (LED1=R/PB5, LED2=G/PB0, LED3=B/PB1, active-low).
  ******************************************************************************
  */

#include "status.h"
#include "bsp_led.h"

/* Current status (defaults to BOOT). */
static SystemStatus_t current = STATUS_BOOT;

/* --------------------------------------------------------------------------
 * Internal: drive the RGB LED. Parameters are active-low: 0 = on, 1 = off.
 * ------------------------------------------------------------------------ */
static void LedRGB(uint8_t r, uint8_t g, uint8_t b)
{
    LED1(r);
    LED2(g);
    LED3(b);
}

/* --------------------------------------------------------------------------
 * Public: initialize the LED and show the boot (white) state.
 * ------------------------------------------------------------------------ */
void Status_Init(void)
{
    LED_GPIO_Config();
    current = STATUS_BOOT;
    Status_Refresh();
}

/* --------------------------------------------------------------------------
 * Public: record a new state. Safe to call from an interrupt.
 * ------------------------------------------------------------------------ */
void Status_Set(SystemStatus_t s)
{
    current = s;
}

/* --------------------------------------------------------------------------
 * Public: read the current state.
 * ------------------------------------------------------------------------ */
SystemStatus_t Status_Get(void)
{
    return current;
}

/* --------------------------------------------------------------------------
 * Public: apply the current state to the RGB LED.
 * ------------------------------------------------------------------------ */
void Status_Refresh(void)
{
    switch (current)
    {
        case STATUS_BOOT:      LedRGB(0, 0, 0); break;   /* White */
        case STATUS_RUNNING:   LedRGB(1, 0, 1); break;   /* Green */
        case STATUS_SINGLE:    LedRGB(1, 1, 0); break;   /* Blue  */
        case STATUS_BROADCAST: LedRGB(1, 0, 0); break;   /* Cyan  */
        case STATUS_ERROR:     LedRGB(0, 1, 1); break;   /* Red   */
        default:               LedRGB(1, 1, 1); break;   /* Off   */
    }
}
