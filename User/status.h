/**
  ******************************************************************************
  * @file    status.h
  * @brief   System status indication via the on-board RGB LED.
  *
  * The RGB LED (R=PB5, G=PB0, B=PB1, active-low) is used to report system
  * state without blocking the main loop. Status_Set() only records the state;
  * Status_Refresh() applies it to the LED, so it is safe to call Status_Set()
  * from an interrupt and Status_Refresh() from the main loop.
  ******************************************************************************
  */

#ifndef __STATUS_H
#define __STATUS_H

#include "stm32f10x.h"

/* ============================ Status values ============================ */
typedef enum
{
    STATUS_BOOT      = 0,   /* White : power-on / initializing        */
    STATUS_RUNNING   = 1,   /* Green : running, no new frame          */
    STATUS_SINGLE    = 2,   /* Blue  : a single-segment frame arrived */
    STATUS_BROADCAST = 3,   /* Cyan  : a broadcast frame arrived      */
    STATUS_ERROR     = 4    /* Red   : invalid frame / parse error    */
} SystemStatus_t;

/* ============================ Public interface ============================ */
void Status_Init(void);                    /* Init LED GPIO, show boot state */
void Status_Set(SystemStatus_t s);         /* Record a new state (ISR-safe)  */
void Status_Refresh(void);                 /* Apply current state to the LED */

#endif /* __STATUS_H */
