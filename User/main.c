/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   ����led
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� F103-ָ���� STM32 ������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */ 
	
#include "stm32f10x.h"
#include "dmx512.h"
#include "protocol.h"
#include "debug.h"
#include "status.h"
#include "shell.h"

#define SOFT_DELAY Delay(0x0FFFFF);

void Delay(__IO u32 nCount); 

/**
  * @brief  ������
  * @param  ��  
  * @retval ��
  */
int main(void)
{
	/* 1. Console UART (on-board CH340, USART1 TX=PA9/RX=PA10, 115200 8N1).
	 *    Also sets up the RX interrupt that feeds both the host frame parser
	 *    and the interactive shell. */
	Debug_Init();
	DBG_STR("\r\n=== DMX512 Light Controller Boot ===\r\n");

	/* 2. Status LED (RGB): show white while booting. */
	Status_Init();

	/* 3. DMX512 output: USART2/PA2 (250kbps 8N2) + TIM4 + RS485 dir PA1. */
	DMX512_Init();

	/* 4. Host frame parser: resets the protocol state machine (USART1 HW is
	 *    already initialized by Debug_Init). */
	Protocol_Init();

	/* 5. Interactive shell: resets the command line buffer (shares USART1). */
	Shell_Init();

	while (1)
	{
		/* Continuously transmit DMX512 frames.
		 * One frame = Break(100us) + MAB(12us) + 513 bytes * 44us
		 *           ~ 22.7 ms  ->  ~ 44 Hz refresh rate. */
		DMX512_SendFrame();

		/* Drain parsed frames / errors: print + update status LED. */
		Protocol_Poll();

		/* Parse console commands and advance the self-test animation. */
		Shell_Poll();
	}
}

void Delay(__IO uint32_t nCount)	 //�򵥵���ʱ����
{
	for(; nCount != 0; nCount--);
}
/*********************************************END OF FILE**********************/
