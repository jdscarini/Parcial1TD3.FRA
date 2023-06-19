/*
 * @brief FreeRTOS Blinky example
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2014
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#include "board.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/
xQueueHandle xQueueCapture;
/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Sets up system hardware */
static void prvSetupHardware(void)
{
	SystemCoreClockUpdate();
	Board_Init();

	/* Initial LED0 state is off */
	Board_LED_Set(0, false);
	Chip_GPIO_WriteDirBit(LPC_GPIO, 3, 26, TRUE);
	Chip_GPIO_WriteDirBit(LPC_GPIO, 3, 25, TRUE);
	Chip_GPIO_WriteDirBit(LPC_GPIO, 2, 5, TRUE);

	Chip_GPIO_WritePortBit(LPC_GPIO, 3, 26, 1);
	Chip_GPIO_WritePortBit(LPC_GPIO, 3,25, 1);
	Chip_GPIO_WritePortBit(LPC_GPIO, 2, 5, 0);
}

/* LED1 toggle thread */
static void vLEDTask1(void *pvParameters) {
	bool LedState = false;

	while (1) {
		Board_LED_Set(0, LedState);
		//Chip_GPIO_WritePortBit(LPC_GPIO, 3, 26, LedState);
		LedState = (bool) !LedState;

		/* About a 3Hz on/off toggle rate */
		vTaskDelay(configTICK_RATE_HZ/7);
	}
}

/* LED2 toggle thread */
static void vLEDTask2(void *pvParameters) {
	bool LedState = false;

	while (1) {
		//Board_LED_Set(1, LedState);
		Chip_GPIO_WritePortBit(LPC_GPIO, 3, 26, LedState);
		LedState = (bool) !LedState;

		/* About a 7Hz on/off toggle rate */
		vTaskDelay(configTICK_RATE_HZ / 14);
	}
}

/* UART (or output) thread */
static void vUARTTask(void *pvParameters) {

	unsigned int captura,rpm[10];
	unsigned int i = 0;

	Chip_Clock_SetPCLKDiv(SYSCTL_PCLK_TIMER3,SYSCTL_CLKDIV_1);
	Chip_TIMER_Init(LPC_TIMER3);
	uint32_t timerFreq = Chip_Clock_GetSystemClockRate();

	Chip_TIMER_CaptureEnableInt(LPC_TIMER3, 1);
	Chip_TIMER_CaptureFallingEdgeEnable(LPC_TIMER3, 1);

	Chip_TIMER_PrescaleSet(LPC_TIMER3, 0);

	Chip_TIMER_Reset(LPC_TIMER3);
	Chip_TIMER_Enable(LPC_TIMER3);

	NVIC_ClearPendingIRQ(TIMER3_IRQn);
	NVIC_EnableIRQ(TIMER3_IRQn);

	Chip_IOCON_PinMux(LPC_IOCON, 0,24, IOCON_MODE_PULLUP, IOCON_FUNC3);
	while (1) {
		xQueueReceive(xQueueCapture, &captura, portMAX_DELAY);
		//rpm[i]=timerFreq/(60*captura); //asumo rueda dentada con 1 click por vuelta

		rpm[i] = captura;
		i++;
		if (i > 10)
			i = 0;

	}
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	main routine for FreeRTOS blinky example
 * @return	Nothing, function should not exit
 */
int main(void)
{
	prvSetupHardware();

	/* LED1 toggle thread */
	xTaskCreate(vLEDTask1, (char *) "vTaskLed1",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
				(xTaskHandle *) NULL);

	/* LED2 toggle thread */
	xTaskCreate(vLEDTask2, (char *) "vTaskLed2",configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),(xTaskHandle *) NULL);

	/* UART output thread, simply counts seconds */
	xTaskCreate(vUARTTask, (char *) "vTaskUart",configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 2UL),(xTaskHandle *) NULL);

	xQueueCapture = xQueueCreate(10,sizeof (unsigned int));

	/* Start the scheduler */
	vTaskStartScheduler();

	/* Should never arrive here */
	return 1;
}

void TIMER3_IRQHandler(void)
{
	unsigned int captura = 0;
	Chip_TIMER_Reset(LPC_TIMER3);
	if (Chip_TIMER_CapturePending(LPC_TIMER3, 1)) {
		captura = Chip_TIMER_ReadCapture(LPC_TIMER3,1);
		Chip_TIMER_ClearCapture(LPC_TIMER3, 1);

		NVIC_ClearPendingIRQ(TIMER3_IRQn);
		xQueueSend(xQueueCapture,&captura,0);
	}
}

/**
 * @}
 */
