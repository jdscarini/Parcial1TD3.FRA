#include "board.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "placa_roja.h"
#include "lcd1602_i2c.h"
#include "stdlib.h"
#include <stdio.h>

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/
#define LCD_TASK_PRIORITY		            ( tskIDLE_PRIORITY + 1 )
#define PULSOS_TASK_PRIORITY		            ( tskIDLE_PRIORITY + 2 )
#define timeLedON 400
#define timeLedOFF 600

#define I2C_SDA_PIN 10
#define I2C_SDA_PORT 0
#define I2C_SCL_PIN 11
#define I2C_SCL_PORT 0

#define GIRO_IZQ_PORT	2
#define GIRO_IZQ_PIN	1
#define GIRO_IZQ		2,1
#define GIRO_DER_PORT	2
#define GIRO_DER_PIN	2
#define GIRO_DER		2,2

#define pulsosPorVuelta	10
#define timeMaxQueuePulsos	100 //Lo mínimo que puedo medir son 60 RPM
#define muestrasPromedio	20
//Agregar define i2c i2c3

typedef struct LCD_t {
	char *mensaje;
	unsigned int linea;
	unsigned int posicion;
} LCD_Struct;

xQueueHandle xQueueLCD;
xQueueHandle xQueueCapture;
xSemaphoreHandle xSemaphorePulso;
xTaskHandle xHandleLedOn, xHandleLedOff;
/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Sets up system hardware */
static void prvSetupHardware(void) {
	SystemCoreClockUpdate();
	Board_Init();

	/* Initial LED0 state is off */
	Board_LED_Set(0, true);
	Board_LED_Set(1, true);
	Board_LED_Set(2, true);

	Chip_GPIO_SetPinDIRInput(LPC_GPIO, PULS_EXT_1);
}

/* LED1 toggle thread */
static void vGiro_Task(void *pvParameters) {
	unsigned int clockPwm = 0;
	unsigned int asereje = 10;
	char mensaje[16] = {' '};
	LCD_Struct dataLCD;
	char *msg = "Sofi te amo";
	dataLCD.mensaje = msg;
	dataLCD.linea = 1;
	dataLCD.posicion = 0;
	uint16_t adcValue=2000, lastPwm = 0;
	static ADC_CLOCK_SETUP_T ADCSetup;
	bool sentidoDeGiro = 0;
	portTickType xLastWakeTime;

	xQueueSend(xQueueLCD, &dataLCD, 10);

	clockPwm = Chip_Clock_GetSystemClockRate()/800; //Divido para obtener un pwm a 200hz, frec ideal para motor.
	asereje = clockPwm/4;



	Chip_ADC_Init(LPC_ADC, &ADCSetup);
	Chip_IOCON_PinMuxSet(LPC_IOCON, ADC_PORT, ADC_PIN, 3);
	Chip_ADC_EnableChannel(LPC_ADC, ADC_CHANNEL, ENABLE);
	Chip_ADC_SetBurstCmd(LPC_ADC, ENABLE);

	while (Chip_ADC_ReadStatus(LPC_ADC, ADC_CHANNEL, ADC_DR_DONE_STAT) != SET) {
	}
	Chip_ADC_ReadValue(LPC_ADC, ADC_CHANNEL, &adcValue);

	Chip_GPIO_SetDir(LPC_GPIO, GIRO_IZQ, 1);
	Chip_GPIO_SetDir(LPC_GPIO, GIRO_DER, 1);
	Chip_GPIO_SetPinState(LPC_GPIO, GIRO_DER, !sentidoDeGiro);
	Chip_GPIO_SetPinState(LPC_GPIO, GIRO_IZQ, sentidoDeGiro);

	Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 0, IOCON_FUNC1);
	//Configuro PWM a 200hz y que arranque en el 25%.
	LPC_PWM1->TCR = (1 << 0) | (1 << 2);
	LPC_PWM1->PR = 0x0; /* No Prescalar */
	LPC_PWM1->MCR = (1 << 1); /*Reset on PWMMR0, reset TC if it matches MR0 */
	LPC_PWM1->MR0 = clockPwm; /* set PWM cycle(Ton+Toff)=100) */
	LPC_PWM1->MR1 = clockPwm / 4; /* Set % Duty Cycle for all four channels */
	/* Trigger the latch Enable Bits to load the new Match Values */
	LPC_PWM1->LER = (1 << 0) | (1 << 1); // | (1 << 5) | (1 << 6);
	/* Enable the PWM output pins for PWM_1-PWM_4(P2_0 - P2_3) */
	LPC_PWM1->PCR = (1 << 9); // | (1 << 13) | (1 << 14);


	while (1) {
		while (Chip_ADC_ReadStatus(LPC_ADC, ADC_CHANNEL, ADC_DR_DONE_STAT)
				!= SET) {
		}
		Chip_ADC_ReadValue(LPC_ADC, ADC_CHANNEL, &adcValue);


		if (lastPwm != adcValue * 100 / 4096) {
			/*comparo si vario el PWM + de 1%*/
			lastPwm = adcValue * 100 / 4096;
			LPC_PWM1->MR1 = adcValue * clockPwm / 4096;
			LPC_PWM1->LER = (1 << 0) | (1 << 1);
			sprintf(mensaje, "PWM:%d     ", lastPwm);
			dataLCD.mensaje = mensaje;
			xQueueSend(xQueueLCD, &dataLCD, 10);
		}


		asereje += 1;
		if (asereje > 50) {
			asereje = 0;
			sentidoDeGiro = !sentidoDeGiro;
			Chip_GPIO_SetPinState(LPC_GPIO, GIRO_DER, !sentidoDeGiro);
			Chip_GPIO_SetPinState(LPC_GPIO, GIRO_IZQ, sentidoDeGiro);
		}


		vTaskDelayUntil(&xLastWakeTime,100);
		xLastWakeTime = xTaskGetTickCount();
	}

}

/* LED2 toggle thread */
static void vPulsos_Task(void *pvParameters) {
	unsigned int captura;
	unsigned int rpm[muestrasPromedio]  = { 0 }; //Guardo un array por si necesito hacer promedio.
	unsigned int frecVuelta = 0;
	//unsigned int capturita[muestrasPromedio]  = { 0 };
	unsigned int i = 0, ii= 0, aux = 0;
	LCD_Struct datosLcd;
	uint32_t timerFreq;
	char mensaje[16] = {' '};

	Chip_Clock_SetPCLKDiv(SYSCTL_PCLK_TIMER3, SYSCTL_CLKDIV_1);
	Chip_TIMER_Init(LPC_TIMER3);
	timerFreq = Chip_Clock_GetSystemClockRate();

	Chip_TIMER_CaptureEnableInt(LPC_TIMER3, 1);
	Chip_TIMER_CaptureFallingEdgeEnable(LPC_TIMER3, 1);

	Chip_TIMER_PrescaleSet(LPC_TIMER3, 0);

	Chip_TIMER_Reset(LPC_TIMER3);
	Chip_TIMER_Enable(LPC_TIMER3);

	NVIC_ClearPendingIRQ(TIMER3_IRQn);
	NVIC_EnableIRQ(TIMER3_IRQn);

	Chip_IOCON_PinMux(LPC_IOCON, 0, 24, IOCON_MODE_PULLUP, IOCON_FUNC3);

	while (1) {
		if (xQueueReceive(xQueueCapture, &captura, timeMaxQueuePulsos) == pdPASS) //Si recibe pulsos antes del tiempo estamos a más de 60 RPM.
		{
			//capturita[i] = captura;
			frecVuelta = timerFreq / captura;
			rpm[i] = frecVuelta * 60 / (12);
		} else {
			rpm[i] = 0;
			captura = 0;
			//debería resetear el timer si no llega el pulso? Probemos.
		}

		datosLcd.linea = 0;
		datosLcd.posicion = 0;
		aux = 0;
		for (ii = 0; ii < muestrasPromedio; ii++) {
			aux += rpm[ii];
		}
		aux = aux / muestrasPromedio;

		i++;
		if (i >= muestrasPromedio) {
			i = 0;
			sprintf(mensaje, "RPM:%d     ", aux);
			datosLcd.mensaje = mensaje;
			xQueueSend(xQueueLCD, &datosLcd, 0);

		}

	}
}

static void vLCD_Task(void *pvParameters) {
	//i2c init
	I2C_ID_T id = I2C2;
	LCD_Struct datosLcd;
	//static I2C_XFER_T seep_xfer;



	Board_I2C_Init(id);
	Chip_I2C_Init(id);
	Chip_I2C_SetClockRate(id, 100000);
	NVIC_DisableIRQ(I2C2_IRQn);
	Chip_I2C_SetMasterEventHandler(id, Chip_I2C_EventHandlerPolling);

	Chip_IOCON_PinMuxSet(LPC_IOCON, I2C_SDA_PORT, I2C_SDA_PIN, 2);
	Chip_IOCON_PinMuxSet(LPC_IOCON, I2C_SCL_PORT, I2C_SCL_PIN, 2);

	//vTaskDelay(300); // Le doy tiempo a la lógica del LCD para iniciar

	lcd_init(id);
	lcd_clear();
	//lcd_set_cursor(0,0);
	//lcd_char('a');

	while (1) {
		xQueueReceive(xQueueLCD, &datosLcd, portMAX_DELAY);
		lcd_set_cursor(datosLcd.linea, datosLcd.posicion);
		lcd_string(datosLcd.mensaje);
		//lcd_toggle_enable(0x05);
		//vTaskDelay(100);
	}

}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	main routine for FreeRTOS blinky example
 * @return	Nothing, function should not exit
 */
int main(void) {

	prvSetupHardware();

	xTaskCreate(vGiro_Task, ( char* )"vMensaje_Task",configMINIMAL_STACK_SIZE, NULL, PULSOS_TASK_PRIORITY, &xHandleLedOn);
	xTaskCreate(vPulsos_Task, ( char* )"vPulsos_Task",configMINIMAL_STACK_SIZE, NULL, PULSOS_TASK_PRIORITY,&xHandleLedOff);

	xTaskCreate(vLCD_Task, ( char* )"vLCD_Task", configMINIMAL_STACK_SIZE,NULL, LCD_TASK_PRIORITY, NULL);

	/* UART output thread, simply counts seconds */
	//xTaskCreate(vUARTTask, (signed char* ) "vTaskUart",configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),(xTaskHandle *) NULL);
	//xSemaphorePulso = xSemaphoreCreateCounting(5000, 1);
	xQueueLCD = xQueueCreate(2, sizeof(LCD_Struct));
	xQueueCapture = xQueueCreate(10, sizeof(unsigned int));

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
		//todo: pasar a  send from ISR.
		xQueueSend(xQueueCapture,&captura,0);
	}
}
/**
 * @}
 */
