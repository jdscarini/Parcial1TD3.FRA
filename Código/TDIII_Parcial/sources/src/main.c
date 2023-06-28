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

#define GIRO_FSM_S1		1
#define GIRO_FSM_S2		2
#define GIRO_FSM_S3		3
#define GIRO_FSM_S4		4

#define BOTON1		0,27
#define BOTON2		0,28
#define BOTON1_PORT	0
#define BOTON1_PIN	27
#define BOTON2_PORT	0
#define BOTON2_PIN	28


#define pulsosPorVuelta	10
#define timeMaxQueuePulsos	20 //Lo mínimo que puedo medir son xx RPM
#define muestrasPromedio	20

#define divisorRPM	10
//Agregar define i2c i2c3

typedef struct LCD_t {
	char *mensaje;
	unsigned int linea;
	unsigned int posicion;
} LCD_Struct;

xQueueHandle 	xQueueLCD;
xQueueHandle 	xQueueCapture;
xQueueHandle	xQueueGiro;
xQueueHandle	xQueueVelocidad;

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

unsigned int actualizarVelocidad (unsigned int actualValue, unsigned int lastValue,unsigned int clockPwm)
{
	unsigned int aux = 0;
	//char mensaje[16] = {' '};
	//bool	cambioVel = 0;

	if (actualValue > lastValue + 2 || actualValue < lastValue - 2 ) {
		/*comparo si vario el PWM + de 1%*/
		aux = actualValue * 100 / 4096;
		LPC_PWM1->MR1 = actualValue * clockPwm / 4096;
		LPC_PWM1->LER = (1 << 0) | (1 << 1);
		//cambioVel = 1;
		//xQueueSend(xQueueLCD, &dataLCD, 10);
	}
	return aux;
}

/* LED1 toggle thread */
static void vGiro_Task(void *pvParameters) {
	unsigned int clockPwm = 0;
	//unsigned int asereje = 10;
	unsigned int estadoActual = 1;
	char mensaje[16] = {' '};
	LCD_Struct dataLCD;
	char *msg = "Sofi te amo";
	dataLCD.mensaje = msg;
	dataLCD.linea = 1;
	dataLCD.posicion = 0;
	uint16_t adcValue=2000, lastAdc = 0;
	static ADC_CLOCK_SETUP_T ADCSetup;
	unsigned int sentidoDeGiro = 0;
	portTickType xLastWakeTime;
	unsigned int velocidadActual = 0;
	unsigned char solicitudGiroDer = 0, solicitudGiroIzq = 0;
	unsigned int velocidadResultante = 0;

	xQueueSend(xQueueLCD, &dataLCD, 10);

	clockPwm = Chip_Clock_GetSystemClockRate()/800; //Divido para obtener un pwm a 200hz, frec ideal para motor.

	Chip_ADC_Init(LPC_ADC, &ADCSetup);
	Chip_IOCON_PinMuxSet(LPC_IOCON, ADC_PORT, ADC_PIN, 3);
	Chip_ADC_EnableChannel(LPC_ADC, ADC_CHANNEL, ENABLE);
	Chip_ADC_SetBurstCmd(LPC_ADC, ENABLE);

	Chip_GPIO_SetDir(LPC_GPIO, GIRO_IZQ, 1);
	Chip_GPIO_SetDir(LPC_GPIO, GIRO_DER, 1);
	Chip_GPIO_SetPinState(LPC_GPIO, GIRO_DER, 0);
	Chip_GPIO_SetPinState(LPC_GPIO, GIRO_IZQ, 0);

	//Configuro PWM a 200hz y que arranque en el 25%.
	Chip_IOCON_PinMuxSet(LPC_IOCON, 2, 0, IOCON_FUNC1);
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
		lastAdc = adcValue;


		//recibo velocidad por cola.
		if(pdPASS != xQueueReceive(xQueueVelocidad,&velocidadActual,0))
		{
			//No pude actualizar velocidad, asumo que la velocidad no vario.
			velocidadActual = velocidadActual;
		}
		//recibo sentido de giro por cola.
		if (pdPASS == xQueueReceive(xQueueGiro, &sentidoDeGiro, 0)) {
			if (sentidoDeGiro == 1) {
				solicitudGiroDer = 1;
				solicitudGiroIzq = 0;
			} else if (sentidoDeGiro == 2) {
				solicitudGiroDer = 0;
				solicitudGiroIzq = 1;
			}
		}

		//actualizarVelocidad(adcValue,lastAdc,clockPwm);



		switch (estadoActual) {
		default:
		case GIRO_FSM_S1:
			if (solicitudGiroDer == 1) {
				if (velocidadActual == 0) {
					estadoActual = GIRO_FSM_S3;
					Chip_GPIO_SetPinState(LPC_GPIO, GIRO_DER, 1);
					Chip_GPIO_SetPinState(LPC_GPIO, GIRO_IZQ, 0);
				} else {
					estadoActual = GIRO_FSM_S2;
					adcValue = 0;
					velocidadResultante = actualizarVelocidad(adcValue, lastAdc,
							clockPwm);
					if (velocidadResultante != 0) {
						sprintf(mensaje, "PWM:%d     ", velocidadResultante);
						dataLCD.mensaje = mensaje;
						xQueueSend(xQueueLCD, &dataLCD, 10);
					}

					Chip_GPIO_SetPinState(LPC_GPIO, GIRO_DER, 0);
					Chip_GPIO_SetPinState(LPC_GPIO, GIRO_IZQ, 0);
				}
				solicitudGiroDer = 0;
			} else {
				estadoActual = GIRO_FSM_S1;
				while (Chip_ADC_ReadStatus(LPC_ADC, ADC_CHANNEL,
						ADC_DR_DONE_STAT) != SET) {
				}
				Chip_ADC_ReadValue(LPC_ADC, ADC_CHANNEL, &adcValue);
				velocidadResultante = actualizarVelocidad(adcValue, lastAdc,
						clockPwm);
				if (velocidadResultante != 0) {
					sprintf(mensaje, "PWM:%d     ", velocidadResultante);
					dataLCD.mensaje = mensaje;
					xQueueSend(xQueueLCD, &dataLCD, 10);
				}

				Chip_GPIO_SetPinState(LPC_GPIO, GIRO_DER, 0);
				Chip_GPIO_SetPinState(LPC_GPIO, GIRO_IZQ, 1);
				solicitudGiroIzq = 0;
			}
			break;
		case GIRO_FSM_S2:
			if (velocidadActual == 0) {
				estadoActual = GIRO_FSM_S3;
				Chip_GPIO_SetPinState(LPC_GPIO, GIRO_DER, 1);
				Chip_GPIO_SetPinState(LPC_GPIO, GIRO_IZQ, 0);
			} else {
				estadoActual = GIRO_FSM_S2;
				adcValue = 0;
				velocidadResultante = actualizarVelocidad(adcValue, lastAdc,
						clockPwm);
				if (velocidadResultante != 0) {
					sprintf(mensaje, "PWM:%d     ", velocidadResultante);
					dataLCD.mensaje = mensaje;
					xQueueSend(xQueueLCD, &dataLCD, 10);
				}

				Chip_GPIO_SetPinState(LPC_GPIO, GIRO_DER, 0);
				Chip_GPIO_SetPinState(LPC_GPIO, GIRO_IZQ, 0);
			}
			break;
		case GIRO_FSM_S3:
			if (solicitudGiroIzq == 1) {
				if (velocidadActual == 0) {
					estadoActual = GIRO_FSM_S1;
					Chip_GPIO_SetPinState(LPC_GPIO, GIRO_DER, 0);
					Chip_GPIO_SetPinState(LPC_GPIO, GIRO_IZQ, 1);
				} else {
					estadoActual = GIRO_FSM_S4;
					adcValue = 0;
					velocidadResultante = actualizarVelocidad(adcValue, lastAdc,
							clockPwm);
					if (velocidadResultante != 0) {
						sprintf(mensaje, "PWM:%d     ", velocidadResultante);
						dataLCD.mensaje = mensaje;
						xQueueSend(xQueueLCD, &dataLCD, 10);
					}

					Chip_GPIO_SetPinState(LPC_GPIO, GIRO_DER, 0);
					Chip_GPIO_SetPinState(LPC_GPIO, GIRO_IZQ, 0);
				}
				solicitudGiroIzq = 0;
			} else {
				estadoActual = GIRO_FSM_S3;
				while (Chip_ADC_ReadStatus(LPC_ADC, ADC_CHANNEL,
						ADC_DR_DONE_STAT) != SET) {
				}
				Chip_ADC_ReadValue(LPC_ADC, ADC_CHANNEL, &adcValue);
				velocidadResultante = actualizarVelocidad(adcValue, lastAdc,
						clockPwm);
				if (velocidadResultante != 0) {
					sprintf(mensaje, "PWM:%d     ", velocidadResultante);
					dataLCD.mensaje = mensaje;
					xQueueSend(xQueueLCD, &dataLCD, 10);
				}

				Chip_GPIO_SetPinState(LPC_GPIO, GIRO_DER, 1);
				Chip_GPIO_SetPinState(LPC_GPIO, GIRO_IZQ, 0);
				solicitudGiroDer = 0;
			}
			break;
		case GIRO_FSM_S4:
			if (velocidadActual == 0) {
				estadoActual = GIRO_FSM_S1;
				Chip_GPIO_SetPinState(LPC_GPIO, GIRO_DER, 0);
				Chip_GPIO_SetPinState(LPC_GPIO, GIRO_IZQ, 1);
			} else {
				estadoActual = GIRO_FSM_S4;
				adcValue = 0;
				velocidadResultante = actualizarVelocidad(adcValue, lastAdc,
						clockPwm);
				if (velocidadResultante != 0) {
					sprintf(mensaje, "PWM:%d     ", velocidadResultante);
					dataLCD.mensaje = mensaje;
					xQueueSend(xQueueLCD, &dataLCD, 10);
				}

				Chip_GPIO_SetPinState(LPC_GPIO, GIRO_DER, 0);
				Chip_GPIO_SetPinState(LPC_GPIO, GIRO_IZQ, 0);
			}
			break;
		}

		vTaskDelayUntil(&xLastWakeTime, 20);
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
	unsigned int counterLCD = 0;

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
		xQueueSend(xQueueVelocidad,&rpm[i],0);

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
			counterLCD++;
			if(counterLCD > (rpm[i]/divisorRPM)){
				sprintf(mensaje, "RPM:%d     ", aux);
							datosLcd.mensaje = mensaje;
							xQueueSend(xQueueLCD, &datosLcd, 0);
							counterLCD = 0;
			}


		}

	}
}

static void vLCD_Task(void *pvParameters) {
	//i2c init
	I2C_ID_T id = I2C2;
	LCD_Struct datosLcd;

	Board_I2C_Init(id);
	Chip_I2C_Init(id);
	Chip_I2C_SetClockRate(id, 100000);
	NVIC_DisableIRQ(I2C2_IRQn);
	Chip_I2C_SetMasterEventHandler(id, Chip_I2C_EventHandlerPolling);

	Chip_IOCON_PinMuxSet(LPC_IOCON, I2C_SDA_PORT, I2C_SDA_PIN, 2);
	Chip_IOCON_PinMuxSet(LPC_IOCON, I2C_SCL_PORT, I2C_SCL_PIN, 2);

	vTaskDelay(300); // Le doy tiempo a la lógica del LCD para iniciar

	lcd_init(id);
	lcd_clear();

	while (1) {
		xQueueReceive(xQueueLCD, &datosLcd, portMAX_DELAY);
		lcd_set_cursor(datosLcd.linea, datosLcd.posicion);
		lcd_string(datosLcd.mensaje);
	}
}

static void vPulsadores_Task (void *pvParameters) {
	int sentidoDeGiro = 1;

	Chip_GPIOINT_Init(LPC_GPIOINT);

	Chip_IOCON_PinMux(LPC_IOCON,BOTON1, IOCON_MODE_PULLUP,IOCON_FUNC0);
	Chip_IOCON_PinMux(LPC_IOCON,BOTON2, IOCON_MODE_PULLUP,IOCON_FUNC0);
	Chip_GPIO_SetPinDIRInput(LPC_GPIO, BOTON2);
	Chip_GPIO_SetPinDIRInput(LPC_GPIO, BOTON1);

	//Chip_GPIOINT_SetIntFalling(LPC_GPIOINT,BOTON1_PORT, 1<<BOTON1_PIN);
	Chip_GPIOINT_SetIntFalling(LPC_GPIOINT,BOTON2_PORT, (1<<BOTON1_PIN| 1<<BOTON2_PIN));

	NVIC_ClearPendingIRQ(EINT3_IRQn);
	NVIC_EnableIRQ(EINT3_IRQn);



	while (1)
	{
		vTaskDelay(5000);
		sentidoDeGiro++;
		if(sentidoDeGiro > 2) sentidoDeGiro = 1;
		//xQueueSend(xQueueGiro,&sentidoDeGiro,0);
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

	xTaskCreate(vGiro_Task, ( char* )"vMensaje_Task",configMINIMAL_STACK_SIZE*2, NULL, PULSOS_TASK_PRIORITY, NULL);
	xTaskCreate(vPulsos_Task, ( char* )"vPulsos_Task",configMINIMAL_STACK_SIZE, NULL, PULSOS_TASK_PRIORITY,NULL);
	xTaskCreate(vPulsadores_Task,(char *)"vPulsadores_Task",configMINIMAL_STACK_SIZE, NULL, PULSOS_TASK_PRIORITY,NULL);

	xTaskCreate(vLCD_Task, ( char* )"vLCD_Task", configMINIMAL_STACK_SIZE,NULL, LCD_TASK_PRIORITY, NULL);

	/* UART output thread, simply counts seconds */
	//xTaskCreate(vUARTTask, (signed char* ) "vTaskUart",configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),(xTaskHandle *) NULL);
	//xSemaphorePulso = xSemaphoreCreateCounting(5000, 1);
	xQueueLCD = xQueueCreate(3, sizeof(LCD_Struct));
	xQueueCapture = xQueueCreate(10, sizeof(unsigned int));
	xQueueGiro = xQueueCreate(10, sizeof(unsigned int));
	xQueueVelocidad = xQueueCreate(10, sizeof(unsigned int));

	/* Start the scheduler */
	vTaskStartScheduler();

	/* Should never arrive here */
	return 1;
}

void TIMER3_IRQHandler(void)
{
	unsigned int captura = 0;
	portBASE_TYPE HigherPriorityTaskWoken = 0;
	Chip_TIMER_Reset(LPC_TIMER3);
	if (Chip_TIMER_CapturePending(LPC_TIMER3, 1))
	{
		captura = Chip_TIMER_ReadCapture(LPC_TIMER3, 1);
		Chip_TIMER_ClearCapture(LPC_TIMER3, 1);

		NVIC_ClearPendingIRQ(TIMER3_IRQn);
				xQueueSendFromISR(xQueueCapture, &captura, &HigherPriorityTaskWoken);
	}
	if (HigherPriorityTaskWoken)
	{
		/* Actual macro used here is port specific. */
		portEND_SWITCHING_ISR(HigherPriorityTaskWoken);
	}

}

void EINT3_IRQHandler(void)
{
	int Pines_IRQ = 0;
	portBASE_TYPE HigherPriorityTaskWoken = 0;
	int sentidoDeGiro = 0;
	Pines_IRQ = Chip_GPIOINT_GetStatusFalling(LPC_GPIOINT, BOTON1_PORT);
	if (Pines_IRQ & (1 << BOTON1_PIN))
	{
		Chip_GPIOINT_ClearIntStatus(LPC_GPIOINT, BOTON1_PORT, 1 << BOTON1_PIN);
		//xQueueSend(xQueueGiro,&sentidoDeGiro,0);
		sentidoDeGiro = 1;
		xQueueSendFromISR(xQueueGiro, &sentidoDeGiro, &HigherPriorityTaskWoken);
	}
	else if (Pines_IRQ & (1 << BOTON2_PIN))
	{
		Chip_GPIOINT_ClearIntStatus(LPC_GPIOINT, BOTON2_PORT, 1 << BOTON2_PIN);
		sentidoDeGiro = 2;
		xQueueSendFromISR(xQueueGiro, &sentidoDeGiro, &HigherPriorityTaskWoken);
	}
	else
	{
		//Limpio todas IRQ
		Chip_GPIOINT_ClearIntStatus(LPC_GPIOINT, 0, Pines_IRQ);
	}
	NVIC_ClearPendingIRQ(EINT3_IRQn);
	if (HigherPriorityTaskWoken)
	{
		/* Actual macro used here is port specific. */
		portEND_SWITCHING_ISR(HigherPriorityTaskWoken);
	}

}
/**
 * @}
 */
