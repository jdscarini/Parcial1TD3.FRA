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

#define pulsosPorVuelta			12	//Cantidad de pulsos del encoder
#define timeMaxQueuePulsos		20 	//Lo mínimo que puedo medir son xx RPM
#define muestrasPromedio		240 	//promedio las ultimas 2 vueltas
#define ticksEsperaLCD 			1000	//Tiempo de refresco del display.

/************************
 * Definiciones para máquina de estados
 */
#define GIRO_FSM_S1				1
#define GIRO_FSM_S2				2
#define GIRO_FSM_S3				3
#define GIRO_FSM_S4				4

/************************
 * Definiciones de pines
 */

#define I2C_SDA					0,10
#define I2C_SDA_PIN 			10
#define I2C_SDA_PORT 			0
#define I2C_SCL					0,11
#define I2C_SCL_PIN 			11
#define I2C_SCL_PORT 			0

#define GIRO_IZQ_PORT			2
#define GIRO_IZQ_PIN			1
#define GIRO_IZQ				2,1
#define GIRO_DER_PORT			2
#define GIRO_DER_PIN			2
#define GIRO_DER				2,2

#define BOTON1					0,27
#define BOTON2					0,28
#define BOTON1_PORT				0
#define BOTON1_PIN				27
#define BOTON2_PORT				0
#define BOTON2_PIN				28

#define ENTRADAPULSOS			0,24
#define ENTRADAPULSOS_PORT		0
#define ENTRADAPULSOS_PIN		24

/**************
 * Definiciones de estructura envío de cola LCD
 */
typedef struct LCD_t
{
	char *mensaje;
	unsigned int linea;
	unsigned int posicion;
} LCD_Struct;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/
xQueueHandle xQueueLCD;
xQueueHandle xQueueCapture;
xQueueHandle xQueueGiro;
xQueueHandle xQueueVelocidad;
/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Sets up system hardware */
static void prvSetupHardware(void)
{
	SystemCoreClockUpdate();
	Board_Init();

	/* Initial LED0 state is off */
	Board_LED_Set(0, true);
	Board_LED_Set(1, true);
	Board_LED_Set(2, true);

	Chip_GPIO_SetPinDIRInput(LPC_GPIO, PULS_EXT_1);
}

/*
 * actualValue = cuentas actuales del ADC
 * lastValue = cuentas anteriores del ADC
 * clockPwm = Tick maximos del pwm.
 */

unsigned int actualizarVelocidad(unsigned int actualValue,
		unsigned int lastValue, unsigned int clockPwm)
{
	unsigned int aux = 0;
	if (actualValue > lastValue + 2 || actualValue < lastValue - 2)
	{
		/* comparo si vario el PWM + de 2 cuentas */
		aux = actualValue * 100 / 4096; //Regla de 3 para llevar a escala 0-100%
		LPC_PWM1->MR1 = (actualValue * clockPwm / 4096); // Regla de 3 para llevar a escala PWM
		LPC_PWM1->LER = (1 << 0) | (1 << 1); // Configuro latch PWM.
	}
	return aux;
}

/* LED1 toggle thread */
static void vGiro_Task(void *pvParameters)
{
	unsigned int clockPwm = 0;
	//unsigned int asereje = 10;
	unsigned int estadoActual = 1;
	char mensaje[16] = { ' ' };
	char msg2[16] = { ' ' };
	LCD_Struct dataLCD;
	//char *msg = "                ";
	//char *msg2 = "                ";
	//dataLCD.mensaje = msg;
	dataLCD.linea = 0;
	dataLCD.posicion = 0;

	uint16_t adcValue = 2000, lastAdc = 0;
	static ADC_CLOCK_SETUP_T ADCSetup;
	unsigned int sentidoDeGiro = 0;
	portTickType xLastWakeTime;
	unsigned int velocidadActual = 0;
	unsigned char solicitudGiroDer = 0, solicitudGiroIzq = 0;
	unsigned int velocidadResultante = 0;

	clockPwm = Chip_Clock_GetSystemClockRate() / 800; //Divido para obtener un pwm a 200hz, frec ideal para motor.

	// Inicializo ADC en modo burst
	Chip_ADC_Init(LPC_ADC, &ADCSetup);
	Chip_IOCON_PinMuxSet(LPC_IOCON, ADC_PORT, ADC_PIN, 3);
	Chip_ADC_EnableChannel(LPC_ADC, ADC_CHANNEL, ENABLE);
	Chip_ADC_SetBurstCmd(LPC_ADC, ENABLE);
	//Configuro pines de salida para dirección de giro
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

	//Configuro pulsadores
	Chip_GPIOINT_Init(LPC_GPIOINT);

	Chip_IOCON_PinMux(LPC_IOCON, BOTON1, IOCON_MODE_PULLUP, IOCON_FUNC0);
	Chip_IOCON_PinMux(LPC_IOCON, BOTON2, IOCON_MODE_PULLUP, IOCON_FUNC0);
	Chip_GPIO_SetPinDIRInput(LPC_GPIO, BOTON2);
	Chip_GPIO_SetPinDIRInput(LPC_GPIO, BOTON1);

	Chip_GPIOINT_SetIntFalling(LPC_GPIOINT, BOTON2_PORT,
			(1 << BOTON1_PIN | 1 << BOTON2_PIN));

	NVIC_ClearPendingIRQ(EINT3_IRQn);
	NVIC_EnableIRQ(EINT3_IRQn);

	while (1)
	{
		lastAdc = adcValue;

		//recibo velocidad por cola.
		if (pdPASS != xQueueReceive(xQueueVelocidad, &velocidadActual, 0))
		{
			//No pude actualizar velocidad, asumo que la velocidad no vario.
			velocidadActual = velocidadActual;
		}
		//recibo sentido de giro por cola.
		if (pdPASS == xQueueReceive(xQueueGiro, &sentidoDeGiro, 0))
		{
			if (sentidoDeGiro == 1)
			{
				solicitudGiroDer = 1;
				solicitudGiroIzq = 0;
			}
			else if (sentidoDeGiro == 2)
			{
				solicitudGiroDer = 0;
				solicitudGiroIzq = 1;
			}
		}
		//Máquina de estados para control de giro
		switch (estadoActual)
		{
		default:
		case GIRO_FSM_S1:
			if (solicitudGiroDer == 1)
			{
				if (velocidadActual == 0)
				{
					//Ya que velocidad == 0, cambio de giro directamente
					estadoActual = GIRO_FSM_S3;
					Chip_GPIO_SetPinState(LPC_GPIO, GIRO_DER, 1);
					Chip_GPIO_SetPinState(LPC_GPIO, GIRO_IZQ, 0);
					dataLCD.linea = 1;
					dataLCD.posicion = 0;
					sprintf(msg2, "Giro Derecha  ");
					dataLCD.mensaje = msg2;
					xQueueSend(xQueueLCD, &dataLCD, portMAX_DELAY);

				}
				else
				{
					//Si velocidad != 0, pongo pwm en 0
					estadoActual = GIRO_FSM_S2;
					dataLCD.linea = 1;
					dataLCD.posicion = 0;
					sprintf(msg2, "Deteniendo    ");
					dataLCD.mensaje = msg2;
					xQueueSend(xQueueLCD, &dataLCD, portMAX_DELAY);

					adcValue = 0;
					velocidadResultante = actualizarVelocidad(adcValue, lastAdc,
							clockPwm);

					dataLCD.linea = 0;
					dataLCD.posicion = 0;
					sprintf(mensaje, "PWM:OFF");
					dataLCD.mensaje = mensaje;
					xQueueSend(xQueueLCD, &dataLCD, 10);

					//Apago sentido de giro
					Chip_GPIO_SetPinState(LPC_GPIO, GIRO_DER, 0);
					Chip_GPIO_SetPinState(LPC_GPIO, GIRO_IZQ, 0);
				}
				solicitudGiroDer = 0;
			}
			else
			{
				estadoActual = GIRO_FSM_S1; //continuo en el estado actual
				while (Chip_ADC_ReadStatus(LPC_ADC, ADC_CHANNEL,
						ADC_DR_DONE_STAT) != SET)
				{
					//Espero que termine de convertir el ADC
				}
				//Leo el ADC
				Chip_ADC_ReadValue(LPC_ADC, ADC_CHANNEL, &adcValue);

				//Actualizo velocidad en pwm y en lcd
				velocidadResultante = actualizarVelocidad(adcValue, lastAdc,
						clockPwm);
				if (velocidadResultante != 0)
				{
					dataLCD.linea = 0;
					dataLCD.posicion = 0;
					sprintf(mensaje, "PWM:%d%% ", velocidadResultante);
					dataLCD.mensaje = mensaje;
					xQueueSend(xQueueLCD, &dataLCD, 10);
				}
				//Me aseguro girar el sentido de giro
				Chip_GPIO_SetPinState(LPC_GPIO, GIRO_DER, 0);
				Chip_GPIO_SetPinState(LPC_GPIO, GIRO_IZQ, 1);
				solicitudGiroIzq = 0;
			}
			break;
		case GIRO_FSM_S2:
			if (velocidadActual == 0)
			{
				//como esta detenido pasa al estado s3, seteando el sentido de giro
				estadoActual = GIRO_FSM_S3;

				dataLCD.linea = 1;
				dataLCD.posicion = 0;
				sprintf(msg2, "Giro Derecha  ");
				dataLCD.mensaje = msg2;
				xQueueSend(xQueueLCD, &dataLCD, portMAX_DELAY);
				Chip_GPIO_SetPinState(LPC_GPIO, GIRO_DER, 1);
				Chip_GPIO_SetPinState(LPC_GPIO, GIRO_IZQ, 0);
			}
			else
			{
				// reasegura el estado actual
				estadoActual = GIRO_FSM_S2;
				// Apaga el pwm y actualiza display
				adcValue = 0;
				velocidadResultante = actualizarVelocidad(adcValue, lastAdc,
						clockPwm);
				dataLCD.linea = 0;
				dataLCD.posicion = 0;
				sprintf(mensaje, "PWM:OFF");
				dataLCD.mensaje = mensaje;
				xQueueSend(xQueueLCD, &dataLCD, 10);

				Chip_GPIO_SetPinState(LPC_GPIO, GIRO_DER, 0);
				Chip_GPIO_SetPinState(LPC_GPIO, GIRO_IZQ, 0);
			}
			break;
		case GIRO_FSM_S3:
			if (solicitudGiroIzq == 1)
			{
				//Controlo velocidad
				if (velocidadActual == 0)
				{
					//Si se encuentra detenido, cambio directo
					estadoActual = GIRO_FSM_S1;
					dataLCD.linea = 1;
					dataLCD.posicion = 0;
					sprintf(msg2, "Giro Izquierda");
					dataLCD.mensaje = msg2;
					xQueueSend(xQueueLCD, &dataLCD, portMAX_DELAY);

					Chip_GPIO_SetPinState(LPC_GPIO, GIRO_DER, 0);
					Chip_GPIO_SetPinState(LPC_GPIO, GIRO_IZQ, 1);
				}
				else
				{
					//Apago el motor encaso de que siga girando. Cambio a S4.
					estadoActual = GIRO_FSM_S4;
					dataLCD.linea = 1;
					dataLCD.posicion = 0;
					sprintf(msg2, "Deteniendo    ");
					dataLCD.mensaje = msg2;
					xQueueSend(xQueueLCD, &dataLCD, portMAX_DELAY);
					adcValue = 0;
					velocidadResultante = actualizarVelocidad(adcValue, lastAdc,
							clockPwm);
					dataLCD.linea = 0;
					dataLCD.posicion = 0;
					sprintf(mensaje, "PWM:OFF ");
					dataLCD.mensaje = mensaje;
					xQueueSend(xQueueLCD, &dataLCD, 10);

					Chip_GPIO_SetPinState(LPC_GPIO, GIRO_DER, 0);
					Chip_GPIO_SetPinState(LPC_GPIO, GIRO_IZQ, 0);
				}
				solicitudGiroIzq = 0;
			}
			else
			{
				estadoActual = GIRO_FSM_S3;
				//Como no tengo solicitud de giro, actualizo el ADC
				while (Chip_ADC_ReadStatus(LPC_ADC, ADC_CHANNEL,
						ADC_DR_DONE_STAT) != SET)
				{
				}
				Chip_ADC_ReadValue(LPC_ADC, ADC_CHANNEL, &adcValue);
				// actualizo el lcd con lo que medi en el ADC.
				velocidadResultante = actualizarVelocidad(adcValue, lastAdc,
						clockPwm);
				if (velocidadResultante != 0)
				{
					dataLCD.linea = 0;
					dataLCD.posicion = 0;
					sprintf(mensaje, "PWM:%d%% ", velocidadResultante);
					dataLCD.mensaje = mensaje;
					xQueueSend(xQueueLCD, &dataLCD, 10);
				}

				Chip_GPIO_SetPinState(LPC_GPIO, GIRO_DER, 1);
				Chip_GPIO_SetPinState(LPC_GPIO, GIRO_IZQ, 0);
				solicitudGiroDer = 0;
			}
			break;
		case GIRO_FSM_S4:
			//IDEM Estado 2
			if (velocidadActual == 0)
			{
				estadoActual = GIRO_FSM_S1;
				dataLCD.linea = 1;
				dataLCD.posicion = 0;
				sprintf(msg2, "Giro Izquierda");
				dataLCD.mensaje = msg2;
				xQueueSend(xQueueLCD, &dataLCD, portMAX_DELAY);
				Chip_GPIO_SetPinState(LPC_GPIO, GIRO_DER, 0);
				Chip_GPIO_SetPinState(LPC_GPIO, GIRO_IZQ, 1);
			}
			else
			{
				estadoActual = GIRO_FSM_S4;
				adcValue = 0;
				velocidadResultante = actualizarVelocidad(adcValue, lastAdc,
						clockPwm);
				dataLCD.linea = 0;
				dataLCD.posicion = 0;
				sprintf(mensaje, "PWM:OFF ");
				dataLCD.mensaje = mensaje;
				xQueueSend(xQueueLCD, &dataLCD, 10);

				Chip_GPIO_SetPinState(LPC_GPIO, GIRO_DER, 0);
				Chip_GPIO_SetPinState(LPC_GPIO, GIRO_IZQ, 0);
			}
			break;
		}
		//Espero 20ms para dejar que las otras tarejas se puedan ejecutar.
		vTaskDelayUntil(&xLastWakeTime, 20);
		xLastWakeTime = xTaskGetTickCount();
	}

}

/* LED2 toggle thread */
static void vPulsos_Task(void *pvParameters)
{
	unsigned int captura;
	unsigned int rpm[muestrasPromedio] =
	{ 0 }; //Guardo un array por si necesito hacer promedio.
	unsigned int frecVuelta = 10;
	unsigned int i = 0, ii = 0, aux = 0;
	LCD_Struct datosLcd;
	uint32_t timerFreq;
	char mensaje[16] =
	{ ' ' };
	TickType_t xLastWakeTime = 0;

	//Configuro timer para tomar muestras por interrupcion. En modo Input Capture
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
	//Configuro pin del timer como funcion timer.
	Chip_IOCON_PinMux(LPC_IOCON, 0, 24, IOCON_MODE_PULLUP, IOCON_FUNC3);

	while (1)
	{

		if (xQueueReceive(xQueueCapture, &captura, timeMaxQueuePulsos) == pdPASS) //Se fija si recibe pulsos antes de timeMaxQueuePulsos
		{
			//Si pudo recibir, hace la conversion a RPM y la guarda en el vector
			frecVuelta = timerFreq / captura; //Calculo frecuencia de los pulsos
			rpm[i] = (frecVuelta * 10 / pulsosPorVuelta) * 6; //Convierto a frecuencia de vuelta y luego a RPM
		}
		else
		{
			//Si no pudo recibir pulsos en la ventana temporal, asume que rpm = 0
			rpm[i] = 0;
		}

		xQueueSend(xQueueVelocidad, &rpm[i], 0); //Envia por cola el ultimo valor medido de rpm

		if ((xLastWakeTime + ticksEsperaLCD) <= xTaskGetTickCount()) //Controla el tiempo de actualizacion del display
		{
			xLastWakeTime = xTaskGetTickCount(); //Actualizo ultimo valor de envio de display
			//posiciona cursor LCD
			datosLcd.linea = 0;
			datosLcd.posicion = 8;
			aux = 0;
			for (ii = 0; ii < muestrasPromedio; ii++)
			{
				aux += rpm[ii];
			}
			aux = aux / muestrasPromedio;		//Realiza promedio de muestras

			sprintf(mensaje, "RPM:%d     ", aux);
			datosLcd.mensaje = mensaje;
			//Envia el dato a imprimir por cola al LCD
			xQueueSend(xQueueLCD, &datosLcd, 0);
		}

		i++; //Incrementa el vector circular de rpm.
		if (i >= muestrasPromedio)
		{
			i = 0;
		}
		vTaskDelay(timeMaxQueuePulsos); //Demora obligatoria para que cambie de tarea.
		//Como tengo una demora maxima de 40ms, puedo medir un minimo de 125 RPM
		//timeMaxQueuePulsos = 20ms

	}

}

static void vLCD_Task(void *pvParameters)
{
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

	//vTaskDelay(300); // Le doy tiempo a la lógica del LCD para iniciar

	lcd_init(id);
	lcd_clear();

	while (1)
	{
		xQueueReceive(xQueueLCD, &datosLcd, portMAX_DELAY);
		lcd_set_cursor(datosLcd.linea, datosLcd.posicion);
		lcd_string(datosLcd.mensaje);
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

	xTaskCreate(vGiro_Task, (char*) "vGiro_Task", configMINIMAL_STACK_SIZE,
	NULL, PULSOS_TASK_PRIORITY, NULL);
	xTaskCreate(vPulsos_Task, (char*) "vPulsos_Task",
	configMINIMAL_STACK_SIZE * 10, NULL, PULSOS_TASK_PRIORITY, NULL);

	xTaskCreate(vLCD_Task, (char*) "vLCD_Task", configMINIMAL_STACK_SIZE, NULL,
	LCD_TASK_PRIORITY, NULL);

	xQueueLCD = xQueueCreate(10, sizeof(LCD_Struct));
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
	Chip_TIMER_Reset(LPC_TIMER3);					//Reseteo contador del timer
	if (Chip_TIMER_CapturePending(LPC_TIMER3, 1))//Me fijo si hay una captura pendiente
	{
		captura = Chip_TIMER_ReadCapture(LPC_TIMER3, 1);	//Leo la captura
		Chip_TIMER_ClearCapture(LPC_TIMER3, 1);				//Limpio la captura

		NVIC_ClearPendingIRQ(TIMER3_IRQn);		//Limpio la interrupcion en NVIC
		xQueueSendFromISR(xQueueCapture, &captura, &HigherPriorityTaskWoken);
		// Envio el dato por cola
	}
	if (HigherPriorityTaskWoken)	// Hago cambio de contaxto si es necesario
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
