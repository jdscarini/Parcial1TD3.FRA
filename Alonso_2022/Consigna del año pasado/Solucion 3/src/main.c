/*
===============================================================================
 Name        : tp_INT_27-06-22.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : Se solicita realizar un sistema que realice el control de giro y
 	 	 	   velocidad de un motor de CC. La velocidad será controlada por una
 	 	 	   señal PWM que podrá variar en forma continua de 0 a 100%. Además,
 	 	 	   se debe proveer de dos pulsadores tal que permitan el cambio de
 	 	 	   giro; cuando se gira hacia la derecha se encenderá un Led Verde y
 	 	 	   cuando se gira a la izquierda se encenderá un Led Rojo. (Para
 	 	 	   este ítem, será necesario detectar que la velocidad del motor es
 	 	 	   cero, luego realizar el cambio de giro).
===============================================================================
*/

#include "chip.h"
#include "FreeRTOS.h"
#include "adc.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "PWM.h"
#include "timer.h"

#define ledrojo_pin			6
#define ledverde_pin		7
#define pulsadorD_pin		8
#define pulsadorI_pin		9
#define giro_pin			24
#define ngiro_pin			25
#define muestra_ADC 		100
#define LAZO_CERRADO		0

/* Handler de la cola */
xQueueHandle queueADC, queueVel;

/* Handler de los semaforos */
xSemaphoreHandle SemaforoVel, SemaforoFrenado;

/* Tarea de muestreo del ADC */
static void MuestraADC(void *pvParameters)
{
	uint16_t dataADC, pwm_lazo=2048, velocidad;

	/* Inicializo el adc en P0.23*/
	init_adc();

	/* Inicializo el PWM */
	initPWM();

	while(1)
	{
		/* Comienza una nueva conversión */
		Chip_ADC_SetStartMode(LPC_ADC, ADC_START_NOW, ADC_TRIGGERMODE_RISING);

		/* Esta demora marca la frecuencia con que se toman las muestras */
		vTaskDelay(muestra_ADC/portTICK_RATE_MS);

		/* Recibe el dato del ADC */
		xQueueReceive(queueADC,&dataADC,portMAX_DELAY);

		if(LAZO_CERRADO)
		{
			xQueueReceive(queueVel,&velocidad,portMAX_DELAY);
			if(velocidad < dataADC)
				pwm_lazo += (dataADC-velocidad);
			else
			{
				if(pwm_lazo < (velocidad-dataADC))
					pwm_lazo = 0;
				else
					pwm_lazo -= (velocidad-dataADC);
			}
			if(pwm_lazo > 4095)
				pwm_lazo = 4095;

			UpdatePWM(pwm_lazo);
		}
		else
		{
			/* Cargo el dato del ADC a la salida PWM para variar la velocidad de giro */
			UpdatePWM(dataADC);
		}
	}
}

static void EncoderVelocidad(void *pvParameters)
{
	uint16_t velocidad=0;

	/* Inicializo el Timer */
	initTimer();

	while(1)
	{
		/* Espero por la medicion del Timer Capture (Si no hay medicion es porque el motor esta detenido) */
		xSemaphoreTake(SemaforoVel, 1000/portTICK_RATE_MS);

		/* Calculo la velocidad */
		velocidad = Velocidad_RPM();

		if(LAZO_CERRADO)
			xQueueSendToBack(queueVel,&velocidad,portMAX_DELAY);

		vTaskDelay(50/portTICK_RATE_MS);
	}
}


/* Tarea de control de giro del motor */
static void Botones(void *pvParameters)
{
	uint16_t Derecha, Izquierda, sentido = 0;

	/* Seteo pines como salida para encender el LED y para habilitar/deshabilitar el puente H*/
	Chip_GPIO_SetPinDIROutput(LPC_GPIO, 0, ledrojo_pin);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO, 0, ledverde_pin);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO, 0, giro_pin);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO, 0, ngiro_pin);

	/* Seteo pines como entrada para el pulsador y para el encoder*/
	Chip_GPIO_SetPinDIRInput(LPC_GPIO, 0, pulsadorD_pin);
	Chip_GPIO_SetPinDIRInput(LPC_GPIO, 0, pulsadorI_pin);

	while(1)
	{
		/* Leo el valor en las entradas donde se encuentra conectado el pulsador derecho e izquierdo */
		Derecha = Chip_GPIO_GetPinState(LPC_GPIO, 0, pulsadorD_pin);
		Izquierda = Chip_GPIO_GetPinState(LPC_GPIO, 0, pulsadorI_pin);

		if(Derecha==0 && sentido == 0)
		{
			/* Detengo el motor */
			Chip_GPIO_SetPinState(LPC_GPIO, 0, giro_pin, false);
			Chip_GPIO_SetPinState(LPC_GPIO, 0, ngiro_pin, false);

			/* Espero a que deje de girar */
			xSemaphoreTake(SemaforoFrenado, portMAX_DELAY);

			/* Habilito el giro hacia la derecha del puente H*/
			Chip_GPIO_SetPinState(LPC_GPIO, 0, giro_pin, true);

			/* Deshabilito el giro hacia la izquierda del puente H*/
			Chip_GPIO_SetPinState(LPC_GPIO, 0, ngiro_pin, false);

			/* Prendo el led verde que indica que el motor gira hacia la derecha */
			Chip_GPIO_SetPinState(LPC_GPIO, 0, ledverde_pin, true);
			Chip_GPIO_SetPinState(LPC_GPIO, 0, ledrojo_pin, false);
			sentido = 1;
		}

		if(Izquierda==0 && sentido == 1)
		{
			/* Detengo el motor */
			Chip_GPIO_SetPinState(LPC_GPIO, 0, giro_pin, false);
			Chip_GPIO_SetPinState(LPC_GPIO, 0, ngiro_pin, false);

			/* Espero a que deje de girar */
			xSemaphoreTake(SemaforoFrenado, portMAX_DELAY);

			/* Habilito el giro hacia la izquierda del puente H*/
			Chip_GPIO_SetPinState(LPC_GPIO, 0, ngiro_pin, true);

			/* Deshabilito el giro hacia la derecha del puente H*/
			Chip_GPIO_SetPinState(LPC_GPIO, 0, giro_pin, false);

			/* Prendo el led rojo que indica que el motor gira hacia la izquierda */
			Chip_GPIO_SetPinState(LPC_GPIO, 0, ledrojo_pin, true);
			Chip_GPIO_SetPinState(LPC_GPIO, 0, ledverde_pin, false);
			sentido = 0;
		}
		/* Demora 50ms */
		vTaskDelay(50/portTICK_RATE_MS);
	}
}


/* Interrupción de ADC */
void ADC_IRQHandler(void){

	uint16_t dataADC;

	static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	/* Lee el valor del ADC y lo guarda en la variable dataADC */
	Chip_ADC_ReadValue(LPC_ADC, ADC_CH0, &dataADC);

	/* Pone el dato en una cola para su procesamiento */
	xQueueSendToBackFromISR(queueADC,&dataADC,&xHigherPriorityTaskWoken);

	/* Fuerza la ejecución del scheduler para así retornar a una tarea más prioritaria de ser necesario */
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);

}

/* Interrupción del Timer 2 */
void TIMER2_IRQHandler(void){

	static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	/* Si la interrupcion fue por el match el motor esta detenido */
	if(Chip_TIMER_MatchPending(LPC_TIMER2, 0))
	{
		Chip_TIMER_ClearMatch(LPC_TIMER2, 0);
		Chip_TIMER_Reset(LPC_TIMER2);
		Chip_TIMER_Enable(LPC_TIMER2);
		/* Motor Detenido */
		xSemaphoreGiveFromISR(SemaforoFrenado,&xHigherPriorityTaskWoken);
	}

	/* Si la interrupcion fue por el capture (encoder) hay que calcular la velocidad */
	if(Chip_TIMER_CapturePending(LPC_TIMER2, 1))
	{
		Chip_TIMER_ClearCapture(LPC_TIMER2, 1);
		Chip_TIMER_Reset(LPC_TIMER2);
		/* Habilita la medicion de la velocidad */
		xSemaphoreGiveFromISR(SemaforoVel,&xHigherPriorityTaskWoken);
	}

	NVIC_ClearPendingIRQ(TIMER2_IRQn);

	/* Fuerza la ejecución del scheduler para así retornar a una tarea más prioritaria de ser necesario */
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);

}

/****************************************************************************************************/
/**************************************** MAIN ******************************************************/
/****************************************************************************************************/

int main(void){

	/* Levanta la frecuencia del micro */
	SystemCoreClockUpdate();

	/* Creacion de las colas */
	queueADC = xQueueCreate(1,sizeof(uint16_t));
	queueVel = xQueueCreate(1,sizeof(uint16_t));

	/* Creacion de los semaforos */
	vSemaphoreCreateBinary(SemaforoVel);
	xSemaphoreTake(SemaforoVel, 0);
	vSemaphoreCreateBinary(SemaforoFrenado);
	xSemaphoreTake(SemaforoFrenado, 0);

    /* Creacion de tareas */
	xTaskCreate(MuestraADC, (char *) "Muestreo",
    			configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
    			(xTaskHandle *) NULL);
    xTaskCreate(Botones, (char *) "Input",
        		configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
        		(xTaskHandle *) NULL);
    xTaskCreate(EncoderVelocidad, (char *) "VelControl",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
				(xTaskHandle *) NULL);
    /* Inicia el scheduler */
	vTaskStartScheduler();

	/* Nunca debería arribar aquí */

    return 0;
}
