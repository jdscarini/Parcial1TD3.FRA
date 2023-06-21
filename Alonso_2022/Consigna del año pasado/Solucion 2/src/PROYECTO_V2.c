/*	Efecto fade con pwm en 10 pasos
 *	Modificaciones a realizar: ingresar los valores del ciclo de actividad en porcentaje
 *	y realizar una funcion o macro que realice esa conversion
 */

#include "chip.h"
#include "FreeRTOS.h"
#include "task.h"
#include "pwm.h"
#include "incapture.h"
#include "queue.h"
#include "semphr.h"

#define DUTYMIN 0
#define DUTYMAX 100
#define ENTRADA     0
#define SALIDA     1
#define SW1         18
#define SW2         17
#define SW3         1
#define IN1         16
#define IN2         15
#define PUERTO_0	0
#define PWMSTOP		0

xQueueHandle dutyQueue, pulsesQueue;
xSemaphoreHandle xSemaforo1, xSemaforo2;

/* Tarea guardiana del PWM */
/* Recibe el valor del ciclo de trabajo por la cola dutyQueue */
static void PWM(void *pvParameters){
	uint32_t duty_cycle;
	// Seteo el PWM al mínimo (apagado) desde el inicio.
	Ancho_pulso(DUTYMIN);

	while(1){
		// Recibo el nuevo valor de ciclo de trabajo lo seteo en el PWM.
		xQueueReceive(dutyQueue,&duty_cycle,portMAX_DELAY);
		Ancho_pulso(duty_cycle);
	}
}

/* Tarea inversora de giro */
/* Si se activa el semaforo de inversion de giro, esta tarea coloca el PWM en 0% */
/* y realiza el seteo de salidas para lograr la inversion de giro. */
static void invertirGiro(void *pvParameters){
	uint32_t duty_cycle = PWMSTOP;
	while(1){
		xSemaphoreTake(xSemaforo1, portMAX_DELAY);
		// Paro el PWM
		xQueueSendToBack(dutyQueue,&duty_cycle,portMAX_DELAY);

		// Realizamos la inversion de giro
		Chip_GPIO_SetPinToggle(LPC_GPIO, PUERTO_0, IN1);
		Chip_GPIO_SetPinToggle(LPC_GPIO, PUERTO_0, IN2);
	}

}

/* Tarea guardiana de la deteccion de pulsos generada por el giro del motor */
/* Se desbloquea por el semaforo 2 y envia el dato de los pulsos a la cola pulsesQueue. */
static void tareaDetectoraPulsos(void *pvParameters){
	uint32_t pulsos = 0;
	while(1){
		xSemaphoreTake(xSemaforo2, portMAX_DELAY);
		// Obtengo los pulsos contados.
		pulsos = get_capture()
		// Mando los pulsos contados a la cola.
		xQueueSendToBack(pulsesQueue,&pulsos,portMAX_DELAY);
	}
}

/* Tarea que se encarga de la lectura de los botones y setear el duty_cycle */
static void tareaControladora(void *pvParameters){
	Chip_GPIO_WriteDirBit(LPC_GPIO, PUERTO_0, SW1, ENTRADA);
	Chip_GPIO_WriteDirBit(LPC_GPIO, PUERTO_0, SW2, ENTRADA);
	Chip_GPIO_WriteDirBit(LPC_GPIO, PUERTO_0, SW3, ENTRADA);

	uint32_t duty_cycle = 0;
	uint8_t flag_inversion = 0;
	uint32_t pulsos = 0;
	while(1){
		if(!flag_inversion){
			// Boton que incrementa en 10 el ciclo de trabajo.
			if(!Chip_GPIO_ReadPortBit(LPC_GPIO,PUERTO_0,SW1)){
				delay_ms(500);
				duty_cycle += 10;
				if(duty_cycle > DUTYMAX){
					duty_cycle = 100;
				}

				xQueueSendToBack(dutyQueue,&duty_cycle,portMAX_DELAY);
			}
			// Boton reductor de velocidad.
			if(!Chip_GPIO_ReadPortBit(LPC_GPIO,PUERTO_0,SW2)){
				delay_ms(500);


				if(duty_cycle >= 10){
					duty_cycle -= 10;
				}
				else{
					// Limito el mínimo a 0% de DC.
					if(duty_cycle < 10 && duty_cycle != 0){
						duty_cycle = 10;
					}
				}

				xQueueSendToBack(dutyQueue,&duty_cycle,portMAX_DELAY);
			}
			// Boton de inversion de giro.
			if(!Chip_GPIO_ReadPortBit(LPC_GPIO,PUERTO_0,SW3)){
							delay_ms(500);
							flag_inversion = 1;
							xSemaphoreGive(xSemaforo1);
			}
		}
		else{
			// Codigo de inversion de giro
			xSemaphoreGive(xSemaforo2); // Doy el semaforo para que la tarea lectora de pulsos se desbloquee.
			xQueueReceive(pulsesQueue,&pulsos,portMAX_DELAY); // Obtengo los pulsos generados por el movimiento del motor.

			// Si los pulsos son 0, el motor freno. Por lo tanto, vuelvo el PWM a su valor original y apago el flag.
			if(pulsos == 0){
				xQueueSendToBack(dutyQueue,&duty_cycle,portMAX_DELAY);
				flag_inversion = 0;
			}
		}
	}
}

int main(void)
{
	SystemCoreClockUpdate();

	/* Creacion de la cola */
	dutyQueue = xQueueCreate(1,sizeof(uint32_t));
	pulsesQueue = xQueueCreate(1,sizeof(uint32_t));
	vSemaphoreCreateBinary(xSemaforo1); // Semaforno inversion de giro.
	vSemaphoreCreateBinary(xSemaforo2); // Semaforno de contador de pulsos.

	xSemaphoreTake(xSemaforo1, 0); // Tomo el semaforo para que no entre en inversion de giro.
	xSemaphoreTake(xSemaforo2, 0); // Tomo el semaforo para que no entre en el contador de pulsos.

	// Seteo de los pines de inversion de giro en salida con valores 1 y 0.
	Chip_GPIO_WriteDirBit(LPC_GPIO, PUERTO_0, IN1, SALIDA);
	Chip_GPIO_WriteDirBit(LPC_GPIO, PUERTO_0, IN2, SALIDA);
	Chip_GPIO_SetPinState(LPC_GPIO, PUERTO_0, IN1, false);
	Chip_GPIO_SetPinState(LPC_GPIO, PUERTO_0, IN2, true);

	/* Inicializa el modulo pwm */
    init_pwm();
	/* Inicializa el inputcapture para el conteo de pulsos del motor */
    init_incapture();
	/* Inicializo el timer */
    init_timer();

    /* Creo las tareas */
    xTaskCreate(invertirGiro, (char *) "invertirGiro",
           			configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 2UL),
           			(xTaskHandle *) NULL);
   	xTaskCreate(PWM, (char *) "PWM",
       			configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 2UL),
       			(xTaskHandle *) NULL);
	xTaskCreate(tareaDetectoraPulsos, (char *) "tareaDetectoraPulsos",
       			configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 2UL),
       			(xTaskHandle *) NULL);
   	xTaskCreate(tareaControladora, (char *) "tareaControladora",
   	       			configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
   	       			(xTaskHandle *) NULL);

    /* Inicia el scheduler */
    vTaskStartScheduler();


    return 0;
}

