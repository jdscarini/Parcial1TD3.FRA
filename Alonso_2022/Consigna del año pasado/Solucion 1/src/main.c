#include "main.h"

xQueueHandle cola_data_from_adc0;
xQueueHandle cola_data_from_encoder;
xQueueHandle cola_motor_parado;
xQueueHandle cola_botones;

xTaskHandle Handle1;
xTaskHandle Handle2;
xTaskHandle Handle3;
xTaskHandle Handle4;

/**********************************************************/
/*************** TAREA maq de estado                 ******/
/*************** Recibe cola_motor_parado            ******/
/*************** Recibe cola_botones                 ******/
/*************** Setea sentido de giro con los pines ******/
/**********************************************************/
static void T1_Mq_Est(void *pvParameters){

	static uint8_t estado=DER;
	static uint8_t sentidoDeGiro=SENTIDO_DER;

	uint16_t estado_boton;
	uint16_t estado_giro;

	bool giroMotor;
	bool botonApretado;


	while(1){
		/* Extracción de los datos de las colas y verifica si llego un dato */
		if(xQueueReceive(cola_botones, &estado_boton, 0) == pdPASS){
			botonApretado=true;
		}
		else{
			botonApretado=false;
		}
		if(xQueueReceive(cola_motor_parado, &estado_giro, 0) == pdPASS){
			giroMotor=true;
		}
		else{
			giroMotor=false;
		}

		/* Maquina de estados */////////////////////////////////////////////
		switch (estado) {
		case DER:
			controlIO(DERECHO);
			/* Chequea que se haya extraído un valor de la cola y compara el valor extraído */
			if (botonApretado==true && estado_boton == BOTON_IZQUIERDO_PRESIONADO)
			{
				/* Cambio al estado CERO */
				estado=CERO;
				sentidoDeGiro=SENTIDO_IZQ;
			}
			break;

		case IZQ:
			controlIO(IZQUIERDO);
			/* Chequea que se haya extraído un valor de la cola y compara el valor extraído */
			if (botonApretado==true && estado_boton == BOTON_DERECHO_PRESIONADO)
			{
				/* Cambio al estado CERO */
				estado=CERO;
				sentidoDeGiro= SENTIDO_DER ;
			}
			break;

		case CERO:
			controlIO(APAGADO);
			/* Chequea que se haya extraído un valor de la cola y compara el valor extraído */
			if (giroMotor==true && estado_giro==MOTOR_FRENADO)
			{
				if(sentidoDeGiro == SENTIDO_DER){
					/* Cambio al estado DER */
					estado=DER;
				}
				else{
					/* Cambio al estado IZQ */
					estado=IZQ;
				}
			}
			break;
		}
	}
}
/**********************************************************/
/*************** TAREA disparo del ADC ********************/
/**********************************************************/
static void T2_Disp_Adc(void *pvParameters){
	while (1) {
		/* Disparo ADC, inicio de la conversion */
		Chip_ADC_SetStartMode(LPC_ADC, ADC_START_NOW,  ADC_TRIGGERMODE_RISING);
		/* Cada 500mseg dispara el adc */
		vTaskDelay(500/portTICK_RATE_MS);
	}
}

/***********************************************************/
/************ TAREA ADC lee y setea el pwm  ****************/
/************ Recibe cola_data_from_adc0    ****************/
/************ Setea pwm con el valor de adc ****************/
/***********************************************************/
static void T3_Pwm(void *pvParameters){
	uint16_t data = 0;
	uint32_t pwm = 0;
	while (1) {
		xQueueReceive(cola_data_from_adc0, &data, portMAX_DELAY);//recibo el valor de adc
		pwm=(data * maxduty) /maxadc ;
		setMatch2(pwm);
	}
}

/************************************************************************/
/************ TAREA enconder                 ****************************/
/************ Recibe cola_data_from_encoder  ****************************/
/************ Verifica que el motor se mueva y envia cola_motor_parado **/
/************************************************************************/
static void T4_encoder(void *pvParameters){
	uint16_t estadoGiro;
	uint16_t valorActual;
	long result;

	while (1) {
		estadoGiro = 0;
		result = xQueueReceive(cola_data_from_encoder, &valorActual, 2000);
		/* En caso de no recibir valor de la cola */
		if( result != pdPASS )
		{
			/* Se carga en la cola el aviso de que el motor se detuvo */
			estadoGiro = MOTOR_FRENADO;
		}
		xQueueSendToBack( cola_motor_parado, &estadoGiro, portMAX_DELAY );
	}
}

/***************************************************/
/*************** MAIN ******************************/
/***************************************************/
int main(void){

	/* Inicializacion del micro */
	SystemCoreClockUpdate();
	initADC();
	initPwm();
	initIO();
	initEint();

	/* Creacion de recursos */
	cola_data_from_adc0 = xQueueCreate(1, sizeof(uint16_t));
	cola_data_from_encoder = xQueueCreate(1, sizeof(uint16_t));
	cola_motor_parado = xQueueCreate(1, sizeof(uint16_t));
	cola_botones = xQueueCreate(1, sizeof(uint16_t));

	/* Creacion de tareas */
	xTaskCreate(T1_Mq_Est, (char *)"T1_maq", configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 2UL), (xTaskHandle *) &Handle1);
	xTaskCreate(T2_Disp_Adc,(char *) "T2_adc_dispara", configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 2UL), (xTaskHandle *) &Handle2);
	xTaskCreate(T3_Pwm,(char *) "t3_adc", configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 2UL),(xTaskHandle *) &Handle3);
	xTaskCreate(T4_encoder,(char *) "T4_encoder", configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 2UL),(xTaskHandle *) &Handle4);

	/* Inicia el scheduler */
	vTaskStartScheduler();

	return 0;
}

