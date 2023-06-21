#include <initEint.h>

extern xQueueHandle cola_data_from_encoder;
extern xQueueHandle cola_botones;

void initEint(void){
	Chip_IOCON_PinMux(LPC_IOCON , Puerto_encoder2 , Pin_encoder2 , 2, 0);
	Chip_IOCON_PinMux(LPC_IOCON , Puerto_boton_der , Pin_boton_der , 3, 0);
	Chip_IOCON_PinMux(LPC_IOCON , Puerto_boton_izq , Pin_boton_izq , 3, 0);
	/* Seteo los pines como input */
	Chip_GPIO_SetPinDIRInput(LPC_GPIO, Puerto_encoder2 , Pin_encoder2);
	Chip_GPIO_SetPinDIRInput(LPC_GPIO,Puerto_boton_der , Pin_boton_der);
	Chip_GPIO_SetPinDIRInput(LPC_GPIO, Puerto_boton_izq , Pin_boton_izq);
	/* Seteo los flancos como rising o falling */
	Chip_GPIOINT_SetIntFalling(LPC_GPIOINT, Puerto_encoder2 , 1 << Pin_encoder2);
	Chip_GPIOINT_SetIntRising(LPC_GPIOINT, Puerto_boton_der , 1 << Pin_boton_der);
	Chip_GPIOINT_SetIntRising(LPC_GPIOINT, Puerto_boton_izq , 1 << Pin_boton_izq);
	/* Se limpian los flag de interrupcion */
	Chip_GPIOINT_ClearIntStatus(LPC_GPIOINT, 0, 1 << Pin_encoder2);
	Chip_GPIOINT_ClearIntStatus(LPC_GPIOINT, 0, 1 << Pin_boton_der);
	Chip_GPIOINT_ClearIntStatus(LPC_GPIOINT, 0, 1 << Pin_boton_izq);
	/* Se habilita la interrupción */
	NVIC_ClearPendingIRQ(EINT3_IRQn);
	NVIC_EnableIRQ(EINT3_IRQn);
}

void EINT3_IRQHandler(void)
{
	uint16_t encoderINT=0;
	uint16_t boton=0;
	portBASE_TYPE pxHigherPriorityTaskWoken = pdFALSE;

	if(Chip_GPIO_GetPinState(LPC_GPIO, Puerto_encoder2, Pin_encoder2)){
		/* Interrupción generada por el encoder */
		Chip_GPIOINT_ClearIntStatus(LPC_GPIOINT, Puerto_encoder2, 1 << Pin_encoder2);
		encoderINT=1;
		xQueueSendToBackFromISR( cola_data_from_encoder, &encoderINT, &pxHigherPriorityTaskWoken );
	}
	if(Chip_GPIO_GetPinState(LPC_GPIO, Puerto_boton_der, Pin_boton_der)){
		/* Interrupción generada por el pulsador derecho */
		Chip_GPIOINT_ClearIntStatus(LPC_GPIOINT, Puerto_boton_der, 1 << Pin_boton_der);
		boton=BOTON_DERECHO_PRESIONADO;
		xQueueSendToBackFromISR( cola_botones, &boton, &pxHigherPriorityTaskWoken );
	}
	else if(Chip_GPIO_GetPinState(LPC_GPIO, Puerto_boton_izq, Pin_boton_izq)){
		/* Interrupción generada por el pulsador izquierdo */
		Chip_GPIOINT_ClearIntStatus(LPC_GPIOINT, Puerto_boton_izq, 1 << Pin_boton_izq);
		boton=BOTON_IZQUIERDO_PRESIONADO;
		xQueueSendToBackFromISR( cola_botones, &boton, &pxHigherPriorityTaskWoken );
	}
	else{
		Chip_GPIOINT_ClearIntStatus(LPC_GPIOINT, 0, 1 << Pin_encoder2);
		Chip_GPIOINT_ClearIntStatus(LPC_GPIOINT, 0, 1 << Pin_boton_der);
		Chip_GPIOINT_ClearIntStatus(LPC_GPIOINT, 0, 1 << Pin_boton_izq);
	}
	portEND_SWITCHING_ISR(pxHigherPriorityTaskWoken);
}
