#include<initIO.h>

void initIO(void){
	/* Se setean los pines digitales como salidas */
	Chip_GPIO_SetPinDIROutput(LPC_GPIO, PIN_DRIVER_DER);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO, PIN_DRIVER_IZQ);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO, Led_red);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO, Led_verde);

	/* Se inicializan los pines en un estado conocido */
	Chip_GPIO_SetPinState(LPC_GPIO, Led_red, LEDOFF);
	Chip_GPIO_SetPinState(LPC_GPIO, Led_verde, LEDOFF);
	Chip_GPIO_SetPinState(LPC_GPIO, PIN_DRIVER_DER, false);
	Chip_GPIO_SetPinState(LPC_GPIO, PIN_DRIVER_IZQ, false);
}

void controlIO(uint8_t modo){
	/* Se setean los leds y pines de control del driver según el modo de giro requerido */
	switch(modo){
	case DERECHO:
		Chip_GPIO_SetPinState(LPC_GPIO, PIN_DRIVER_DER, OFF);
		Chip_GPIO_SetPinState(LPC_GPIO, PIN_DRIVER_IZQ, ON);
		Chip_GPIO_SetPinState(LPC_GPIO, Led_red, LEDON);
		Chip_GPIO_SetPinState(LPC_GPIO, Led_verde, LEDOFF);
		break;
	case IZQUIERDO:
		Chip_GPIO_SetPinState(LPC_GPIO, PIN_DRIVER_DER, ON);
		Chip_GPIO_SetPinState(LPC_GPIO, PIN_DRIVER_IZQ, OFF);
		Chip_GPIO_SetPinState(LPC_GPIO, Led_red, LEDOFF);
		Chip_GPIO_SetPinState(LPC_GPIO, Led_verde, LEDON);
		break;
	case APAGADO:
		Chip_GPIO_SetPinState(LPC_GPIO, PIN_DRIVER_DER, OFF);
		Chip_GPIO_SetPinState(LPC_GPIO, PIN_DRIVER_IZQ, OFF);
		Chip_GPIO_SetPinState(LPC_GPIO, Led_red, LEDOFF);
		Chip_GPIO_SetPinState(LPC_GPIO, Led_verde, LEDOFF);
		break;
	default :
		break;
	}
}
