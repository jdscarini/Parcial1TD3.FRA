#include <adc.h>

extern xQueueHandle cola_data_from_adc0;

void initADC(void)
{
	Chip_IOCON_PinMux (LPC_IOCON, PUERTOADC, PINADC , IOCON_MODE_INACT , IOCON_FUNC1);
	ADC_CLOCK_SETUP_T adc;
	/* Inicializa el ADC */
	Chip_ADC_Init(LPC_ADC, &adc);
	/* Habilita el canal */
	Chip_ADC_EnableChannel(LPC_ADC, ADC_CH0, ENABLE);
	/* Muestras por segundo */
	Chip_ADC_SetSampleRate(LPC_ADC, &adc, 50000);
	/* Modo ráfaga deshabilitado */
	Chip_ADC_SetBurstCmd(LPC_ADC, DISABLE);

	/* Habilita la interrupción de ADC */
	Chip_ADC_Int_SetChannelCmd(LPC_ADC, ADC_CH0, ENABLE);
	NVIC_ClearPendingIRQ (ADC_IRQn);
	NVIC_EnableIRQ(ADC_IRQn);
}

void ADC_IRQHandler(void)
{
	static uint16_t data = 0;
	portBASE_TYPE pxHigherPriorityTaskWoken = pdFALSE;
	/* Se realiza la lectura de la conversion */
	Chip_ADC_ReadValue(LPC_ADC, ADC_CH0, &data);

	if(data)
	{
		/* Cargo en la queue el valor leido */
		xQueueSendToBackFromISR(cola_data_from_adc0, &data, &pxHigherPriorityTaskWoken);
	}

	portEND_SWITCHING_ISR(pxHigherPriorityTaskWoken);
}

