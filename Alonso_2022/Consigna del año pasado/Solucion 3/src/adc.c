
#include <adc.h>


/* Inicialización del ADC */
void init_adc(void)
{
	static ADC_CLOCK_SETUP_T ADCSetup;

	/* Asocia el pin a la función de ADC */
	Chip_IOCON_PinMux(LPC_IOCON,ADC_CH0_PORT,ADC_CH0_PIN,IOCON_MODE_INACT,IOCON_FUNC1);

	/* Inicializa el ADC */
	Chip_ADC_Init(LPC_ADC, &ADCSetup);

	/* Conecta el ADC con el canal elegido */
	Chip_ADC_EnableChannel(LPC_ADC, ADC_CH0, ENABLE);

	/* Velocidad de conversión máxima */
	Chip_ADC_SetSampleRate(LPC_ADC, &ADCSetup, ADC_MAX_SAMPLE_RATE);

	/* Habilita la interrupción de ADC */
	NVIC_EnableIRQ(ADC_IRQn);
	Chip_ADC_Int_SetChannelCmd(LPC_ADC, ADC_CH0, ENABLE);

	/* Desactiva modo ráfaga */
	Chip_ADC_SetBurstCmd(LPC_ADC, DISABLE);

}


/* Inicialización del DAC */
void init_dac(void){

	/* Asocia el pin a la función de ADC */
	Chip_IOCON_PinMux (LPC_IOCON , 0 , 26 , IOCON_MODE_INACT, IOCON_FUNC2 );

	/* Inicializa el DAC */
	Chip_DAC_Init(LPC_DAC);
}


