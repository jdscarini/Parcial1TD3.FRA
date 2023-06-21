#include<initPwm.h>

void initPwm(){
	Chip_IOCON_PinMux(LPC_IOCON, pwmPortPin, IOCON_MODE_INACT , PWM_MODE);

	LPC_PWM1->MR0 = maxduty;	//Match0 set la frec
	LPC_PWM1->MR2 = 400;		//Match2 set el duty por defecto

	/* Reset count on match0 */
	LPC_PWM1->MCR &= ~(0xFFFFFFFF); /// limpio el registro
	LPC_PWM1->MCR |= 1<<1; /// reseteo la cuenta de contador del pwm

	/* Enable PWM2 */
	LPC_PWM1->PCR &= ~(0xFFFFFFFF); //
	LPC_PWM1->PCR |= 1 << 10;// indico el canal

	/* Counter PWM enable & Counter */
	LPC_PWM1->TCR &= ~(0xFFFFFFFF); /// limpio
	LPC_PWM1->TCR |= (1 << 0) | (1 << 3);// habilito
}

void setMatch2(uint32_t matchValue){
	/* si no supera el umbral minimo se apaga */
	if (matchValue > minduty)
	{
		LPC_PWM1->MR2 = matchValue; /// cambio el duty
		LPC_PWM1->LER |= 1<<2; /// indico que debe actualizar valor
	}
	else
	{
		LPC_PWM1->MR2 = 0;
		LPC_PWM1->LER |= 1<<2;
	}

}
