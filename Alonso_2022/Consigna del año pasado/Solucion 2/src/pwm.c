
#include <pwm.h>


/* Inicialización de timer */
void init_timer(void)
{
	/*Asumiendo que el PLL0 ha sido seteado con CCLK = 100Mhz*/

	    LPC_SYSCTL->PCONP |= (1 << 1); //Timer0
		LPC_SYSCTL->PCLKSEL[0] &= ~(0x3 << 2);// Timer0 tiene PCLK = 25 MHz

		LPC_TIMER0->CTCR = 0x0;
		LPC_TIMER0->PR = 24999; // (25Mhz * 1ms) -1 = (25*10e6 * 10e-3 ) -1 = 24999
		LPC_TIMER0->TCR = 0x2; // Reset

		//  LPC_SYSCTL->PCONP |=(1 << 22);// Timer2
		//	LPC_SYSCTL->PCLKSEL[1] |= (0x01 << 12 );// Timer2 tiene CLK = 100 MHz
		//	LPC_IOCON->PINSEL[0] |= (0x3 << 8);    // CAP2.0
/*
		LPC_TIMER2->PR = 100000000; // 100.000.000 -> 1s | 50.000.000 -> 0.5s | 5.000.000 -> 0.05s | 100.000 -> 0.001s = 1ms
		LPC_TIMER2->TCR = (1 << 1 | 0 << 0); // Reset
		LPC_TIMER2->TCR = (0 << 1 | 1 << 0); // Enable
		LPC_TIMER2->IR |= (1 << 5);
		LPC_TIMER2->CCR |= (1 << 0); //CAP0RE */
}
void delay_ms(uint32_t milisegundos)
{
	LPC_TIMER0->TCR = 0x02; //Reset Timer

	LPC_TIMER0->TCR = 0x01; //Enable timer

	while(LPC_TIMER0->TC < milisegundos); //wait until timer counter reaches the desired delay

	LPC_TIMER0->TCR = 0x00; //Disable timer
}

void init_pwm(void)
{
	uint32_t PWM1Freq;

	/* Enables power and clocking for a peripheral */
	PWM1_enableCLK(my_PWM1); //step 1: PowerCONtrol of Peripheral (PCONP) register, bit 6 p PWM1
	PWM1Freq = Chip_Clock_GetPeripheralClockRate(my_PWM1_SPEED);//step 2: PCLKSEL0 para PWM1

	PWM1_preescaleCLK(my_PWM1, SYSCTL_CLKDIV_4); //seteo el preescaler dividiendo por 1, asi trabajo a 30MHz

	PWM1_SelectChannel(my_PWM1, CH1); //MI SALIDA PWM CON LA QUE CONMUTARÉ EL SCR

	PWM1_ControlChannel(my_PWM1, CH1, SINGLE_EDGE, DISABLE_OUT); //canal 1 single edge TRABAJA CON EL PCR
	PWM1_ValuePrescale(my_PWM1, PWMPRESCALE);
	PWM1_SetMatch(my_PWM1, MATCH0, PWM_PERIOD); //seteo periodo PWM mediante registro MR0(match 0)
	PWM1_SetMatch(my_PWM1, MATCH1, DUTY_CICLE); //seteo dutycicle PWM mediante registro MR1(match 1)

	/*Restablecer en PWM1MR0 Match: si se establece en 1, restablecerá el contador del temporizador PWM,
	 * es decir, PWM1TC, de lo contrario, se deshabilitará si se establece en 0.
	 */
	PWM1_MCR_RestoreTC(my_PWM1,MATCH0);//ojo con que se malinteprete el nombre: es para que se reestablezca luego de que TC=MRx
	//PWM1_ConfigMatch(my_PWM1, MATCH0, 0, 0, 0); //010: no interrupt, RESET, no stop registro MCR

	PWM1_EnableMatchValue(my_PWM1, CH0);//actualizo valor del MR0 mediante registro LER
	PWM1_EnableMatchValue(my_PWM1, CH1);//actualizo valor del MR1 mediante registro LER

	PWM1_ControlChannel(my_PWM1, CH1, SINGLE_EDGE, ENABLE_OUT);//habilito la salida P2[0] pin 42

	PWM1_ResetCounters(my_PWM1); //Reseteo el TC y el PC para el conteo
	PWM1_EnableCounters(my_PWM1);//Habilita el TC y el PC para el conteo
}

void Ancho_pulso(uint16_t pulseWidth)
{
	PWM1_SetMatch(my_PWM1, MATCH1, (pulseWidth*PWM_PERIOD)/100);
	PWM1_EnableMatchValue(my_PWM1, CH1);
}
