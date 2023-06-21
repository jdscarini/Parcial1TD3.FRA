/*
 * PWM.c
 *
 *  Created on: 26 jun. 2022
 *      Author: Leo_b
 */

#include "PWM.h"


void initPWM(void)
{
	uint32_t PWM1Freq;

	/* Habilito el CLK del periferico */
	PWM1_enableCLK (LPC_PWM1);
	PWM1_ConfigMatch(LPC_PWM1, 0, 0, 1, 0); // Accion al llegar al match: Reset TC

	/* Inicializo el PWM */
	PWM1_preescaleCLK( LPC_PWM1, SYSCTL_CLKDIV_8 );
	PWM1Freq = Chip_Clock_GetPeripheralClockRate(SYSCTL_PCLK_PWM1);//step 2: PCLKSEL0 para PWM1

	/* Seteo la salida PWM1.1 que es la unica que utilizaremos */
	PWM1_SelectChannel(LPC_PWM1, 1);		// Establece PWM1.1 en P2.0
	PWM1_ControlChannel(LPC_PWM1, 1, 0, 0); // Habilito la salida PWM1.1 en modo single edge(solo borde)
	PWM1_ValuePrescale(LPC_PWM1, 0);	    // PRESCALER=100MHz/PR->Si PR=0 no utilizo el prescaler.
	LPC_PWM1->TC =0;
	PWM1_SetMatch(LPC_PWM1, 0, 4096);	    // MR0:Utilizo el valor maximo del ADC(2^12=4096)->4096 * 100MHz. MR0 establece el periodo.
	PWM1_SetMatch(LPC_PWM1, 1, 2048); 			// MR1:Seteo PWM1.1 en 0 para luego ir actualizandolo. MR1 establece el tiempo de trabajo.
	PWM1_ConfigMatch(LPC_PWM1, 0, 0, 1, 0); // Accion al llegar al match: Reset TC
	PWM1_EnableMatchValue(LPC_PWM1, 0);		// Habilito el nuevo valor que seteamos en PWM1.1
	PWM1_EnableMatchValue(LPC_PWM1, 1);		// Habilito el nuevo valor que seteamos en PWM1.1
	PWM1_ControlChannel(LPC_PWM1, 1, 0, 1); // Habilito la salida PWM1.1 en modo single edge(solo borde)
	PWM1_ResetCounters(LPC_PWM1);			// Reseteo TC y PR para el conteo
	LPC_PWM1->MCR =2;						// Accion al llegar al match: Reset TC
	PWM1_EnableCounters(LPC_PWM1);			// Habilito el TC y el PC para el conteo y activo el modo PWM.

}

void UpdatePWM(unsigned int pulseWidth)
{
	/* Cargo el dato del ADC a las distintas salidas PWM para variar la velocidad de giro */
	PWM1_SetMatch(LPC_PWM1, 1, pulseWidth);		// Cargo el nuevo valor obtenido del adc a PWM1.1
	PWM1_EnableMatchValue(LPC_PWM1, 1);			// Habilito el nuevo valor que seteamos en PWM1.1
}

