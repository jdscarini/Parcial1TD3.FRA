#include "incapture.h"
#include "pwm.h"

/*Using CCLK = 100Mhz and PCLK_TIMER2 = 100Mhz.*/
void init_incapture(void){
	LPC_SYSCTL->PCONP |= (1<<22); //Power-Up Timer2 module. It is disabled by default.
	LPC_SYSCTL->PCLKSEL[1] |= (1<<12); //Set bits[13:12] = [01] to select PCLK_TIMER2 = CCLK i.e. 100Mhz in our case.
	LPC_IOCON->PINSEL[0] |= (1<<9) | (1<<8); //Set Bits[9:8] = [11] to Select CAP2.0 for P0.4
	LPC_TIMER2->CTCR = 0x1; //Increment TC on rising edges of External Signal for CAP2.0
	LPC_TIMER2->PR = 0; //Using lowest PR gives most accurate results
	LPC_TIMER2->CCR = 0x0; //Must be [000] for selected CAP input
	LPC_TIMER2->TCR = 0x2; //Reset & Disable Timer2 Initially
}

int get_capture(void){
	uint32_t pulses = 0;
	LPC_TIMER2->TCR = 0x1; //Start Timer2
	delay_ms(1000); //'Gate' signal for defined Time (1 second)
	LPC_TIMER2->TCR = 0x0; //Stop Timer2

	pulses = LPC_TIMER2->TC; //Read current value in TC, which contains  no.of. pulses counted in 1s
	LPC_TIMER2->TCR = 0x2; //Reset Timer2 TC
	return pulses;
}
