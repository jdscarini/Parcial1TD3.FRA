
#include "timer.h"

void initTimer(void)
{
	/* Inicializo el Timer 2 */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_TIMER2);
	Chip_Clock_SetPCLKDiv(SYSCTL_PCLK_TIMER2,SYSCTL_CLKDIV_8);
	Chip_IOCON_PinMux(LPC_IOCON,0,5,IOCON_MODE_INACT,IOCON_FUNC3);

	Chip_TIMER_Init(LPC_TIMER2);
	Chip_TIMER_PrescaleSet(LPC_TIMER2, 0);

	/* El match 0 indica que el motor esta detenido si la cuenta llega a 1000000 */
	Chip_TIMER_SetMatch(LPC_TIMER2, 0, 1000000);
	Chip_TIMER_MatchEnableInt(LPC_TIMER2, 0);
	Chip_TIMER_ResetOnMatchEnable(LPC_TIMER2, 0);

	/* El pin de capture indica cuando el encoder pasa por una ranura */
	Chip_TIMER_CaptureRisingEdgeEnable(LPC_TIMER2, 1);
	Chip_TIMER_CaptureEnableInt(LPC_TIMER2, 1);
	Chip_TIMER_TIMER_SetCountClockSrc(LPC_TIMER2, TIMER_CAPSRC_RISING_PCLK, 0);
	NVIC_EnableIRQ(TIMER2_IRQn);
	Chip_TIMER_Reset(LPC_TIMER2);
	Chip_TIMER_Enable(LPC_TIMER2);
}

uint32_t Velocidad_RPM(void)
{
	uint32_t frec, vel, capture;
	/* Obtengo la frecuencia del contador del Timer 2 */
	frec = Chip_Clock_GetPeripheralClockRate(SYSCTL_PCLK_TIMER2);

	/* Obtengo el valor de la cuenta cuando llego una ranura del encoder */
	capture = Chip_TIMER_ReadCapture(LPC_TIMER2, 1);

	/* Multiplico por las 32 ranuras */
	capture = capture << 6;

	/* Calculo la velocidad en RPM */
	vel = (frec*60)/capture;
	return vel;
}
