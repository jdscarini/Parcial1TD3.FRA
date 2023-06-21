
#ifndef PWM_H_
#define PWM_H_

#include "chip.h"

#define my_PWM1 	          LPC_PWM1
#define my_PWM1_IRQ           PWM1_IRQn
#define my_PWM1_SPEED 		  SYSCTL_PCLK_PWM1 //es el 6to en la lista
#define HANDLER_PWM1    	  PWM1_IRQHandler

//DARO: modifique para trabajar con el Timer1
#define my_TIMER 	          LPC_TIMER1
#define my_TIMER_IRQ          TIMER1_IRQn
#define my_TIMER_SPEED 		  SYSCTL_PCLK_TIMER1 //es el 1ro en la lista
#define HANDLER_TIMER1    	  TIMER1_IRQHandler

//defines para funciones de configuracion de pwm
#define DISABLE_OUT 0
#define ENABLE_OUT 1
#define SINGLE_EDGE 0
#define DOUBLE_EDGE 1

#define PWMPRESCALE (30-1) //30 PCLK cycles to increment TC by 1 i.e. 1 Micro-second

#define MATCH0 0
#define MATCH1 1
#define MATCH2 2
#define MATCH3 3

#define TIMER_TICK_RATE_HZ (1)

#define PWM_PERIOD (1000) //valor en microsegundos
#define DUTY_CICLE (0)	//corregir de tal manera de que se pueda ingresar en porcentaje

void init_pwm(void);
void init_timer(void);
void delay_ms(uint32_t milisegundos);
void Ancho_pulso(uint16_t );

#endif /* PWM_H_ */
