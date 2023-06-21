/*
 * PWM.h
 *
 *  Created on: 8 feb. 2022
 *      Author: Gustavo
 */

#ifndef PWM_H_
#define PWM_H_

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <chip.h>
#include <pwm1_17xx_40xx.h>
#include <timer_17xx_40xx.h>
#include <clock_17xx_40xx.h>
#include <chip_lpc175x_6x.h>
#include <iocon_17xx_40xx.h>
#include <sysctl_17xx_40xx.h>

void initPWM(void);
void UpdatePWM(unsigned int pulseWidth);

#endif /* PWM_H_ */
