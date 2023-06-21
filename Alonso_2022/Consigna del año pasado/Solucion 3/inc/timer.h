/*
 * timer.h
 *
 *  Created on: 27 jun. 2022
 *      Author: leo_b
 */

#ifndef TIMER_H_
#define TIMER_H_

#include <chip.h>
#include <timer_17xx_40xx.h>
#include <clock_17xx_40xx.h>
#include <chip_lpc175x_6x.h>
#include <iocon_17xx_40xx.h>
#include <sysctl_17xx_40xx.h>

#define RANURAS_ENCODER 32

void initTimer(void);
uint32_t Velocidad_RPM(void);


#endif /* TIMER_H_ */
