
#ifndef ADC_H_
#define ADC_H_

#include "chip.h"

#define ADC_CH0_PORT  0
#define ADC_CH0_PIN  23

#define ADC_CH1_PORT  0
#define ADC_CH1_PIN  24

#define ADC_CH2_PORT  0
#define ADC_CH2_PIN  25

#define ADC_CH3_PORT  0
#define ADC_CH3_PIN  26

#define ADC_CH4_PORT  1
#define ADC_CH4_PIN  30

#define ADC_CH5_PORT  1
#define ADC_CH5_PIN  31

#define ADC_CH6_PORT  0
#define ADC_CH6_PIN   3

#define ADC_CH7_PORT  0
#define ADC_CH7_PIN   2


void init_adc(void);

void init_dac(void);

#endif /* ADC_H_ */
