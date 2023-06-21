#ifndef ADC_H_
#define ADC_H_

#include "chip.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/////////////////////////////////////ADC //////////////////////////////////////////////
#define PUERTOADC 0
#define PINADC 23
/////////////////////////////////////////////define TIMER1//////////////////////////////////////////////
#define TIMER_match1		1
#define MATCH1		1      //// cada un seg /// si es N>1 entonces timer < 1seg
//////variables adc

void initADC(void);
void ADC_IRQHandler(void);


#endif /* ADC_H_ */
