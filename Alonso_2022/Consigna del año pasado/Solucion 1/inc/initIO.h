#ifndef INITIO_H_
#define INITIO_H_


#include<Chip.h>
#include<main.h>

#define Led_red 2,12
#define Led_verde 2,13
#define PIN_DRIVER_DER 0 ,4
#define PIN_DRIVER_IZQ 0 ,5
#define DERECHO		1
#define IZQUIERDO	2
#define APAGADO		3

void initIO(void);
void controlIO(uint8_t);



#endif
