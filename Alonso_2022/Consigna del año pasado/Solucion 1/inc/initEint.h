#include<Chip.h>
#include<main.h>
#include "chip.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define Puerto_encoder1 0
#define Pin_encoder1 10
#define Puerto_encoder2 0
#define Pin_encoder2 11

#define Puerto_boton_der 0
#define Pin_boton_der 17
#define Puerto_boton_izq 0
#define Pin_boton_izq 18

void initEint(void);
