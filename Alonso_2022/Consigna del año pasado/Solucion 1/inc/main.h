
#ifndef MAIN_H_
#define MAIN_H_

#include "chip.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <initIO.h>
#include <adc.h>
#include <initPwm.h>
#include <initEint.h>

#define LEDON true
#define LEDOFF false
#define ON true
#define OFF false

#define DER 1
#define IZQ 2
#define CERO 3
#define SENTIDO_IZQ 1
#define SENTIDO_DER 2

#define BOTON_IZQUIERDO_PRESIONADO 3
#define BOTON_DERECHO_PRESIONADO 2

#define MOTOR_MOVIMIENTO 0
#define MOTOR_FRENADO 1

#define maxduty 800
#define minduty 200

#define maxadc 4096

#endif /* MAIN_H_ */



