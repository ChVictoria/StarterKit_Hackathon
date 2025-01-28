#ifndef HCSR04_H
#define HCSR04_H

#include "main.h"

void HCSR04_Trigger(GPIO_TypeDef* Port, uint16_t Pin);
float HCSR04_ListenEcho(GPIO_TypeDef* Port, uint16_t Pin);

#endif
