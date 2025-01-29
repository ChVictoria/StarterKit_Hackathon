#include "hcsr04.h"

void HCSR04_Trigger(GPIO_TypeDef* Port, uint16_t Pin) {
	HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_RESET);
}
