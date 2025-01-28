#include "hcsr04.h"

const float SPEED_OF_SOUND = 343.0f;

void HCSR04_Trigger(GPIO_TypeDef* Port, uint16_t Pin) {
	HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_RESET);
}

float HCSR04_ListenEcho(GPIO_TypeDef* Port, uint16_t Pin) {
	while (HAL_GPIO_ReadPin(Port, Pin) == GPIO_PIN_RESET);

	uint32_t startTime = HAL_GetTick();

	while (HAL_GPIO_ReadPin(Port, Pin) == GPIO_PIN_SET);

	uint32_t endTime = HAL_GetTick();

	return ((endTime - startTime) * SPEED_OF_SOUND) / 2 / 1000.0f;
}