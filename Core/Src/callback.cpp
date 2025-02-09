//
// Created by 81301 on 2025/1/19.
//

#include "gpio.h"
#include "stm32f4xx.h"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == KEY_Pin) {
        while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET) {
        }
        HAL_GPIO_TogglePin(LASER_GPIO_Port, LASER_Pin);
    }
}
