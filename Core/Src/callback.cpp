//
// Created by 81301 on 2025/1/19.
//

#include "can.h"
#include "gpio.h"
#include "stm32f4xx.h"
#include "tim.h"
#include "tof.h"

extern uint16_t distance_right; // 距离，16位，单位mm
extern uint16_t distance_left;

uint32_t can_tx_mailbox;
uint8_t can_tx_data[4] = { 0 };
extern CAN_TxHeaderTypeDef can_tx_header;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim == &htim10) {
        can_tx_data[0] = distance_right & 0xff; // 右侧距离低8位
        can_tx_data[1] = (distance_right >> 8) & 0xff; // 右侧距离高8位
        can_tx_data[2] = distance_left & 0xff; // 左侧距离低8位
        can_tx_data[3] = (distance_left >> 8) & 0xff; // 左侧距离高8位
        if (HAL_GetTick() % 2 == 0) {
            HAL_CAN_AddTxMessage(&hcan1, &can_tx_header, can_tx_data, &can_tx_mailbox);}
    }
}
