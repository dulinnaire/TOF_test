//
// Created by 81301 on 2025/1/19.
//

#include "can.h"
#include "gpio.h"
#include "stm32f4xx.h"
#include "tim.h"
#include "tof.h"

#include <string.h>

extern uint16_t distance_right; // 距离，16位，单位mm
extern uint16_t distance_left;

uint32_t can_tx_mailbox;
uint8_t can_tx_data[8] = { 0 };
extern CAN_TxHeaderTypeDef can_tx_header;

struct TOFData {
    uint16_t right_dis; // [mm]
    uint16_t left_dis; // [mm]
    uint8_t reserved[4];
} tof_data;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim == &htim10) {
        tof_data.right_dis = distance_right;
        tof_data.left_dis = distance_left;
        memcpy(can_tx_data, &tof_data, 8);
        if (HAL_GetTick() % 2 == 0) {
            HAL_CAN_AddTxMessage(&hcan1, &can_tx_header, can_tx_data, &can_tx_mailbox);
        }
    }
}
