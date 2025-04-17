#include "callback.h"
#include "can.h"
#include "gpio.h"
#include "stm32f4xx.h"
#include "tim.h"
#include "tof.h"
#include <string.h>

// 板上UART2 4pin: USART1 右侧tof
// 板上UART1 3pin: USART6 左侧tof
TOF tof_left(&huart6);
TOF tof_right(&huart1);

uint32_t can_tx_mailbox;
uint8_t can_tx_data[8] = { 0 };
extern CAN_TxHeaderTypeDef can_tx_header;

struct TOFData {
    uint16_t right_dis; // [mm]
    uint16_t left_dis; // [mm]
    uint8_t reserved[4];
} tof_data;

bool tof_startup = false;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim == &htim10) {
        if (!tof_startup) {
            tof_left.init();
            tof_right.init();
            tof_startup = true;
        }
        tof_left.handle();
        tof_right.handle();

        tof_data.left_dis = tof_left.connect_.check() ? tof_left.get_distance() : 0;
        tof_data.right_dis = tof_right.connect_.check() ? tof_right.get_distance() : 0;
        memcpy(can_tx_data, &tof_data, 8);
        if (HAL_GetTick() % 2 == 0) {
            HAL_CAN_AddTxMessage(&hcan1, &can_tx_header, can_tx_data, &can_tx_mailbox);
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) //接收回调函数
{
    if (tof_left.uart_check(huart)) {
        tof_left.rx_callback();
    }
    if (tof_right.uart_check(huart)) {
        tof_right.rx_callback();
    }
}

// UART idle callback. Called in stm32f4xx_it.c USARTx_IRQHandler()
// UART空闲中断处理，在stm32f4xx_it.c的USARTx_IRQHandler()函数中调用
void User_UART_IdleHandler(UART_HandleTypeDef* huart) {
    // Judge if idle enabled. 判断空闲中断是否使能
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET) {
        // Clear idle flag. 清除空闲中断标记
        __HAL_UART_CLEAR_IDLEFLAG(huart);
        // idle中断回调
        if (tof_left.uart_check(huart)) {
            tof_left.idle_callback();
        }
        if (tof_right.uart_check(huart)) {
            tof_right.idle_callback();
        }
    }
}
