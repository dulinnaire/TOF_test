#ifndef CALLBACK_H
#define CALLBACK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usart.h"

// UART idle callback. Called in stm32f4xx_it.c USARTx_IRQHandler()
// UART空闲中断处理，在stm32f4xx_it.c的USARTx_IRQHandler()函数中调用
void User_UART_IdleHandler(UART_HandleTypeDef* huart);

#ifdef __cplusplus
}
#endif

#endif //CALLBACK_H
