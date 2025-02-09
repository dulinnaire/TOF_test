//
// Created by 81301 on 2025/1/16.
//

#ifndef TOF_H
#define TOF_H

#include "stm32f4xx.h"
#include "usart.h"

extern uint16_t receive_cnt;
extern uint16_t distance;

#define USART_REC_LEN 200 //定义最大接收字节数 200

extern char USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符
extern uint16_t USART_RX_STA; //接收状态标记

#define ANGLE_PER_FRAME 12
#define HEADER 0x54
#define POINT_PER_PACK 12
#define VERLEN 0x2C //低五位是一帧数据接收到的点数，目前固定是12，高三位固定为1

typedef struct __attribute__((packed)) Point_Data {
    uint16_t distance; //距离
    uint8_t intensity; //置信度
} LidarPointStructDef;

typedef struct __attribute__((packed)) Pack_Data {
    uint8_t header;
    uint8_t ver_len;
    uint16_t temperature;
    uint16_t start_angle;
    LidarPointStructDef point[POINT_PER_PACK];
    uint16_t end_angle;
    uint16_t timestamp;
    uint8_t crc8;
} LiDARFrameTypeDef;

extern UART_HandleTypeDef huart1;

extern char rxdatabuffer;
void data_process(void);


#endif //TOF_H
