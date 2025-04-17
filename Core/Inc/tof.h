#ifndef TOF_H
#define TOF_H

#include "connect.h"
#include "stm32f4xx.h"
#include "usart.h"

#define USART_REC_LEN 200 //定义最大接收字节数 200
#define TOF_FRAME_LEN 47 //一帧数据的长度

#define ANGLE_PER_FRAME 12
#define HEADER 0x54
#define POINT_PER_PACK 12
#define VERLEN 0x2C //低五位是一帧数据接收到的点数，目前固定是12，高三位固定为1

class TOF {
public:
    Connect connect_;

    typedef struct {
        uint16_t distance; // 距离
        uint8_t intensity; // 置信度
    } __packed PointData;

    typedef struct {
        uint8_t header;
        uint8_t ver_len;
        uint16_t temperature;
        uint16_t start_angle;
        PointData point[POINT_PER_PACK];
        uint16_t end_angle;
        uint16_t timestamp;
        uint8_t crc8;
    } __packed TOFDataPack;

    TOFDataPack rx_data_pack_;

public:
    TOF(UART_HandleTypeDef* huart = nullptr);

    void init(void);
    void reset(void);
    void handle(void);

    bool crc_check();
    bool pack_header_check();

    bool uart_check(UART_HandleTypeDef* huart);
    void rx_callback(void);
    void idle_callback(void);

    uint16_t get_distance(void);

private:
    UART_HandleTypeDef* huart_;

    uint8_t rx_buffer_[USART_REC_LEN]; // 接收缓冲区
    uint8_t rx_data_[TOF_FRAME_LEN]; // 待校验数据
    volatile uint8_t rx_len_;

    uint16_t distance;
};

// extern uint8_t usart1_receive_buf[1]; //串口1接收中断数据存放的缓冲区
// extern uint32_t time_now, last_time, interval;
//
// extern LiDARFrameTypeDef pack_data_right; //雷达接收的数据储存在这个变量之中
//
// extern uint16_t distance_right;


#endif //TOF_H
