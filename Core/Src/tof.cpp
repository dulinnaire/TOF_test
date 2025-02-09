//
// Created by 81301 on 2025/1/16.
//
#include "tof.h"

static const uint8_t CRC_TABLE[256] = {
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c,
    0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
    0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea,
    0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62,
    0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d,
    0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4,
    0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89,
    0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f,
    0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e,
    0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7,
    0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
}; //用于crc校验的数组

uint16_t distance = 0;
uint8_t usart1_receive_buf[1]; //串口1接收中断数据存放的缓冲区
uint32_t time_now = 0, last_time = 0, interval = 0;

LiDARFrameTypeDef pack_data; //雷达接收的数据储存在这个变量之中

void data_process(void);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) //接收回调函数
{
    static uint8_t state = 0; //状态位
    static uint8_t crc = 0; //校验和
    static uint8_t cnt = 0; //用于一帧12个点的计数
    uint8_t temp_data;
    if (huart->Instance == USART1) {
        temp_data = usart1_receive_buf[0];
        if (state > 5) {
            if (state < 42) {
                if (state % 3 == 0) //一帧数据中的序号为6,9.....39的数据，距离值低8位
                {
                    pack_data.point[cnt].distance = (uint16_t)temp_data;
                    state++;
                    crc = CRC_TABLE[(crc ^ temp_data) & 0xff];
                } else if (state % 3 == 1) //一帧数据中的序号为7,10.....40的数据，距离值高8位
                {
                    pack_data.point[cnt].distance =
                        ((uint16_t)temp_data << 8) + pack_data.point[cnt].distance;
                    state++;
                    crc = CRC_TABLE[(crc ^ temp_data) & 0xff];
                } else //一帧数据中的序号为8,11.....41的数据，置信度
                {
                    pack_data.point[cnt].intensity = temp_data;
                    cnt++;
                    state++;
                    crc = CRC_TABLE[(crc ^ temp_data) & 0xff];
                }
            } else {
                switch (state) {
                    case 42:
                        pack_data.end_angle = (uint16_t)temp_data; //结束角度低8位
                        state++;
                        crc = CRC_TABLE[(crc ^ temp_data) & 0xff];
                        break;
                    case 43:
                        pack_data.end_angle =
                            ((uint16_t)temp_data << 8) + pack_data.end_angle; //结束角度高8位
                        state++;
                        crc = CRC_TABLE[(crc ^ temp_data) & 0xff];
                        break;
                    case 44:
                        pack_data.timestamp = (uint16_t)temp_data; //时间戳低8位
                        state++;
                        crc = CRC_TABLE[(crc ^ temp_data) & 0xff];
                        break;
                    case 45:
                        pack_data.timestamp =
                            ((uint16_t)temp_data << 8) + pack_data.timestamp; //时间戳高8位
                        state++;
                        crc = CRC_TABLE[(crc ^ temp_data) & 0xff];
                        break;
                    case 46:
                        pack_data.crc8 = temp_data; //雷达传来的校验和
                        if (pack_data.crc8 == crc) //校验正确
                        {
                            data_process(); //接收到一帧且校验正确可以进行数据处理
                        } else {
                            //校验不正确
                        }
                        //memset(&Pack_Data,0,sizeof(Pack_Data)*);//清零
                        crc = 0;
                        state = 0;
                        cnt = 0; //复位
                    default:
                        break;
                }
            }
        } else {
            switch (state) {
                case 0:
                    if (temp_data == HEADER) //头固定
                    {
                        pack_data.header = temp_data;
                        state++;
                        crc = CRC_TABLE[(crc ^ temp_data) & 0xff]; //开始进行校验
                    } else {
                        state = 0, crc = 0;
                    }
                    break;
                case 1:
                    if (temp_data == VERLEN) //测量的点数，目前固定
                    {
                        pack_data.ver_len = temp_data;
                        state++;
                        crc = CRC_TABLE[(crc ^ temp_data) & 0xff];
                    } else {
                        state = 0, crc = 0;
                    }
                    break;
                case 2:
                    pack_data.temperature =
                        (uint16_t)temp_data; //温度低8位，一共16位ADC，0--4096，无量纲
                    state++;
                    crc = CRC_TABLE[(crc ^ temp_data) & 0xff];
                    break;
                case 3:
                    pack_data.temperature =
                        ((uint16_t)temp_data << 8) + pack_data.temperature; //温度高8位
                    state++;
                    crc = CRC_TABLE[(crc ^ temp_data) & 0xff];
                    break;
                case 4:
                    pack_data.start_angle = (uint16_t)temp_data; //开始角度低8位，放大了100倍
                    state++;
                    crc = CRC_TABLE[(crc ^ temp_data) & 0xff];
                    break;
                case 5:
                    pack_data.start_angle = ((uint16_t)temp_data << 8) + pack_data.start_angle;
                    state++;
                    crc = CRC_TABLE[(crc ^ temp_data) & 0xff];
                    break;
                default:
                    break;
            }
        }
        HAL_UART_Receive_IT(
            &huart1,
            usart1_receive_buf,
            sizeof(usart1_receive_buf)
        ); //串口5回调函数执行完毕之后，需要再次开启接收中断等待下一次接收中断的发生
    }
}

int times = 1;

void data_process(void) //数据处理函数，完成一帧之后可进行数据处理
{
    //计算距离
    static uint8_t cnt = 0;
    static uint16_t count = 0;
    static uint32_t sum = 0;
    for (int i = 0; i < 12; i++) //12个点取平均
    {
        if (pack_data.point[i].distance != 0) //去除0的点
        {
            count++;
            sum += pack_data.point[i].distance;
        }
    }
    if (++cnt == times) //100个数据帧计算一次距离
    {
        // time_now = HAL_GetTick();
        // interval = time_now - last_time;
        // last_time = time_now;

        distance = sum / count;

        sum = 0;
        count = 0;
        cnt = 0;
    }
}
