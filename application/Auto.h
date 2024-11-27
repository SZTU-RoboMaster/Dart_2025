//
// Created by 86134 on 2024/11/23.
//

#ifndef DEMO1_AUTO_H
#define DEMO1_AUTO_H
#include "struct_typedef.h"

#define AUTO_TASK_INIT_TIME 7200
extern void Auto_task(void const* pvParameters);
#define VISION_BUFFER_SIZE 200 //字节缓冲区长度
#define VISION_BUFFER_SEND 20

struct Vision_frame
{
    uint8_t head;
    uint8_t cmd;
};

struct Vision_info_get
{
    struct Vision_frame frame_header;
    union {
        uint8_t data[4];
        fp32 value;
    }yaw;
    int8_t target_lock;
    union {
        uint8_t data[2];
        uint16_t len;
    }data_length;
    uint8_t seq;
};

#endif //DEMO1_AUTO_H
