//
// Created by 小新 on 2024/2/18.
//

#include "Packet.h"

// 数据帧尾
msg_end_info msg_end;
extern QueueHandle_t CDC_send_queue;    // 发送数据队列

// 将数据添加到队列中
void rm_queue_data(uint16_t cmd_id,void* buf,uint16_t len){
    uint16_t index = 0;
    uint8_t queue_data[128];        // 定义队列缓存数组
    // 将CMD_ID放入数组
    memcpy(queue_data,  (void*)&cmd_id, sizeof(uint16_t));
    index +=sizeof(uint16_t);
    // 添加数据
    memcpy(queue_data + index, (void*)buf, len);
    // 将数据放入队列
    xQueueSend(CDC_send_queue, queue_data, 50);
}

// 取出数据,进行协议序列化
void rm_dequeue_send_data(void* buf,uint16_t len) {
    // 取出CMD_ID
    uint16_t cmd_id;
    memcpy(&cmd_id,buf,sizeof(uint16_t));
    switch(cmd_id)
    {
        // 需要发送的数据包ID号
        case CHASSIS_ODOM_CMD_ID:
            encode_send_data(CHASSIS_ODOM_CMD_ID,((uint8_t*)buf+2),sizeof(chassis_odom_info_t));
            break;
        case CHASSIS_CTRL_CMD_ID:
            encode_send_data(CHASSIS_CTRL_CMD_ID,((uint8_t*)buf+2),sizeof(robot_ctrl_info_t ));
            break;
        case VISION_ID:
            encode_send_data(VISION_ID,((uint8_t*)buf+2),sizeof(vision_t));
            break;
    }
}

// RM协议序列化
void encode_send_data(uint16_t cmd_id, void* buf, uint16_t len){
    static uint8_t send_buf[128];   // 定义发送数据缓存数组
    uint16_t index = 0;             // 序号

    // 初始化帧头尾
    msg_end.end1=END1_SOF;
    msg_end.end2=END2_SOF;

    // 初始化帧头结构体
    frame_header_struct_t referee_send_header;  // 定义帧头结构体
    referee_send_header.SOF = HEADER_SOF;       // 数据帧起始字节
    referee_send_header.data_length = len;      // 数据帧中 data 的长度
    referee_send_header.seq++;                  // 包序号

    /* 生成CRC8校验 */
    append_CRC8_check_sum((uint8_t*)&referee_send_header, sizeof(frame_header_struct_t));
    memcpy(send_buf, (uint8_t*)&referee_send_header, sizeof(frame_header_struct_t));
    index += sizeof(frame_header_struct_t);
    // 填充CMD_ID
    memcpy(send_buf + index, (void*)&cmd_id, sizeof(uint16_t));
    index += sizeof(uint16_t);
    // 填充数据包
    memcpy(send_buf + index, (void*)buf, len);
    index += len;

    /* 生成CRC16校验 */
    append_CRC16_check_sum(send_buf, REF_HEADER_CRC_CMDID_LEN + len);
    index += sizeof(uint16_t);

    // 添加视觉解包帧尾
    memcpy(send_buf + index,(void*)&msg_end,sizeof(msg_end_info));
    index += sizeof(msg_end_info);

    //调用底层发送函数
    CDC_Transmit_FS(send_buf, index);
}