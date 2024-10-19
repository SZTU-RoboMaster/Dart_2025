    //
// Created by 小新 on 2024/2/18.
//

#include "Decode.h"

fifo_s_t usb_fifo;                              // usb fifo 控制结构体
uint8_t usb_fifo_buf[512];                      // usb fifo环形缓存区
unpack_data_t decode_unpack;                    // 解包结构体
frame_header_struct_t decode_receive_header;    // 解包数据帧头
robot_ctrl_info_t robot_ctrl;                   // 控制结构体

void decode_task(void const * argument){
    // USB FIFO 初始化
    fifo_s_init(&usb_fifo, usb_fifo_buf, 512);
    while (1){
        // 进行数据解码
        decode_unpack_fifo_data();
        osDelay(1);
    }
}

// USB接受中断
void usb_receiver(uint8_t *buf, uint32_t len){
    // 将数据写入usb_fifo中
    fifo_s_puts(&usb_fifo, (char*)buf, len);
}

void decode_unpack_fifo_data(){
    uint8_t byte=0;                     // 单字节
    uint8_t SOF = HEADER_SOF;           // 帧头
    unpack_data_t *p = &decode_unpack;  // 解包指针
    while ( fifo_s_used(&usb_fifo) ){
        // 获取单字节数据
        byte = fifo_s_get(&usb_fifo);
        switch(p->unpack_step){
            case STEP_HEADER_SOF:{
                // 查找帧头
                if(byte == SOF){
                    p->unpack_step = STEP_LENGTH_LOW;
                    p->protocol_packet[p->index++] = byte;
                } else{
                    p->index = 0;
                }
            }break;
            case STEP_LENGTH_LOW:{
                /* 接收数据低字节 */
                p->data_len = byte;
                p->protocol_packet[p->index++] = byte;
                p->unpack_step = STEP_LENGTH_HIGH;
            }break;
            case STEP_LENGTH_HIGH:{
                /* 接收数据高字节 */
                p->data_len |= (byte << 8);             // 拼接数据长度
                p->protocol_packet[p->index++] = byte;
                // 数据长度小于 (128-帧头数据长度)
                if(p->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN)){
                    /* 获取包序 */
                    p->unpack_step = STEP_FRAME_SEQ;
                } else{
                    /* 重新寻找帧头 */
                    p->unpack_step = STEP_HEADER_SOF;
                    p->index = 0;
                }
            }break;
            case STEP_FRAME_SEQ:{
                /* 记录协议序列号,进行CRC8验证 */
                p->protocol_packet[p->index++] = byte;
                p->unpack_step = STEP_HEADER_CRC8;
            }break;
            case STEP_HEADER_CRC8:{
                p->protocol_packet[p->index++] = byte;
                // 目前长度等于帧头长度
                if (p->index == REF_PROTOCOL_HEADER_SIZE){
                    /* 进行CRC8校验 */
                    if (verify_CRC8_check_sum(p->protocol_packet, REF_PROTOCOL_HEADER_SIZE)){
                        p->unpack_step = STEP_DATA_CRC16;
                    } else{
                        p->unpack_step = STEP_HEADER_SOF;
                        p->index = 0;
                    }
                }
            }break;
            case STEP_DATA_CRC16:{
                /* 循环获取数据 */
                if (p->index < (REF_HEADER_CRC_CMDID_LEN + p->data_len))
                    p->protocol_packet[p->index++] = byte;

                /* 进行CRC16校验 */
                if (p->index >= (REF_HEADER_CRC_CMDID_LEN + p->data_len)){
                    /* 重新设置为初始状态 */
                    p->unpack_step = STEP_HEADER_SOF;
                    p->index = 0;

                    /* 进行CRC16校验 */
                    if (verify_CRC16_check_sum(p->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p->data_len)){
                        // 开始解析数据
                        decode_data_solve(p->protocol_packet);
                    }
                }
            }break;
            default:
            {
                /* 解包失败,重新初始化 */
                p->unpack_step = STEP_HEADER_SOF;
                p->index = 0;
                break;
            }
        }
    }
}

uint16_t decode_data_solve(uint8_t *frame){
    uint8_t index = 0;
    uint16_t cmd_id = 0;
    /* 提取帧头数据 */
    memcpy(&decode_receive_header, frame, sizeof(frame_header_struct_t));
    index += sizeof(frame_header_struct_t);
    /* 提取CMD_ID */
    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id){
        //接受控制码对应信息包
        case CHASSIS_CTRL_CMD_ID:{
            memcpy(&robot_ctrl, frame + index, sizeof(robot_ctrl_info_t));
            // 离线检查部分没有
            break;
        }
        default:{
            break;
        }
    }
    index += decode_receive_header.data_length + 2;
    return index;
}