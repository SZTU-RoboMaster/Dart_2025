//
// Created by xhuanc on 2021/9/27.
//

#include "can_receive.h"
#include "cmsis_os.h"
#include "main.h"
#include "chassis.h"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
/******************** define *******************/

//电子数据解算
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

//电机总编码值的计算,do while作为保护性代码，防止在展开时被错误编译
#define get_motor_round_cnt(ptr)  \
    do{                            \
             if(ptr.ecd-ptr.last_ecd> 4192){ \
                ptr.round_cnt--;                    \
             }                   \
             else if(ptr.ecd-ptr.last_ecd< -4192)    \
             {                   \
                ptr.round_cnt++;            \
             }                   \
             ptr.total_ecd= ptr.round_cnt*8192+ptr.ecd-ptr.offset_ecd;\
                                 \
    }while(0)
/******************** variable *******************/

motor_measure_t motor_3508_dart[4];

motor_measure_t motor_3508_measure[4];//0-3 分别对应  RF,LF,LB,RB

motor_measure_t motor_yaw_measure;
motor_measure_t motor_pitch_measure;
motor_measure_t motor_turn_measure;

motor_measure_t motor_2006_measure[3];


#if isBalance
motor_measure_t motor_left_measure;
motor_measure_t motor_right_measure;
#endif

static CAN_TxHeaderTypeDef  tx_message;
static uint8_t              can_send_data[8];
extern chassis_t chassis;
//底盘电机的发送函数
void CAN_cmd_motor(CAN_TYPE can_type,can_msg_id_e CMD_ID,int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    tx_message.StdId = CMD_ID;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;
    can_send_data[0] = motor1 >> 8;
    can_send_data[1] = motor1;
    can_send_data[2] = motor2 >> 8;
    can_send_data[3] = motor2;
    can_send_data[4] = motor3 >> 8;
    can_send_data[5] = motor3;
    can_send_data[6] = motor4 >> 8;
    can_send_data[7] = motor4;

    if(can_type==CAN_1)
    {
        HAL_CAN_AddTxMessage(&hcan1, &tx_message, can_send_data, &send_mail_box);
    }
    else if(can_type==CAN_2)
    {
        HAL_CAN_AddTxMessage(&hcan2, &tx_message, can_send_data, &send_mail_box);
    }

}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;

    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if (hcan == &hcan1) {
        switch (rx_header.StdId) {


            case CAN_CHASSIS_3508_MOTOR_1: get_motor_measure(&motor_3508_dart[0], rx_data);

                break;
            case CAN_CHASSIS_3508_MOTOR_2: get_motor_measure(&motor_3508_dart[1], rx_data);

                break;
            case CAN_CHASSIS_3508_MOTOR_3: get_motor_measure(&motor_3508_dart[2], rx_data);

                break;
            case CAN_CHASSIS_3508_MOTOR_4: get_motor_measure(&motor_3508_dart[3], rx_data);
                break;

            default: {
                break;
            }
        }
    }
    if (hcan == &hcan2) {
        switch (rx_header.StdId){
            case CAN_TURN_6020: get_motor_measure(&motor_turn_measure, rx_data);  //换弹

                break;

            case CAN_GIMBAL_6020_YAW: get_motor_measure(&motor_yaw_measure, rx_data);
                break;

            case CAN_GIMBAL_6020_PITCH: get_motor_measure(&motor_pitch_measure, rx_data);
                break;

            case  CAN_UP_2006: get_motor_measure(&motor_2006_measure[0], rx_data);
                get_motor_round_cnt(motor_2006_measure[0]);
                break;

            default: {
                break;
            }
        }
   }

}

fp32 motor_ecd_to_rad_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    return relative_ecd * MOTOR_ECD_TO_RAD;
}


//计算距离零点的度数  -180-180
fp32 motor_ecd_to_angle_change(uint16_t ecd,uint16_t offset_ecd)
{
    int16_t tmp=0;
    if(offset_ecd>=4096)
    {
        if(ecd>offset_ecd-4096)
        {
            tmp=ecd-offset_ecd;
        }
        else
        {
            tmp=ecd+8192-offset_ecd;
        }
    }
    else{
        if(ecd>offset_ecd+4096)
        {
            tmp=ecd-8192-offset_ecd;
        }
        else
        {
            tmp=ecd-offset_ecd;
        }
    }
    return (fp32)tmp/8192.f*360;
}


