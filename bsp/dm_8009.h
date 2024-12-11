//
// Created by 86134 on 2024/12/11.
//

#ifndef DART_C_DM_8009_H
#define DART_C_DM_8009_H

#endif //DART_C_DM_8009_H
#ifndef DM_6006_H
#define DM_6006_H

#include "stdint-gcc.h"
#include "can_receive.h"
#include "dart.h"

struct Dm8009 {
    uint32_t id;
    float pos_r;
    float angular_vel;
    float torque;
    uint32_t last_heartbeat_timestamp_ms;
};

void dm8009_init(struct Dm4310 *motor, uint32_t device_id);

void set_dm8009_MIT(CAN_TYPE can_type, can_msg_id_e CMD_ID, float pos, float speed, float kp, float kd, float torque);

void set_dm8009_enable(CAN_TYPE can_type, can_msg_id_e CMD_ID);

void set_dm8009_disable(CAN_TYPE can_type, can_msg_id_e CMD_ID);

void set_dm8009_pos_speed(CAN_TYPE can_type, can_msg_id_e CMD_ID, float pos_rad, float speed_rps);

void dm8009_can_msg_unpack(uint32_t id, uint8_t data[]);

#endif //DM_6006_H
