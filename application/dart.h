#ifndef DEMO1_DART_H
#define DEMO1_DART_H
#include "can_receive.h"
#include "PID.h"
#include "remote.h"
#include "AHRS.h"
#include "stdbool.h" //todo 引入了bool类型

#define DART_TASK_INIT_TIME 201

//YAW轴角度环PID
#define YAW_ANGLE_PID_KP     1.0f
#define YAW_ANGLE_PID_KI     1.0f
#define YAW_ANGLE_PID_KD     1.0f
#define YAW_ANGLE_MAX_OUT    1.0f
#define YAW_ANGLE_MAX_IOUT   1.0f
//YAW轴速度环PID
#define YAW_SPEED_PID_KP     1.0f
#define YAW_SPEED_PID_KI     1.0f
#define YAW_SPEED_PID_KD     1.0f
#define YAW_SPEED_MAX_OUT    1.0f
#define YAW_SPEED_MAX_IOUT   1.0f

//换弹电机角度PID
#define TURN_ANGLE_PID_KP     1.0f
#define TURN_ANGLE_PID_KI     1.0f
#define TURN_ANGLE_PID_KD     1.0f
#define TURN_ANGLE_MAX_OUT    1.0f
#define TURN_ANGLE_MAX_IOUT   1.0f
//换弹电机速度PID
#define TURN_SPEED_PID_KP     1.0f
#define TURN_SPEED_PID_KI     1.0f
#define TURN_SPEED_PID_KD     1.0f
#define TURN_SPEED_MAX_OUT    1.0f
#define TURN_SPEED_MAX_IOUT   1.0f

//推动电机角度PID
#define DRIVE_ANGLE_PID_KP     100.0f
#define DRIVE_ANGLE_PID_KI     100.0f
#define DRIVE_ANGLE_PID_KD     100.0f
#define DRIVE_ANGLE_MAX_OUT    100.0f
#define DRIVE_ANGLE_MAX_IOUT   100.0f
//推动电机速度PID
#define DRIVE_SPEED_PID_KP     1.0f
#define DRIVE_SPEED_PID_KI     1.0f
#define DRIVE_SPEED_PID_KD     1.0f
#define DRIVE_SPEED_MAX_OUT    1.0f
#define DRIVE_SPEED_MAX_IOUT   1.0f

//扳机移动电机角度PID
#define TRIGGER_MOVE_ANGLE_PID_KP     1.0f
#define TRIGGER_MOVE_ANGLE_PID_KI     1.0f
#define TRIGGER_MOVE_ANGLE_PID_KD     1.0f
#define TRIGGER_MOVE_ANGLE_MAX_OUT    1.0f
#define TRIGGER_MOVE_ANGLE_MAX_IOUT   1.0f
//扳机移动电机速度PID
#define TRIGGER_MOVE_SPEED_PID_KP     1.0f
#define TRIGGER_MOVE_SPEED_PID_KI     1.0f
#define TRIGGER_MOVE_SPEED_PID_KD     1.0f
#define TRIGGER_MOVE_SPEED_MAX_OUT    1.0f
#define TRIGGER_MOVE_SPEED_MAX_IOUT   1.0f

//推弹角度电机角度PID
#define THRUST_ANGLE_ANGLE_PID_KP     1.0f
#define THRUST_ANGLE_ANGLE_PID_KI     1.0f
#define THRUST_ANGLE_ANGLE_PID_KD     1.0f
#define THRUST_ANGLE_ANGLE_MAX_OUT    1.0f
#define THRUST_ANGLE_ANGLE_MAX_IOUT   1.0f
//推弹角度电机速度PID
#define THRUST_ANGLE_SPEED_PID_KP     1.0f
#define THRUST_ANGLE_SPEED_PID_KI     1.0f
#define THRUST_ANGLE_SPEED_PID_KD     1.0f
#define THRUST_ANGLE_SPEED_MAX_OUT    1.0f
#define THRUST_ANGLE_SPEED_MAX_IOUT   1.0f

//推弹移动电机角度PID
#define THRUST_MOVE_ANGLE_PID_KP     1.0f
#define THRUST_MOVE_ANGLE_PID_KI     1.0f
#define THRUST_MOVE_ANGLE_PID_KD     1.0f
#define THRUST_MOVE_ANGLE_MAX_OUT    1.0f
#define THRUST_MOVE_ANGLE_MAX_IOUT   1.0f
//推弹移动电机速度PID
#define THRUST_MOVE_SPEED_PID_KP     1.0f
#define THRUST_MOVE_SPEED_PID_KI     1.0f
#define THRUST_MOVE_SPEED_PID_KD     1.0f
#define THRUST_MOVE_SPEED_MAX_OUT    1.0f
#define THRUST_MOVE_SPEED_MAX_IOUT   1.0f

/******************** extern *******************/

enum Dart_Mode{
    DART_RELAX=0,
    DART_BACK,
    DART_CONTROL,
    DART_GOAL_SET,
    DART_READY,
    DART_TRIGGER,
    DART_LAUNCH
};

enum Fire_Mode{
    FIRE_OFF=0,
    FIRE_ON=1
};

struct Launch_t{
    enum Fire_Mode mode;
    enum Fire_Mode last_mode;
    motor_3508_t push_motor_r; //todo 改成push_motor_r
    motor_3508_t push_motor_l;

    motor_6020_t turn_motor;//换弹电机
};

struct Gimbal_t{
    enum Dart_Mode mode;
    enum Dart_Mode last_mode;
    motor_6020_t yaw;//yaw轴电机  //rodo 改成motor_yaw
};

struct Thrust_t{
    motor_2006_t thrust_angle_motor;//推弹角度电机
    motor_2006_t thrust_move_motor;//推弹移动电机
    motor_2006_t trigger_motor;//扳机移动
};

struct All_Flag  //todo 改成bool吧 并且改成枚举 //没太懂是什么意思
{
                    // bool is_back_drive_ok  复制用ture 和 false
    bool is_back_drive_ok;
    //bool is_turn_angle
    bool is_turn_angle_ok;
    //bool thrust_angle;
    bool is_thrust_angle_ok;
    bool is_thrust_move_ok;
    bool is_ready_ok;
    bool is_back_ok;
    bool is_trigger_move_ok;
    bool is_ready_trigger_move_ok;
};

enum trigger_angle
{
    front=0,//前哨站
    base//基地
};

enum thrust_motor_angle_mode
{
    free_mode=0,
    work_mode
};


extern void dart_task(void const*pvParameters);


#endif //DEMO1_DART_H
