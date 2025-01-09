#ifndef DEMO1_DART_H
#define DEMO1_DART_H
#include "can_receive.h"
#include "PID.h"
#include "remote.h"
#include "AHRS.h"
#include "stdbool.h"

#define DART_TASK_INIT_TIME 201

//YAW轴角度环PID
#define YAW_ANGLE_PID_KP     300.0f//100
#define YAW_ANGLE_PID_KI     0.002f//0.1
#define YAW_ANGLE_PID_KD     400.0f//900
#define YAW_ANGLE_MAX_OUT    3000.0f
#define YAW_ANGLE_MAX_IOUT   3000.0f
//YAW轴速度环PID
#define YAW_SPEED_PID_KP     3.0f//5
#define YAW_SPEED_PID_KI     0.0f//0.0f
#define YAW_SPEED_PID_KD     0.0f//30
#define YAW_SPEED_MAX_OUT    25000.0f
#define YAW_SPEED_MAX_IOUT   6000.0f

//换弹电机角度PID
#define TURN_ANGLE_PID_KP_3     5.5f//5.5
#define TURN_ANGLE_PID_KI_3     0.04f//0.04
#define TURN_ANGLE_PID_KD_3     400.0f//400
#define TURN_ANGLE_PID_KP_2     10.0f//125
#define TURN_ANGLE_PID_KI_2     0.1f//1
#define TURN_ANGLE_PID_KD_2    400.0f//400
#define TURN_ANGLE_PID_KP_1     8.0f//125
#define TURN_ANGLE_PID_KI_1     0.05f//1
#define TURN_ANGLE_PID_KD_1     400.0f//400
#define TURN_ANGLE_PID_KP_0     15.0f//125
#define TURN_ANGLE_PID_KI_0     0.05f//1
#define TURN_ANGLE_PID_KD_0     400.0f//400
#define TURN_ANGLE_MAX_OUT    360.0f
#define TURN_ANGLE_MAX_IOUT   1000.0f
//换弹电机速度PID
#define TURN_SPEED_PID_KP_2     15.0f//7.5
#define TURN_SPEED_PID_KP_3     20.0f
#define TURN_SPEED_PID_KI     0.0f//0
#define TURN_SPEED_PID_KD     0.0f//0
#define TURN_SPEED_MAX_OUT    15000.0f
#define TURN_SPEED_MAX_IOUT   500.0f

//推动电机角度PID
#define DRIVE_ANGLE_left_PID_KP     300.0f//500
#define DRIVE_ANGLE_left_PID_KI     0.0f//0
#define DRIVE_ANGLE_left_PID_KD     0.0f
#define DRIVE_ANGLE_right_PID_KP     0.0f//500
#define DRIVE_ANGLE_right_PID_KI     0.0f//1
#define DRIVE_ANGLE_right_PID_KD     0.0f
#define DRIVE_ANGLE_MAX_OUT    3000.0f
#define DRIVE_ANGLE_MAX_IOUT   3000.0f
//推动电机速度PID
#define DRIVE_SPEED_left_PID_KP     10.0f//10
#define DRIVE_SPEED_left_PID_KI     0.0f
#define DRIVE_SPEED_left_PID_KD     0.0f
#define DRIVE_SPEED_right_PID_KP     0.0f//10
#define DRIVE_SPEED_right_PID_KI     0.0f
#define DRIVE_SPEED_right_PID_KD     0.0f
#define DRIVE_SPEED_MAX_OUT    16000.0f
#define DRIVE_SPEED_MAX_IOUT   6000.0f

//扳机移动电机角度PID
#define TRIGGER_MOVE_ANGLE_PID_KP     10000.0f//10000
#define TRIGGER_MOVE_ANGLE_PID_KI     1.0f
#define TRIGGER_MOVE_ANGLE_PID_KD     0.0f
#define TRIGGER_MOVE_ANGLE_MAX_OUT    10000.0f
#define TRIGGER_MOVE_ANGLE_MAX_IOUT   80.0f
//扳机移动电机速度PID
#define TRIGGER_MOVE_SPEED_PID_KP     15.0f
#define TRIGGER_MOVE_SPEED_PID_KI     0.0f
#define TRIGGER_MOVE_SPEED_PID_KD     0.0f
#define TRIGGER_MOVE_SPEED_MAX_OUT    9000.0f
#define TRIGGER_MOVE_SPEED_MAX_IOUT   1000.0f

//推弹角度电机角度PID
#define THRUST_ANGLE_ANGLE_PID_KP     100.0f//100
#define THRUST_ANGLE_ANGLE_PID_KI     0.0f
#define THRUST_ANGLE_ANGLE_PID_KD     0.0f
#define THRUST_ANGLE_ANGLE_MAX_OUT    3000.0f
#define THRUST_ANGLE_ANGLE_MAX_IOUT   80.0f
//推弹角度电机速度PID
#define THRUST_ANGLE_SPEED_PID_KP     20.0f
#define THRUST_ANGLE_SPEED_PID_KI     0.0f
#define THRUST_ANGLE_SPEED_PID_KD     0.0f
#define THRUST_ANGLE_SPEED_MAX_OUT    9000.0f
#define THRUST_ANGLE_SPEED_MAX_IOUT   1000.0f

//推弹移动电机角度PID
#define THRUST_MOVE_ANGLE_PID_KP     1500.0f//1500
#define THRUST_MOVE_ANGLE_PID_KI     0.0f
#define THRUST_MOVE_ANGLE_PID_KD     0.0f//34
#define THRUST_MOVE_ANGLE_MAX_OUT    3000.0f
#define THRUST_MOVE_ANGLE_MAX_IOUT   80.0f
//推弹移动电机速度PID
#define THRUST_MOVE_SPEED_PID_KP     15.0f//15
#define THRUST_MOVE_SPEED_PID_KI     0.0f
#define THRUST_MOVE_SPEED_PID_KD     0.0f
#define THRUST_MOVE_SPEED_MAX_OUT    9000.0f
#define THRUST_MOVE_SPEED_MAX_IOUT   1000.0f

#define YAW_BACK_ANGLE -9.448242f

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

struct Dm4310
{
    uint32_t id;
    fp32 pos_r;
    fp32 angular_vel;
    fp32 torque;
};

struct Launch_t{
    enum Fire_Mode mode;
    enum Fire_Mode last_mode;
    motor_3508_t push_motor_r;
    motor_3508_t push_motor_l;

    motor_6020_t turn_motor;//换弹电机
};


struct Gimbal_t{
    enum Dart_Mode mode;
    enum Dart_Mode last_mode;
    motor_6020_t motor_yaw;//yaw轴电机  //todo 改成motor_yaw
};

struct Thrust_t{
    motor_2006_t thrust_angle_motor;//推弹角度电机
    motor_2006_t thrust_move_motor;//推弹移动电机
    motor_2006_t trigger_motor;//扳机移动
};

struct All_Flag
{

    bool is_back_drive_ok;
    //bool is_turn_angle
    bool is_turn_angle_ok;
    //bool thrust_angle;
    bool is_thrust_angle_ok;
    bool is_thrust_move_ok;
    bool is_ready_ok;
    bool is_back_ok;
    bool is_trigger_move_ok;
    bool is_ready1_trigger_move_ok;
    bool is_ready1_drive_ok;
    bool is_ready1_trigger_on_ok;
    bool is_ready1_drive_init_ok;
    bool is_ready1_trigger_off_ok;
    bool is_back_turn_ok;
    bool is_ready2_ok;
    bool is_ready2_drive_load_ok;
    bool is_ready2_turn_init_ok;
    bool is_ready2_turn_load_ok;
    bool is_ready2_thrust_move_goal_ok;
    bool is_ready2_thrust_angle_goal_ok;
    bool is_ready2_thrust_move_back_ok;
    bool is_ready2_thrust_angle_back_ok;
    bool is_ready2_turn_end_ok;
    bool is_ready2_drive_goal_ok;
    bool is_ready2_drive_back_ok;
    bool is_ready2_thrust_back_ok;
    bool is_ready2_trigger_off_ok;
    bool is_ready2_turn_angle_ok_init;
    bool is_ready2_turn_angle_ok_load;
    bool is_ready2_turn_angle_ok_end;
    bool is_ready2_trigger_open_ok;
    bool is_ready2_turn_continue_ok;
    bool is_ready2_trigger_open_first;
    bool is_back_Drive_first;
    bool is_ready2_trigger_ok;
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

enum Dart_goal{
    GOAL_FRONT_STATION=1,//前哨站
    GOAL_BASE_STATION//基地
};


extern void dart_task(void const*pvParameters);

extern struct Launch_t launcher_dart;
extern struct Gimbal_t gimbal_dart;
extern struct Thrust_t thrust_motor;

#endif //DEMO1_DART_H
