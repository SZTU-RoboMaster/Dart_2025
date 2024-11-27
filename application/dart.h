#ifndef DEMO1_DART_H
#define DEMO1_DART_H
#include "can_receive.h"
#include "PID.h"
#include "remote.h"
#include "AHRS.h"
#include "stdbool.h"

#define DART_TASK_INIT_TIME 201

//YAW轴角度环PID
#define YAW_ANGLE_PID_KP     50.0f//8
#define YAW_ANGLE_PID_KI     0.001f//0
#define YAW_ANGLE_PID_KD     800.0f//235
#define YAW_ANGLE_MAX_OUT    3000.0f
#define YAW_ANGLE_MAX_IOUT   3000.0f
//YAW轴速度环PID
#define YAW_SPEED_PID_KP     150.0f//120
#define YAW_SPEED_PID_KI     0.0f//0.0f
#define YAW_SPEED_PID_KD     0.0f//30
#define YAW_SPEED_MAX_OUT    25000.0f
#define YAW_SPEED_MAX_IOUT   6000.0f

//换弹电机角度PID
#define TURN_ANGLE_PID_KP     25.0f//5
#define TURN_ANGLE_PID_KI     0.5f//0.1
#define TURN_ANGLE_PID_KD     10.0f//130
#define TURN_ANGLE_MAX_OUT    360.0f
#define TURN_ANGLE_MAX_IOUT   100.0f
//换弹电机速度PID
#define TURN_SPEED_PID_KP     10.0f//80
#define TURN_SPEED_PID_KI     0.0f//0.02
#define TURN_SPEED_PID_KD     0.0f//55
#define TURN_SPEED_MAX_OUT    15000.0f
#define TURN_SPEED_MAX_IOUT   500.0f

//推动电机角度PID
#define DRIVE_ANGLE_PID_KP     1600.0f//1600
#define DRIVE_ANGLE_PID_KI     0.0f//1
#define DRIVE_ANGLE_PID_KD     0.0f
#define DRIVE_ANGLE_MAX_OUT    3000.0f
#define DRIVE_ANGLE_MAX_IOUT   3000.0f
//推动电机速度PID
#define DRIVE_SPEED_PID_KP     10.0f//10
#define DRIVE_SPEED_PID_KI     0.0f
#define DRIVE_SPEED_PID_KD     0.0f
#define DRIVE_SPEED_MAX_OUT    16000.0f
#define DRIVE_SPEED_MAX_IOUT   6000.0f

//扳机移动电机角度PID
#define TRIGGER_MOVE_ANGLE_PID_KP     10000.0f
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
#define THRUST_ANGLE_ANGLE_PID_KP     300.0f
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


#endif //DEMO1_DART_H
