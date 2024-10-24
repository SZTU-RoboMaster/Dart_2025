#ifndef DEMO1_DART_H
#define DEMO1_DART_H
#include "can_receive.h"
#include "PID.h"
#include "remote.h"
#include "AHRS.h"

#define DART_TASK_INIT_TIME 201

//YAW��ǶȻ�PID
#define YAW_ANGLE_PID_KP     1.0f
#define YAW_ANGLE_PID_KI     1.0f
#define YAW_ANGLE_PID_KD     1.0f
#define YAW_ANGLE_MAX_OUT    1.0f
#define YAW_ANGLE_MAX_IOUT   1.0f
//YAW���ٶȻ�PID
#define YAW_SPEED_PID_KP     1.0f
#define YAW_SPEED_PID_KI     1.0f
#define YAW_SPEED_PID_KD     1.0f
#define YAW_SPEED_MAX_OUT    1.0f
#define YAW_SPEED_MAX_IOUT   1.0f

//��������Ƕ�PID
#define TURN_ANGLE_PID_KP     1.0f
#define TURN_ANGLE_PID_KI     1.0f
#define TURN_ANGLE_PID_KD     1.0f
#define TURN_ANGLE_MAX_OUT    1.0f
#define TURN_ANGLE_MAX_IOUT   1.0f
//��������ٶ�PID
#define TURN_SPEED_PID_KP     1.0f
#define TURN_SPEED_PID_KI     1.0f
#define TURN_SPEED_PID_KD     1.0f
#define TURN_SPEED_MAX_OUT    1.0f
#define TURN_SPEED_MAX_IOUT   1.0f

//�ƶ�����Ƕ�PID
#define DRIVE_ANGLE_PID_KP     100.0f
#define DRIVE_ANGLE_PID_KI     100.0f
#define DRIVE_ANGLE_PID_KD     100.0f
#define DRIVE_ANGLE_MAX_OUT    100.0f
#define DRIVE_ANGLE_MAX_IOUT   100.0f
//�ƶ�����ٶ�PID
#define DRIVE_SPEED_PID_KP     1.0f
#define DRIVE_SPEED_PID_KI     1.0f
#define DRIVE_SPEED_PID_KD     1.0f
#define DRIVE_SPEED_MAX_OUT    1.0f
#define DRIVE_SPEED_MAX_IOUT   1.0f

//����ƶ�����Ƕ�PID
#define TRIGGER_MOVE_ANGLE_PID_KP     1.0f
#define TRIGGER_MOVE_ANGLE_PID_KI     1.0f
#define TRIGGER_MOVE_ANGLE_PID_KD     1.0f
#define TRIGGER_MOVE_ANGLE_MAX_OUT    1.0f
#define TRIGGER_MOVE_ANGLE_MAX_IOUT   1.0f
//����ƶ�����ٶ�PID
#define TRIGGER_MOVE_SPEED_PID_KP     1.0f
#define TRIGGER_MOVE_SPEED_PID_KI     1.0f
#define TRIGGER_MOVE_SPEED_PID_KD     1.0f
#define TRIGGER_MOVE_SPEED_MAX_OUT    1.0f
#define TRIGGER_MOVE_SPEED_MAX_IOUT   1.0f

//�Ƶ��Ƕȵ���Ƕ�PID
#define THRUST_ANGLE_ANGLE_PID_KP     1.0f
#define THRUST_ANGLE_ANGLE_PID_KI     1.0f
#define THRUST_ANGLE_ANGLE_PID_KD     1.0f
#define THRUST_ANGLE_ANGLE_MAX_OUT    1.0f
#define THRUST_ANGLE_ANGLE_MAX_IOUT   1.0f
//�Ƶ��Ƕȵ���ٶ�PID
#define THRUST_ANGLE_SPEED_PID_KP     1.0f
#define THRUST_ANGLE_SPEED_PID_KI     1.0f
#define THRUST_ANGLE_SPEED_PID_KD     1.0f
#define THRUST_ANGLE_SPEED_MAX_OUT    1.0f
#define THRUST_ANGLE_SPEED_MAX_IOUT   1.0f

//�Ƶ��ƶ�����Ƕ�PID
#define THRUST_MOVE_ANGLE_PID_KP     1.0f
#define THRUST_MOVE_ANGLE_PID_KI     1.0f
#define THRUST_MOVE_ANGLE_PID_KD     1.0f
#define THRUST_MOVE_ANGLE_MAX_OUT    1.0f
#define THRUST_MOVE_ANGLE_MAX_IOUT   1.0f
//�Ƶ��ƶ�����ٶ�PID
#define THRUST_MOVE_SPEED_PID_KP     1.0f
#define THRUST_MOVE_SPEED_PID_KI     1.0f
#define THRUST_MOVE_SPEED_PID_KD     1.0f
#define THRUST_MOVE_SPEED_MAX_OUT    1.0f
#define THRUST_MOVE_SPEED_MAX_IOUT   1.0f

/******************** extern *******************/
extern motor_measure_t motor_3508_dart1[2];
extern motor_measure_t motor_2006_dart1[3];
extern motor_measure_t motor_6020_dart1[2];

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
    motor_3508_t R;
    motor_3508_t L;

    motor_6020_t turn_motor;//�������
};

struct Gimbal_t{
    enum Dart_Mode mode;
    enum Dart_Mode last_mode;
    motor_6020_t yaw;//yaw����
};

struct Thrust_t{
    motor_2006_t thrust_angle_motor;//�Ƶ��Ƕȵ��
    motor_2006_t thrust_move_motor;//�Ƶ��ƶ����
    motor_2006_t trigger_motor;//����ƶ�
};

struct All_Flag
{
    uint8_t back_drive_ok;
    uint8_t turn_angle;
    uint8_t turn_angle_ok;
    uint8_t thrust_angle;
    uint8_t thrust_angle_ok;
    uint8_t thrust_move_ok;
    uint8_t ready_ok;
    uint8_t back_ok;
    uint8_t trigger_move_ok;
};



extern void dart_task(void const*pvParameters);


#endif //DEMO1_DART_H
