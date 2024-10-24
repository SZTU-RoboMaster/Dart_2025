#include <math.h>
#include "dart.h"
#include "cmsis_os.h"
#include "can_receive.h"
#include "user_lib.h"
#include "Atti.h"
#include "protocol_shaob.h"
#include "stdlib.h"

uint8_t direction=0;
uint8_t num_launched=0;//飞镖已发射数目
uint8_t dart_goal;//飞镖目标,1为前哨站,2为基地
uint8_t launcherable_num;//飞镖可发射数目1为两发,2为四发
struct Launch_t launcher1;
struct Gimbal_t gimbal_dart;
struct Thrust_t thrust_motor;
struct All_Flag flags;
motor_measure_t motor_3508_dart1[2];
motor_measure_t motor_2006_dart1[3];
motor_measure_t motor_6020_dart1[2];

extern RC_ctrl_t rc_ctrl;
extern Eulr_t Eulr;
extern fp32 INS_angle[3];
extern fp32 INS_gyro[3];
extern fp32 INS_quat[4];

int16_t goal_ecd_drive;//推动电机复位目标值
int32_t get_position;//当前推弹电机的ecd值
float up_speed;
int16_t goal_ecd_thrust;//推弹单机复位目标值

fp32 angle_turn[8];
fp32 angle_thrust[2];
fp32 angle_goal[2];//0表示前哨站的角度,1表示基地的角度
uint16_t ecd_trigger[2];//0表示前哨站位置,1表示基地位置

/*    函数及声明    */
static void dart_init();
static void dart_mode_set();
static void dart_relax_handle();
static void dart_back_handle();
static void dart_control_handle();
static void dart_goal_set_handle();
static void dart_ready_handle();
static void dart_launch_handle();
static void dart_trigger_handle();
static void dart_ready1();
static void dart_ready2();
static void dart_angle_update();
static void yaw_control();
static void turn_control();

void dart_task(void const*pvParameters)
{
    vTaskDelay(DART_TASK_INIT_TIME);

    dart_init();

    while(1)
    {
        dart_angle_update();
        dart_mode_set();

        switch(gimbal_dart.mode)
        {
            case DART_RELAX:
            {
                dart_relax_handle();
                break;
            }

            case DART_BACK:
            {
                dart_back_handle();
                break;
            }

            case DART_CONTROL:
            {
                dart_control_handle();
                break;
            }

            case DART_GOAL_SET:
            {
                dart_goal_set_handle();
                break;
            }

            case DART_READY:
            {
                dart_ready_handle();
                break;
            }

            case DART_LAUNCH:
            {
                dart_launch_handle();
                break;
            }

            case DART_TRIGGER:
            {
                dart_trigger_handle();
                break;
            }

            default:{
                break;
            }

        }

        vTaskDelay(2);
    }
}

static void dart_trigger_handle()
{

}

static void dart_launch_handle()
{
    num_launched+=1;
    gimbal_dart.mode=DART_READY;
}

static void dart_ready1()
{
    //扳机打开
    if(dart_goal==1)
    {

    }
    flags.ready_ok=1;
}

static void dart_ready2()
{
    flags.ready_ok=1;
}

static void dart_ready_handle()
{
    osDelay(500);
    if(num_launched==0)
    {
        dart_ready1();
    }
    if(num_launched>=1)
    {
        dart_ready2();
    }


}

static void dart_goal_set_handle()
{
    if(dart_goal==1)
    {
        gimbal_dart.yaw.relative_angle_set=angle_goal[0];
        thrust_motor.trigger_motor.speed= pid_calc(&thrust_motor.trigger_motor.angle_p,thrust_motor.trigger_motor.motor_measure->ecd,ecd_trigger[0]);
        thrust_motor.trigger_motor.give_current= pid_calc(&thrust_motor.trigger_motor.speed_p,thrust_motor.trigger_motor.motor_measure->speed_rpm,thrust_motor.trigger_motor.speed);
    }else
    {
        gimbal_dart.yaw.relative_angle_set=angle_goal[1];
        thrust_motor.trigger_motor.speed= pid_calc(&thrust_motor.trigger_motor.angle_p,thrust_motor.trigger_motor.motor_measure->ecd,ecd_trigger[1]);
        thrust_motor.trigger_motor.give_current= pid_calc(&thrust_motor.trigger_motor.speed_p,thrust_motor.trigger_motor.motor_measure->speed_rpm,thrust_motor.trigger_motor.speed);
    }
    gimbal_dart.yaw.relative_angle_get= motor_ecd_to_angle_change(gimbal_dart.yaw.motor_measure->ecd,gimbal_dart.yaw.motor_measure->offset_ecd);
    gimbal_dart.yaw.gyro_set= pid_loop_calc(&gimbal_dart.yaw.angle_p,gimbal_dart.yaw.relative_angle_get,gimbal_dart.yaw.relative_angle_set,180,-180);
    gimbal_dart.yaw.give_current= pid_calc(&gimbal_dart.yaw.speed_p,gimbal_dart.yaw.motor_measure->speed_rpm,gimbal_dart.yaw.gyro_set);
}

static void dart_control_handle()
{
    yaw_control();
    turn_control();
}

static void turn_control()
{
    launcher1.turn_motor.relative_angle_set-=rc_ctrl.rc.ch[3]*0.01*0.03f;
    launcher1.turn_motor.relative_angle_get= motor_ecd_to_angle_change(launcher1.turn_motor.motor_measure->ecd,launcher1.turn_motor.motor_measure->offset_ecd);
    launcher1.turn_motor.gyro_set= pid_loop_calc(&launcher1.turn_motor.angle_p,launcher1.turn_motor.relative_angle_get,
                                                 launcher1.turn_motor.relative_angle_set,
                                                 180,-180);
    launcher1.turn_motor.give_current= pid_calc(&launcher1.turn_motor.speed_p,
                                                launcher1.turn_motor.motor_measure->speed_rpm,
                                                launcher1.turn_motor.gyro_set);
}

static void yaw_control()
{
    gimbal_dart.yaw.relative_angle_set-=rc_ctrl.rc.ch[2]*0.01*0.03f;
    gimbal_dart.yaw.gyro_set= pid_loop_calc(&gimbal_dart.yaw.angle_p,gimbal_dart.yaw.relative_angle_get,
                                    gimbal_dart.yaw.relative_angle_set,
                                    180,-180);
    gimbal_dart.yaw.give_current= pid_calc(&gimbal_dart.yaw.speed_p,
                                   gimbal_dart.yaw.motor_measure->speed_rpm,
                                   gimbal_dart.yaw.gyro_set);
}

static void dart_back_handle()
{
    get_position=launcher1.R.motor_measure->total_ecd;
    up_speed=pid_calc(&launcher1.R.angle_p,
                      get_position,
                      goal_ecd_drive);
    launcher1.R.give_current=(int16_t) pid_calc(&launcher1.R.speed_p,
                                                launcher1.R.rpm_get,
                                                up_speed);
    if(abs(get_position-goal_ecd_drive)<2)
    {
        flags.back_drive_ok=1;
    }

    if(flags.back_drive_ok==1)
    {
        launcher1.turn_motor.relative_angle_get= motor_ecd_to_angle_change(launcher1.turn_motor.motor_measure->ecd,launcher1.turn_motor.motor_measure->offset_ecd);
        launcher1.turn_motor.relative_angle_set=angle_turn[flags.turn_angle];
        launcher1.turn_motor.gyro_set= pid_loop_calc(&launcher1.turn_motor.angle_p,launcher1.turn_motor.relative_angle_get,
                                              launcher1.turn_motor.relative_angle_set,180,-180);
        launcher1.turn_motor.give_current= pid_calc_my(&launcher1.turn_motor.speed_p,launcher1.turn_motor.motor_measure->speed_rpm,launcher1.turn_motor.gyro_set);
        if(fabs(launcher1.turn_motor.relative_angle_get-launcher1.turn_motor.relative_angle_set)<0.1)
        {
            flags.turn_angle_ok=1;
        }
    }

    if(flags.turn_angle_ok==1)
    {
        thrust_motor.thrust_angle_motor.angle_p.get= motor_ecd_to_angle_change(thrust_motor.thrust_angle_motor.motor_measure->ecd,thrust_motor.thrust_angle_motor.motor_measure->offset_ecd);
        thrust_motor.thrust_angle_motor.angle_p.set=angle_thrust[flags.thrust_angle];
        thrust_motor.thrust_angle_motor.speed= pid_loop_calc(&thrust_motor.thrust_angle_motor.angle_p,thrust_motor.thrust_angle_motor.angle_p.get,thrust_motor.thrust_angle_motor.angle_p.set,180,-180);
        thrust_motor.thrust_angle_motor.give_current= pid_calc(&thrust_motor.thrust_angle_motor.speed_p,thrust_motor.thrust_angle_motor.motor_measure->speed_rpm,thrust_motor.thrust_angle_motor.speed);
        if(fabs(thrust_motor.thrust_angle_motor.angle_p.get-thrust_motor.thrust_angle_motor.angle_p.get)<0.1)
        {
            flags.thrust_angle_ok=1;
        }
    }

    if(flags.thrust_angle_ok==1)
    {
        thrust_motor.thrust_move_motor.speed= pid_calc(&thrust_motor.thrust_move_motor.angle_p,thrust_motor.thrust_move_motor.motor_measure->total_ecd,goal_ecd_thrust);
        thrust_motor.thrust_move_motor.give_current= pid_calc(&thrust_motor.thrust_move_motor.speed_p,thrust_motor.thrust_move_motor.motor_measure->speed_rpm,thrust_motor.thrust_move_motor.speed);
        if(abs(thrust_motor.thrust_move_motor.motor_measure->total_ecd-goal_ecd_thrust)<5)
        {
            flags.thrust_move_ok=1;
        }
    }

    if(flags.thrust_move_ok==1)
    {
        thrust_motor.trigger_motor.speed= pid_calc(&thrust_motor.trigger_motor.angle_p,thrust_motor.trigger_motor.motor_measure->ecd,ecd_trigger[0]);
        thrust_motor.trigger_motor.give_current= pid_calc(&thrust_motor.trigger_motor.speed_p,thrust_motor.trigger_motor.motor_measure->speed_rpm,thrust_motor.trigger_motor.speed);
        if(abs(thrust_motor.trigger_motor.motor_measure->ecd-ecd_trigger[0])<1)
        {
            flags.trigger_move_ok=1;
        }
    }

    if(flags.trigger_move_ok==1)
    {
        flags.back_ok=1;
        flags.back_drive_ok=0;
        flags.turn_angle_ok=0;
        flags.thrust_angle_ok=0;
        flags.thrust_move_ok=0;
    }
}

static void dart_relax_handle()
{
    launcher1.R.give_current=0;
    launcher1.L.give_current=0;
    launcher1.turn_motor.give_current=0;
    thrust_motor.thrust_move_motor.give_current=0;
    thrust_motor.thrust_angle_motor.give_current=0;
    thrust_motor.trigger_motor.give_current=0;
}

static void dart_init()
{
    launcher1.R.motor_measure=&motor_3508_dart1[0];
    launcher1.L.motor_measure=&motor_3508_dart1[1];
    launcher1.turn_motor.motor_measure=&motor_6020_dart1[1];
    gimbal_dart.yaw.motor_measure=&motor_6020_dart1[0];
    thrust_motor.thrust_angle_motor.motor_measure=&motor_2006_dart1[0];
    thrust_motor.thrust_move_motor.motor_measure=&motor_2006_dart1[1];
    thrust_motor.trigger_motor.motor_measure=&motor_2006_dart1[2];

    //发射模式初始化
    launcher1.mode=FIRE_OFF;
    launcher1.last_mode=FIRE_OFF;
    //模式初始化
    gimbal_dart.mode=DART_RELAX;
    gimbal_dart.last_mode=DART_RELAX;

    pid_init(&launcher1.R.angle_p,
             DRIVE_ANGLE_MAX_OUT,
             DRIVE_ANGLE_MAX_IOUT,
             DRIVE_ANGLE_PID_KP,
             DRIVE_ANGLE_PID_KI,
             DRIVE_ANGLE_PID_KD);

    pid_init(&launcher1.R.speed_p,
             DRIVE_SPEED_MAX_OUT,
             DRIVE_SPEED_MAX_IOUT,
             DRIVE_SPEED_PID_KP,
             DRIVE_SPEED_PID_KI,
             DRIVE_SPEED_PID_KD);

    pid_init(&launcher1.L.angle_p,
             DRIVE_ANGLE_MAX_OUT,
             DRIVE_ANGLE_MAX_IOUT,
             DRIVE_ANGLE_PID_KP,
             DRIVE_ANGLE_PID_KI,
             DRIVE_ANGLE_PID_KD);

    pid_init(&launcher1.L.speed_p,
             DRIVE_SPEED_MAX_OUT,
             DRIVE_SPEED_MAX_IOUT,
             DRIVE_SPEED_PID_KP,
             DRIVE_SPEED_PID_KI,
             DRIVE_SPEED_PID_KD);

    pid_init(&launcher1.turn_motor.angle_p,
             TURN_ANGLE_MAX_OUT,
             TURN_ANGLE_MAX_IOUT,
             TURN_ANGLE_PID_KP,
             TURN_ANGLE_PID_KI,
             TURN_ANGLE_PID_KD);

    pid_init(&launcher1.turn_motor.speed_p,
             TURN_SPEED_MAX_OUT,
             TURN_SPEED_MAX_IOUT,
             TURN_SPEED_PID_KP,
             TURN_SPEED_PID_KI,
             TURN_SPEED_PID_KD);

    pid_init(&gimbal_dart.yaw.angle_p,
             YAW_ANGLE_MAX_OUT,
             YAW_ANGLE_MAX_IOUT,
             YAW_ANGLE_PID_KP,
             YAW_ANGLE_PID_KI,
             YAW_ANGLE_PID_KD);

    pid_init(&gimbal_dart.yaw.speed_p,
             YAW_SPEED_MAX_OUT,
             YAW_SPEED_MAX_IOUT,
             YAW_SPEED_PID_KP,
             YAW_SPEED_PID_KI,
             YAW_SPEED_PID_KD);

    pid_init(&thrust_motor.trigger_motor.angle_p,
             TRIGGER_MOVE_ANGLE_MAX_OUT,
             TRIGGER_MOVE_ANGLE_MAX_IOUT,
             TRIGGER_MOVE_ANGLE_PID_KP,
             TRIGGER_MOVE_ANGLE_PID_KI,
             TRIGGER_MOVE_ANGLE_PID_KD);

    pid_init(&thrust_motor.trigger_motor.speed_p,
             TRIGGER_MOVE_SPEED_MAX_OUT,
             TRIGGER_MOVE_SPEED_MAX_IOUT,
             TRIGGER_MOVE_SPEED_PID_KP,
             TRIGGER_MOVE_SPEED_PID_KI,
             TRIGGER_MOVE_SPEED_PID_KD);

    pid_init(&thrust_motor.thrust_angle_motor.angle_p,
             THRUST_ANGLE_ANGLE_MAX_OUT,
             THRUST_ANGLE_ANGLE_MAX_IOUT,
             THRUST_ANGLE_ANGLE_PID_KP,
             THRUST_ANGLE_ANGLE_PID_KI,
             THRUST_ANGLE_ANGLE_PID_KD);

    pid_init(&thrust_motor.thrust_angle_motor.speed_p,
             THRUST_ANGLE_SPEED_MAX_OUT,
             THRUST_ANGLE_SPEED_MAX_IOUT,
             THRUST_ANGLE_SPEED_PID_KP,
             THRUST_ANGLE_SPEED_PID_KI,
             THRUST_ANGLE_SPEED_PID_KD);

    pid_init(&thrust_motor.thrust_move_motor.angle_p,
             THRUST_MOVE_ANGLE_MAX_OUT,
             THRUST_MOVE_ANGLE_MAX_IOUT,
             THRUST_MOVE_ANGLE_PID_KP,
             THRUST_MOVE_ANGLE_PID_KI,
             THRUST_MOVE_ANGLE_PID_KD);

    pid_init(&thrust_motor.thrust_move_motor.speed_p,
             THRUST_MOVE_SPEED_MAX_OUT,
             THRUST_MOVE_SPEED_MAX_IOUT,
             THRUST_MOVE_SPEED_PID_KP,
             THRUST_MOVE_SPEED_PID_KI,
             THRUST_MOVE_SPEED_PID_KD);

}

static void dart_mode_set()
{
    if(switch_is_down(rc_ctrl.rc.s[RC_s_L])&&switch_is_down(rc_ctrl.rc.s[RC_s_R]))
    {
        gimbal_dart.last_mode=gimbal_dart.mode;
        gimbal_dart.mode=DART_RELAX;
    }
    if(gimbal_dart.mode==DART_RELAX||gimbal_dart.mode==DART_CONTROL)
    {
        if(switch_is_mid(rc_ctrl.rc.s[RC_s_L])&&switch_is_mid(rc_ctrl.rc.s[RC_s_R]))
        {
            gimbal_dart.last_mode=gimbal_dart.mode;
            gimbal_dart.mode=DART_BACK;
        }
    }
    if(switch_is_up(rc_ctrl.rc.s[RC_s_L])&&switch_is_up(rc_ctrl.rc.s[RC_s_R])&&(gimbal_dart.mode==DART_BACK)&&rc_ctrl.rc.ch[4]==0)
    {
        gimbal_dart.last_mode=gimbal_dart.mode;
        gimbal_dart.mode=DART_CONTROL;
    }
    if(gimbal_dart.mode==DART_BACK&&rc_ctrl.rc.ch[4]<-500)
    {
        if(switch_is_up(rc_ctrl.rc.s[RC_s_L]))launcherable_num=2;
        if(switch_is_down(rc_ctrl.rc.s[RC_s_L]))launcherable_num=1;
        if(switch_is_up(rc_ctrl.rc.s[RC_s_R]))dart_goal=1;
        if(switch_is_down(rc_ctrl.rc.s[RC_s_R]))dart_goal=2;
        if(launcherable_num>0&&dart_goal>0)
        {
            gimbal_dart.last_mode=gimbal_dart.mode;
            gimbal_dart.mode=DART_GOAL_SET;
        }
    }
    if(gimbal_dart.mode==DART_GOAL_SET)
    {
        if(rc_ctrl.rc.ch[2]>0&&rc_ctrl.rc.ch[3]>0&&rc_ctrl.rc.ch[0]<0&&rc_ctrl.rc.ch[1]>0)
        {
            gimbal_dart.last_mode=gimbal_dart.mode;
            gimbal_dart.mode=DART_READY;
        }
    }
    if(gimbal_dart.mode==DART_READY)
    {
        if(num_launched==0&&rc_ctrl.rc.ch[4]<-500&&flags.ready_ok==1)
        {
                gimbal_dart.last_mode=gimbal_dart.mode;
                gimbal_dart.mode=DART_LAUNCH;
                flags.ready_ok=0;
        }else
        if(num_launched>0&&!switch_is_mid(rc_ctrl.rc.s[RC_s_R])&&flags.ready_ok==1)
        {
            gimbal_dart.last_mode=gimbal_dart.mode;
            gimbal_dart.mode=DART_TRIGGER;
            flags.ready_ok=0;
        }
    }
    if(gimbal_dart.mode==DART_TRIGGER)
    {
        if(rc_ctrl.rc.ch[4]<-500)
        {
            gimbal_dart.last_mode=gimbal_dart.mode;
            gimbal_dart.mode=DART_LAUNCH;
        }
    }
}

static void dart_angle_update()
{
    gimbal_dart.yaw.absolute_angle_get=INS_angle[0]*MOTOR_RAD_TO_ANGLE;
    gimbal_dart.yaw.relative_angle_get-= motor_ecd_to_angle_change(gimbal_dart.yaw.motor_measure->ecd,gimbal_dart.yaw.motor_measure->offset_ecd);
}