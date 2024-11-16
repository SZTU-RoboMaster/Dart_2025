#include <math.h>
#include "dart.h"
#include "cmsis_os.h"
#include "can_receive.h"
#include "Atti.h"
#include "protocol_shaob.h"
#include "stdlib.h"
#include "user_lib.h"

uint8_t direction=0;
uint8_t num_launched=0;//飞镖已发射数目
uint8_t dart_goal;//飞镖目标,1为前哨站,2为基地

uint8_t launcherable_num;//飞镖可发射数目1为两发,2为四发
struct Launch_t launcher_dart;
struct Gimbal_t gimbal_dart;
struct Thrust_t thrust_motor;
struct All_Flag flags;

extern RC_ctrl_t rc_ctrl;
extern Eulr_t Eulr;
extern fp32 INS_angle[3];
extern fp32 INS_gyro[3];
extern fp32 INS_quat[4];

int32_t init_ecd_trigger;//初始时候扳机的total_ecd
int32_t init_ecd_drive;//初始时候推动电机的total_ecd
int32_t init_ecd_thrust;//初始化推弹电机的total_ecd

int16_t goal_ecd_loading_drive;//上膛位置的ecd
int16_t goal_ecd_loading_thrust;//上弹时到达位置的ecd
int16_t trigger_ecd_set[4];
fp32 turn_motor_angle_set[6];//换弹电机角度数组
int8_t turn_angle=0;//换弹电机角度数组索引
fp32 thrust_motor_angle_set[2];//推弹电机角度数组
fp32 angle_goal[2];//0表示前哨站的角度,1表示基地的角度
int32_t ecd_trigger[2];//0表示前哨站位置,1表示基地位置
bool trigger_move_down_l; //确定左拨杆放下面的标志位
bool trigger_move_mid_l;  //确定左拨杆放中间的标志位
uint8_t trigger_move;      //确定扳机移动的格数
uint8_t begin_count1=1;

int64_t time_now;//测试用
uint8_t flag1=false;//测试用
float speed_move=-1000.0f;//测试用
fp32 dart_yaw=0.0f;
fp32 speed_drive=550.0f;

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
static void drive_control();
static void trigger_control();
static void thrust_motor_angle_control();
static void thrust_motor_move_control();
static void dart_reset();

/*      滤波      */
first_order_filter_type_t filter_yaw_in;

void dart_task(void const*pvParameters)
{
    vTaskDelay(DART_TASK_INIT_TIME);

    dart_init();
    //dart_reset();
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
        //todo can发送函数
            CAN_cmd_motor(CAN_2,
                      CAN_MOTOR_0x1FF_ID,
                      gimbal_dart.motor_yaw.give_current,
                      thrust_motor.trigger_motor.give_current,
                      launcher_dart.push_motor_r.give_current,
                      launcher_dart.push_motor_l.give_current);
        CAN_cmd_motor(CAN_1,
                      CAN_MOTOR_0x200_ID,
                      thrust_motor.thrust_move_motor.give_current,
                      thrust_motor.thrust_angle_motor.give_current,
                      0,
                      0);
        vTaskDelay(2);
    }
}

static void dart_reset()
{
    uint32_t reset_time;
    while(1)
    {
        if(begin_count1==1)
        {
            reset_time=HAL_GetTick();
            begin_count1=0;
        }
        if(HAL_GetTick() - reset_time > 2000)
        {
            break;
        }
        float up_speed_out=speed_move;
        thrust_motor.thrust_move_motor.give_current=(int16_t) pid_calc(&thrust_motor.thrust_move_motor.speed_p,
                                                                       thrust_motor.thrust_move_motor.motor_measure->speed_rpm,
                                                                       up_speed_out);
          //thrust_motor.thrust_move_motor.give_current=-750;

        launcher_dart.push_motor_r.give_current=1000;
        launcher_dart.push_motor_l.give_current=-launcher_dart.push_motor_r.give_current;
        CAN_cmd_motor(CAN_2,
                      CAN_MOTOR_0x1FF_ID,
                      0,
                      0,
                      launcher_dart.push_motor_r.give_current,
                      launcher_dart.push_motor_l.give_current);

        CAN_cmd_motor(CAN_1,
                      CAN_MOTOR_0x200_ID,
                      thrust_motor.thrust_move_motor.give_current,
                      0,
                      0,
                      0);
    }
    init_ecd_trigger=thrust_motor.trigger_motor.motor_measure->total_ecd;
    init_ecd_thrust=thrust_motor.thrust_move_motor.motor_measure->total_ecd;
    init_ecd_drive=launcher_dart.push_motor_r.motor_measure->total_ecd;
    launcher_dart.push_motor_r.angle_p.set=init_ecd_drive;
}

//基本完成差个移动一格移动多少ecd
static void dart_trigger_handle()
{
    if(switch_is_up(rc_ctrl.rc.s[RC_s_R]))
    {
        direction=1;
    }
    if(switch_is_down(rc_ctrl.rc.s[RC_s_R]))
    {
        direction=-1;
    }
    if(trigger_move_mid_l==1 && trigger_move_down_l==1)
    {
        //trigger_move+=1;//检测的时候用的
        trigger_move_down_l=0;
        trigger_move_mid_l=0;
    }
    if(switch_is_mid(rc_ctrl.rc.s[RC_s_L]))
    {
        trigger_move_mid_l=1;
    }
    if(switch_is_down(rc_ctrl.rc.s[RC_s_L]))
    {
        trigger_move_down_l=1;
    }

}

//差个扳机打开
static void dart_launch_handle()
{
    if(launcherable_num==1 && num_launched<2)
    {
        num_launched+=1;
    }
    if(launcherable_num==2 && num_launched<4)
    {
        num_launched+=1;
    }
    gimbal_dart.mode=DART_READY;

}

static void dart_ready1()
{
    //扳机打开
    if(dart_goal==GOAL_FRONT_STATION)
    {
        thrust_motor.trigger_motor.speed_p.set= pid_calc(&thrust_motor.trigger_motor.angle_p,
                                                         thrust_motor.trigger_motor.motor_measure->total_ecd,
                                                         ecd_trigger[front]+init_ecd_trigger);
        thrust_motor.trigger_motor.give_current= pid_calc(&thrust_motor.trigger_motor.speed_p,
                                                          thrust_motor.trigger_motor.motor_measure->speed_rpm,
                                                          thrust_motor.trigger_motor.speed_p.set);
        if(abs(thrust_motor.trigger_motor.motor_measure->total_ecd-ecd_trigger[front]-init_ecd_trigger)<5)
        {
            flags.is_ready1_trigger_move_ok=true;
        }
    }else
    {
        thrust_motor.trigger_motor.speed_p.set= pid_calc(&thrust_motor.trigger_motor.angle_p,
                                                         thrust_motor.trigger_motor.motor_measure->total_ecd,
                                                         ecd_trigger[base]+init_ecd_trigger);
        thrust_motor.trigger_motor.give_current= pid_calc(&thrust_motor.trigger_motor.speed_p,
                                                          thrust_motor.trigger_motor.motor_measure->speed_rpm,
                                                          thrust_motor.trigger_motor.speed_p.set);
        if(abs(thrust_motor.trigger_motor.motor_measure->total_ecd-ecd_trigger[base]-init_ecd_trigger)<5)
        {
            flags.is_ready1_trigger_move_ok=true;
        }
    }
    if(flags.is_ready1_trigger_move_ok==true)
    {
        launcher_dart.push_motor_r.speed_p.set= pid_calc(&launcher_dart.push_motor_r.angle_p,
                                                         launcher_dart.push_motor_r.motor_measure->total_ecd,
                                                         init_ecd_drive);
        launcher_dart.push_motor_r.give_current= pid_calc(&launcher_dart.push_motor_r.speed_p,
                                                          launcher_dart.push_motor_r.motor_measure->speed_rpm,
                                                          launcher_dart.push_motor_r.speed_p.set);
        if(abs(launcher_dart.push_motor_r.motor_measure->total_ecd-init_ecd_drive)<5)
        {
            flags.is_ready1_drive_ok=true;
        }
    }
    if(flags.is_ready1_drive_ok==true)
    {
        //扳机闭合
    }
    if(flags.is_ready1_trigger_on_ok==true)
    {
        flags.is_ready_ok=true;
        flags.is_ready1_trigger_move_ok=false;
        flags.is_ready1_drive_ok=false;
        flags.is_ready1_trigger_on_ok=false;
    }
}

static void dart_ready2()
{
    //扳机打开
    //推动滑台下放到上膛位置
    launcher_dart.push_motor_r.speed_p.set= pid_calc(&launcher_dart.push_motor_r.angle_p,
                                                     launcher_dart.push_motor_r.motor_measure->total_ecd,
                                                     goal_ecd_loading_drive+init_ecd_drive);
    launcher_dart.push_motor_r.give_current= pid_calc(&launcher_dart.push_motor_r.speed_p,
                                                      launcher_dart.push_motor_r.motor_measure->speed_rpm,
                                                      launcher_dart.push_motor_r.speed_p.set);
    //换弹电机旋转到上弹位置
    if(abs(launcher_dart.push_motor_r.motor_measure->total_ecd-goal_ecd_loading_drive-init_ecd_drive)<5)
    {
        turn_angle+=1;
        turn_angle%=8;
        launcher_dart.turn_motor.angle_p.set=turn_motor_angle_set[turn_angle];
        launcher_dart.turn_motor.angle_p.get= motor_ecd_to_angle_change(launcher_dart.turn_motor.motor_measure->ecd,launcher_dart.turn_motor.motor_measure->offset_ecd);
        launcher_dart.turn_motor.speed_p.set= pid_loop_calc(&launcher_dart.turn_motor.angle_p,
                                                            launcher_dart.turn_motor.angle_p.get,
                                                            launcher_dart.turn_motor.angle_p.set,
                                                            180,-180);
        launcher_dart.turn_motor.give_current= pid_calc(&launcher_dart.turn_motor.speed_p,
                                                        launcher_dart.turn_motor.motor_measure->speed_rpm,
                                                        launcher_dart.turn_motor.speed_p.set);
    }
    //推弹滑台推弹进行上弹
    if(fabs(launcher_dart.turn_motor.angle_p.get-turn_motor_angle_set[turn_angle])<0.1)
    {
        thrust_motor.thrust_angle_motor.angle_p.set=thrust_motor_angle_set[work_mode];
        thrust_motor.thrust_angle_motor.angle_p.get= motor_ecd_to_angle_change(thrust_motor.thrust_angle_motor.motor_measure->ecd,thrust_motor.thrust_angle_motor.motor_measure->offset_ecd);
        thrust_motor.thrust_angle_motor.speed_p.set= pid_loop_calc(&thrust_motor.thrust_angle_motor.angle_p,
                                                                   thrust_motor.thrust_angle_motor.angle_p.get,
                                                                   thrust_motor.thrust_angle_motor.angle_p.set,
                                                                   180,-180);
        thrust_motor.thrust_angle_motor.give_current= pid_calc(&thrust_motor.thrust_angle_motor.speed_p,
                                                               thrust_motor.thrust_angle_motor.motor_measure->speed_rpm,
                                                               thrust_motor.thrust_angle_motor.speed_p.set);
    }
    if(fabs(thrust_motor.thrust_angle_motor.angle_p.get-thrust_motor_angle_set[work_mode])<0.1)
    {
        thrust_motor.thrust_move_motor.speed_p.set= pid_calc(&thrust_motor.thrust_move_motor.angle_p,
                                                             thrust_motor.thrust_move_motor.motor_measure->total_ecd,
                                                             goal_ecd_loading_thrust+init_ecd_thrust);
        thrust_motor.thrust_move_motor.give_current= pid_calc(&thrust_motor.thrust_move_motor.speed_p,
                                                              thrust_motor.thrust_move_motor.motor_measure->speed_rpm,
                                                              thrust_motor.thrust_move_motor.speed_p.set);
    }
    //推弹滑台复位
    if(abs(thrust_motor.thrust_move_motor.motor_measure->total_ecd-goal_ecd_loading_thrust-init_ecd_thrust)<1)
    {
        thrust_motor.thrust_angle_motor.angle_p.set=thrust_motor_angle_set[free_mode];
        thrust_motor.thrust_angle_motor.angle_p.get= motor_ecd_to_angle_change(thrust_motor.thrust_angle_motor.motor_measure->ecd,thrust_motor.thrust_angle_motor.motor_measure->offset_ecd);
        thrust_motor.thrust_angle_motor.speed_p.set= pid_loop_calc(&thrust_motor.thrust_angle_motor.angle_p,
                                                                   thrust_motor.thrust_angle_motor.angle_p.get,
                                                                   thrust_motor.thrust_angle_motor.angle_p.set,
                                                                   180,-180);
        thrust_motor.thrust_angle_motor.give_current= pid_calc(&thrust_motor.thrust_angle_motor.speed_p,
                                                               thrust_motor.thrust_angle_motor.motor_measure->speed_rpm,
                                                               thrust_motor.thrust_angle_motor.speed_p.set);

        thrust_motor.thrust_move_motor.speed_p.set= pid_calc(&thrust_motor.thrust_move_motor.angle_p,
                                                             thrust_motor.thrust_move_motor.motor_measure->total_ecd,
                                                             init_ecd_thrust);
        thrust_motor.thrust_move_motor.give_current = pid_calc(&thrust_motor.thrust_move_motor.speed_p,
                                                              thrust_motor.thrust_move_motor.motor_measure->speed_rpm,
                                                              thrust_motor.thrust_move_motor.speed_p.set);
    }
    //推动滑台下放至扳机位置同时换弹电机旋转至下一个角度值
    if(fabs(thrust_motor.thrust_angle_motor.angle_p.get-thrust_motor_angle_set[free_mode])<0.1&&abs(abs(thrust_motor.thrust_move_motor.motor_measure->total_ecd-init_ecd_thrust)<1))
    {
        launcher_dart.push_motor_r.angle_p.set=trigger_ecd_set[launcherable_num]+init_ecd_drive;
        launcher_dart.push_motor_r.speed_p.set= pid_calc(&launcher_dart.push_motor_r.angle_p,
                                                         launcher_dart.push_motor_r.motor_measure->total_ecd,
                                                         launcher_dart.push_motor_r.angle_p.set);
        launcher_dart.push_motor_r.give_current= pid_calc(&launcher_dart.push_motor_r.speed_p,
                                                          launcher_dart.push_motor_r.motor_measure->speed_rpm,
                                                          launcher_dart.push_motor_r.speed_p.set);
        turn_angle+=1;
        turn_angle%=8;
        launcher_dart.turn_motor.angle_p.get= motor_ecd_to_angle_change(launcher_dart.turn_motor.motor_measure->ecd,launcher_dart.turn_motor.motor_measure->offset_ecd);
        launcher_dart.turn_motor.speed_p.set= pid_loop_calc(&launcher_dart.turn_motor.angle_p,
                                                       launcher_dart.turn_motor.angle_p.get,
                                                       turn_motor_angle_set[turn_angle],
                                                       180,-180);
        launcher_dart.turn_motor.give_current= pid_calc(&launcher_dart.turn_motor.speed_p,
                                                        launcher_dart.turn_motor.motor_measure->speed_rpm,
                                                        launcher_dart.turn_motor.speed_p.set);
    }
    //扳机闭合同时推动滑台复位
    if(abs(launcher_dart.push_motor_r.motor_measure->total_ecd-trigger_ecd_set[launcherable_num]-init_ecd_trigger)<1&&fabs(launcher_dart.turn_motor.angle_p.get-turn_motor_angle_set[turn_angle])<0.1)
    {
        //扳机闭合
        launcher_dart.push_motor_r.speed_p.set= pid_calc(&launcher_dart.push_motor_r.angle_p,
                                                         launcher_dart.push_motor_r.motor_measure->total_ecd,
                                                         init_ecd_drive);
        launcher_dart.push_motor_r.give_current= pid_calc(&launcher_dart.push_motor_r.speed_p,
                                                          launcher_dart.push_motor_r.motor_measure->speed_rpm,
                                                          launcher_dart.push_motor_r.motor_measure->speed_rpm);
    }
    if(abs(launcher_dart.push_motor_r.motor_measure->total_ecd-init_ecd_drive)<1)
    {
        flags.is_ready_ok=true;
    }
}

static void dart_ready_handle()
{
    osDelay(2);
    if(num_launched==0)
    {
        dart_ready1();
    }
    if(num_launched>=1)
    {
        dart_ready2();
    }


}
//差个接收视觉发送的数据
static void dart_goal_set_handle()
{
    if(dart_goal==GOAL_FRONT_STATION)
    {
        gimbal_dart.motor_yaw.relative_angle_set=angle_goal[front];
        thrust_motor.trigger_motor.speed= pid_calc(&thrust_motor.trigger_motor.angle_p,thrust_motor.trigger_motor.motor_measure->total_ecd,ecd_trigger[0]+init_ecd_trigger);
        thrust_motor.trigger_motor.give_current= pid_calc(&thrust_motor.trigger_motor.speed_p,thrust_motor.trigger_motor.motor_measure->speed_rpm,thrust_motor.trigger_motor.speed);
    }else
    {
        gimbal_dart.motor_yaw.relative_angle_set=angle_goal[base];
        thrust_motor.trigger_motor.speed= pid_calc(&thrust_motor.trigger_motor.angle_p,thrust_motor.trigger_motor.motor_measure->total_ecd,ecd_trigger[1]+init_ecd_trigger);
        thrust_motor.trigger_motor.give_current= pid_calc(&thrust_motor.trigger_motor.speed_p,thrust_motor.trigger_motor.motor_measure->speed_rpm,thrust_motor.trigger_motor.speed);
    }
    gimbal_dart.motor_yaw.relative_angle_get= motor_ecd_to_angle_change(gimbal_dart.motor_yaw.motor_measure->ecd,gimbal_dart.motor_yaw.motor_measure->offset_ecd);
    gimbal_dart.motor_yaw.gyro_set= pid_loop_calc(&gimbal_dart.motor_yaw.angle_p,gimbal_dart.motor_yaw.relative_angle_get,gimbal_dart.motor_yaw.relative_angle_set,180,-180);
    gimbal_dart.motor_yaw.give_current= pid_calc(&gimbal_dart.motor_yaw.speed_p,gimbal_dart.motor_yaw.motor_measure->speed_rpm,gimbal_dart.motor_yaw.gyro_set);
}

//已完成
static void dart_control_handle()
{
    //dart_reset();
    //yaw_control();
    //turn_control();
    drive_control();
    //trigger_control();
    //thrust_motor_angle_control();
    //thrust_motor_move_control();
}

static void thrust_motor_move_control()
{
    //thrust_motor.thrust_move_motor.angle_p.set-=rc_ctrl.rc.ch[2]*0.01*0.03f;
    thrust_motor.thrust_move_motor.speed_p.set= pid_calc(&thrust_motor.thrust_move_motor.angle_p,
                                                         thrust_motor.thrust_move_motor.motor_measure->total_ecd,
                                                         300000+init_ecd_thrust);
    thrust_motor.thrust_move_motor.give_current= (int16_t)pid_calc(&thrust_motor.thrust_move_motor.speed_p,
                                                          thrust_motor.thrust_move_motor.motor_measure->speed_rpm,
                                                          thrust_motor.thrust_move_motor.speed_p.set);
    thrust_motor.thrust_move_motor.give_current=(int16_t)thrust_motor.thrust_move_motor.give_current;

}

static void thrust_motor_angle_control()
{
    thrust_motor.thrust_angle_motor.angle_p.set-=rc_ctrl.rc.ch[1]*0.01*0.03f;
    thrust_motor.thrust_angle_motor.angle_p.get= motor_ecd_to_angle_change(thrust_motor.thrust_angle_motor.motor_measure->ecd,thrust_motor.thrust_angle_motor.motor_measure->offset_ecd);
    thrust_motor.thrust_angle_motor.speed_p.set= pid_loop_calc(&thrust_motor.thrust_angle_motor.angle_p,
                                                               thrust_motor.thrust_angle_motor.angle_p.get,
                                                               thrust_motor.thrust_angle_motor.angle_p.set,
                                                               180,-180);
    thrust_motor.thrust_angle_motor.give_current= pid_calc(&thrust_motor.thrust_angle_motor.speed_p,
                                                           thrust_motor.thrust_angle_motor.motor_measure->speed_rpm,
                                                           thrust_motor.thrust_angle_motor.speed_p.set);
}

static void trigger_control()
{
//    thrust_motor.trigger_motor.angle_p.set-=rc_ctrl.rc.ch[0]*0.01*0.03f;
//    thrust_motor.trigger_motor.speed_p.set= pid_calc(&thrust_motor.trigger_motor.angle_p,
//                                                          thrust_motor.trigger_motor.motor_measure->total_ecd,
//                                                          thrust_motor.trigger_motor.angle_p.set);
//    thrust_motor.trigger_motor.give_current= pid_calc(&thrust_motor.trigger_motor.speed_p,
//                                                      thrust_motor.trigger_motor.motor_measure->speed_rpm,
//                                                      thrust_motor.trigger_motor.speed_p.set);
    thrust_motor.trigger_motor.give_current=rc_ctrl.rc.ch[3]*16;
    abs_limit(&thrust_motor.trigger_motor.give_current,9000);
}

static void drive_control()
{
//    launcher_dart.push_motor_r.angle_p.set=rc_ctrl.rc.ch[1]*0.5;//瞎填的数值
//    if(launcher_dart.push_motor_r.angle_p.set>init_ecd_drive)
//    {
//        launcher_dart.push_motor_r.angle_p.set=init_ecd_drive;
//    }
//    launcher_dart.push_motor_r.speed_p.set= pid_calc(&launcher_dart.push_motor_r.angle_p,
//                                                          launcher_dart.push_motor_r.motor_measure->total_ecd,
//                                                     launcher_dart.push_motor_r.angle_p.set);
    //launcher_dart.push_motor_r.speed_p.set=rc_ctrl.rc.ch[1]*3;
    launcher_dart.push_motor_r.give_current= pid_calc(&launcher_dart.push_motor_r.speed_p,
                                                      launcher_dart.push_motor_r.motor_measure->speed_rpm,
                                                      launcher_dart.push_motor_r.speed_p.set);
    launcher_dart.push_motor_l.give_current=-launcher_dart.push_motor_r.give_current;
//    launcher_dart.push_motor_r.give_current=rc_ctrl.rc.ch[1]*8;
//    launcher_dart.push_motor_l.give_current=-launcher_dart.push_motor_r.give_current;
}

static void turn_control()
{
    //launcher_dart.turn_motor.relative_angle_set-=rc_ctrl.rc.ch[2]*0.01*0.03f;
    //launcher_dart.turn_motor.relative_angle_get= -motor_ecd_to_angle_change(launcher_dart.turn_motor.motor_measure->ecd,launcher_dart.turn_motor.motor_measure->offset_ecd);
    launcher_dart.turn_motor.relative_angle_set=turn_motor_angle_set[turn_angle];
    //launcher_dart.turn_motor.relative_angle_set=69.7850f;
    if(launcher_dart.turn_motor.relative_angle_set>180)
    {
        launcher_dart.turn_motor.relative_angle_set=launcher_dart.turn_motor.relative_angle_set-360;
    }
    else if(launcher_dart.turn_motor.relative_angle_set<-180)
    {
        launcher_dart.turn_motor.relative_angle_set=launcher_dart.turn_motor.relative_angle_set+360;
    }
    launcher_dart.turn_motor.gyro_set= pid_loop1_calc(&launcher_dart.turn_motor.angle_p,
                                                     launcher_dart.turn_motor.relative_angle_get,
                                                     launcher_dart.turn_motor.relative_angle_set,
                                                     180,-180);
    launcher_dart.turn_motor.give_current= -pid_calc(&launcher_dart.turn_motor.speed_p,
                                                launcher_dart.turn_motor.motor_measure->speed_rpm,
                                                launcher_dart.turn_motor.gyro_set);
    if((fabs(launcher_dart.turn_motor.relative_angle_get-launcher_dart.turn_motor.relative_angle_set)<1)&&((turn_angle==0&&flag1==false)||HAL_GetTick()-time_now>1000))
    {
        //vTaskDelay(5000);
        turn_angle+=1;
        turn_angle%=6;
        time_now=HAL_GetTick();
        flag1=true;
    }
}

static void yaw_control()
{
    gimbal_dart.motor_yaw.relative_angle_set-=rc_ctrl.rc.ch[2]*0.01*0.03f;
    if(gimbal_dart.motor_yaw.relative_angle_set>=50.0)
    {
        gimbal_dart.motor_yaw.relative_angle_set=50;
    }
    else if(gimbal_dart.motor_yaw.relative_angle_set<=-51)
    {
        gimbal_dart.motor_yaw.relative_angle_set=-51;
    }
    //gimbal_dart.motor_yaw.relative_angle_set=0.96f;
    gimbal_dart.motor_yaw.gyro_set= pid_loop_calc(&gimbal_dart.motor_yaw.angle_p,gimbal_dart.motor_yaw.relative_angle_get,
                                    gimbal_dart.motor_yaw.relative_angle_set,
                                    180,-180);

    first_order_filter_cali(&filter_yaw_in,gimbal_dart.motor_yaw.gyro_set);

    gimbal_dart.motor_yaw.give_current= -pid_calc(&gimbal_dart.motor_yaw.speed_p,
                                                  gimbal_dart.motor_yaw.motor_measure->speed_rpm,
                                                  filter_yaw_in.out);

}

//已完成
static void dart_back_handle()
{
    gimbal_dart.motor_yaw.give_current=0;
    launcher_dart.push_motor_r.give_current=0;
    launcher_dart.push_motor_l.give_current=0;
//    gimbal_dart.motor_yaw.angle_p.set=YAW_BACK_ANGLE;
//    gimbal_dart.motor_yaw.speed_p.set= pid_loop_calc(&gimbal_dart.motor_yaw.angle_p,
//                                                     gimbal_dart.motor_yaw.relative_angle_get,
//                                                     gimbal_dart.motor_yaw.angle_p.set,
//                                                     180,-180);
//    gimbal_dart.motor_yaw.give_current= -pid_calc(&gimbal_dart.motor_yaw.speed_p,
//                                                 gimbal_dart.motor_yaw.motor_measure->speed_rpm,
//                                                 gimbal_dart.motor_yaw.speed_p.set);
//
//    launcher_dart.push_motor_r.speed_p.set=pid_calc(&launcher_dart.push_motor_r.angle_p,
//                      launcher_dart.push_motor_r.motor_measure->total_ecd,
//                      init_ecd_drive);
//    launcher_dart.push_motor_r.give_current=(int16_t) pid_calc(&launcher_dart.push_motor_r.speed_p,
//                                                launcher_dart.push_motor_r.rpm_get,
//                                                launcher_dart.push_motor_r.speed_p.set);
//
//    if(abs(launcher_dart.push_motor_r.motor_measure->total_ecd-init_ecd_drive)<2)
//    {
//        flags.is_back_drive_ok=true;
//    }
//
//    if(flags.is_back_drive_ok==true)
//    {
//        launcher_dart.turn_motor.relative_angle_get= motor_ecd_to_angle_change(launcher_dart.turn_motor.motor_measure->ecd,launcher_dart.turn_motor.motor_measure->offset_ecd);
//        launcher_dart.turn_motor.relative_angle_set=turn_motor_angle_set[turn_angle];
//        launcher_dart.turn_motor.gyro_set= pid_loop_calc(&launcher_dart.turn_motor.angle_p,launcher_dart.turn_motor.relative_angle_get,
//                                              launcher_dart.turn_motor.relative_angle_set,180,-180);
//        launcher_dart.turn_motor.give_current= pid_calc_my(&launcher_dart.turn_motor.speed_p,launcher_dart.turn_motor.motor_measure->speed_rpm,launcher_dart.turn_motor.gyro_set);
//        if(fabs(launcher_dart.turn_motor.relative_angle_get-launcher_dart.turn_motor.relative_angle_set)<0.1)
//        {
//            flags.is_turn_angle_ok=true;
//        }
//    }
//
//    if(flags.is_turn_angle_ok==true)
//    {
//        thrust_motor.thrust_angle_motor.angle_p.get= motor_ecd_to_angle_change(thrust_motor.thrust_angle_motor.motor_measure->ecd,thrust_motor.thrust_angle_motor.motor_measure->offset_ecd);
//        thrust_motor.thrust_angle_motor.angle_p.set=thrust_motor_angle_set[free_mode];
//        thrust_motor.thrust_angle_motor.speed= pid_loop_calc(&thrust_motor.thrust_angle_motor.angle_p,thrust_motor.thrust_angle_motor.angle_p.get,thrust_motor.thrust_angle_motor.angle_p.set,180,-180);
//        thrust_motor.thrust_angle_motor.give_current= pid_calc(&thrust_motor.thrust_angle_motor.speed_p,thrust_motor.thrust_angle_motor.motor_measure->speed_rpm,thrust_motor.thrust_angle_motor.speed);
//        if(fabs(thrust_motor.thrust_angle_motor.angle_p.get-thrust_motor.thrust_angle_motor.angle_p.get)<0.1)
//        {
//            flags.is_thrust_angle_ok=true;
//        }
//    }
//
//    if(flags.is_thrust_angle_ok==true)
//    {
//        thrust_motor.thrust_move_motor.speed= pid_calc(&thrust_motor.thrust_move_motor.angle_p,thrust_motor.thrust_move_motor.motor_measure->total_ecd,init_ecd_thrust);
//        thrust_motor.thrust_move_motor.give_current= pid_calc(&thrust_motor.thrust_move_motor.speed_p,thrust_motor.thrust_move_motor.motor_measure->speed_rpm,thrust_motor.thrust_move_motor.speed);
//        if(abs(thrust_motor.thrust_move_motor.motor_measure->total_ecd-init_ecd_thrust)<5)
//        {
//            flags.is_thrust_move_ok=true;
//        }
//    }
//
//    if(flags.is_thrust_move_ok==true)
//    {
//        thrust_motor.trigger_motor.speed= pid_calc(&thrust_motor.trigger_motor.angle_p,thrust_motor.trigger_motor.motor_measure->total_ecd,init_ecd_trigger);
//        thrust_motor.trigger_motor.give_current= pid_calc(&thrust_motor.trigger_motor.speed_p,thrust_motor.trigger_motor.motor_measure->speed_rpm,thrust_motor.trigger_motor.speed);
//        if(abs(thrust_motor.trigger_motor.motor_measure->ecd-init_ecd_trigger)<1)
//        {
//            flags.is_trigger_move_ok=1;
//        }
//    }
//
//    if(flags.is_trigger_move_ok==true)
//    {
//        flags.is_back_ok=true;
//        flags.is_back_drive_ok=false;
//        flags.is_turn_angle_ok=false;
//        flags.is_thrust_angle_ok=false;
//        flags.is_thrust_move_ok=false;
//    }
}

//已经完成
static void dart_relax_handle()
{
    launcher_dart.push_motor_r.give_current=0;
    launcher_dart.push_motor_l.give_current=0;
    launcher_dart.turn_motor.give_current=0;
    thrust_motor.thrust_move_motor.give_current=0;
    thrust_motor.thrust_angle_motor.give_current=0;
    thrust_motor.trigger_motor.give_current=0;
    gimbal_dart.motor_yaw.give_current=0;
}
//已完成
static void dart_init()
{
    launcher_dart.push_motor_r.motor_measure=&motor_3508[0];
    launcher_dart.push_motor_l.motor_measure=&motor_3508[1];
    launcher_dart.turn_motor.motor_measure=&motor_6020[1];
    gimbal_dart.motor_yaw.motor_measure=&motor_6020[0];
    thrust_motor.thrust_angle_motor.motor_measure=&motor_2006[0];
    thrust_motor.thrust_move_motor.motor_measure=&motor_2006[1];
    thrust_motor.trigger_motor.motor_measure=&motor_2006[2];

    //发射模式初始化
    launcher_dart.mode=FIRE_OFF;
    launcher_dart.last_mode=FIRE_OFF;
    //模式初始化
    gimbal_dart.mode=DART_RELAX;
    gimbal_dart.last_mode=DART_RELAX;
    //pid初始化
    pid_init(&launcher_dart.push_motor_r.angle_p,
             DRIVE_ANGLE_MAX_OUT,
             DRIVE_ANGLE_MAX_IOUT,
             DRIVE_ANGLE_PID_KP,
             DRIVE_ANGLE_PID_KI,
             DRIVE_ANGLE_PID_KD);

    pid_init(&launcher_dart.push_motor_r.speed_p,
             DRIVE_SPEED_MAX_OUT,
             DRIVE_SPEED_MAX_IOUT,
             DRIVE_SPEED_PID_KP,
             DRIVE_SPEED_PID_KI,
             DRIVE_SPEED_PID_KD);

    pid_init(&launcher_dart.push_motor_l.angle_p,
             DRIVE_ANGLE_MAX_OUT,
             DRIVE_ANGLE_MAX_IOUT,
             DRIVE_ANGLE_PID_KP,
             DRIVE_ANGLE_PID_KI,
             DRIVE_ANGLE_PID_KD);

    pid_init(&launcher_dart.push_motor_l.speed_p,
             DRIVE_SPEED_MAX_OUT,
             DRIVE_SPEED_MAX_IOUT,
             DRIVE_SPEED_PID_KP,
             DRIVE_SPEED_PID_KI,
             DRIVE_SPEED_PID_KD);

    pid_init(&launcher_dart.turn_motor.angle_p,
             TURN_ANGLE_MAX_OUT,
             TURN_ANGLE_MAX_IOUT,
             TURN_ANGLE_PID_KP,
             TURN_ANGLE_PID_KI,
             TURN_ANGLE_PID_KD);

    pid_init(&launcher_dart.turn_motor.speed_p,
             TURN_SPEED_MAX_OUT,
             TURN_SPEED_MAX_IOUT,
             TURN_SPEED_PID_KP,
             TURN_SPEED_PID_KI,
             TURN_SPEED_PID_KD);

    pid_init(&gimbal_dart.motor_yaw.angle_p,
             YAW_ANGLE_MAX_OUT,
             YAW_ANGLE_MAX_IOUT,
             YAW_ANGLE_PID_KP,
             YAW_ANGLE_PID_KI,
             YAW_ANGLE_PID_KD);

    pid_init(&gimbal_dart.motor_yaw.speed_p,
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

    first_order_filter_init(&filter_yaw_in,5,30);

    gimbal_dart.motor_yaw.motor_measure->offset_ecd=1357;
    launcher_dart.turn_motor.motor_measure->offset_ecd=5459;

    turn_motor_angle_set[0]=60.2490f;
    turn_motor_angle_set[1]=114.3017f;
    turn_motor_angle_set[2]=168.6044f;
    turn_motor_angle_set[3]=-120.0146f;
    turn_motor_angle_set[4]=-60.60058f;
    turn_motor_angle_set[5]=-1.5380f;

}
//已完成
static void dart_mode_set()
{

    if(switch_is_down(rc_ctrl.rc.s[RC_s_L]) && switch_is_down(rc_ctrl.rc.s[RC_s_R]))
    {
        gimbal_dart.last_mode=gimbal_dart.mode;
        gimbal_dart.mode=DART_RELAX;
    }
    if(gimbal_dart.mode==DART_RELAX || gimbal_dart.mode==DART_CONTROL)
    {
        if(switch_is_mid(rc_ctrl.rc.s[RC_s_L]) && switch_is_mid(rc_ctrl.rc.s[RC_s_R]))
        {
            gimbal_dart.last_mode=gimbal_dart.mode;
            gimbal_dart.mode=DART_BACK;
        }
    }
    if(switch_is_up(rc_ctrl.rc.s[RC_s_L]) && switch_is_up(rc_ctrl.rc.s[RC_s_R]) && (gimbal_dart.mode==DART_BACK) && rc_ctrl.rc.ch[4]==0)
    {
        gimbal_dart.last_mode=gimbal_dart.mode;
        gimbal_dart.mode=DART_CONTROL;
    }
    if(gimbal_dart.mode==DART_BACK && rc_ctrl.rc.ch[4]<-500)
    {

        if(switch_is_up(rc_ctrl.rc.s[RC_s_L]))
        {
            launcherable_num=2;
        }
        if(switch_is_down(rc_ctrl.rc.s[RC_s_L]))
        {
            launcherable_num=1;
        }
        if(switch_is_up(rc_ctrl.rc.s[RC_s_R]))
        {
            dart_goal=GOAL_FRONT_STATION;
        }
        if(switch_is_down(rc_ctrl.rc.s[RC_s_R]))
        {
            dart_goal=GOAL_BASE_STATION;
        }
        if(launcherable_num>0 && dart_goal>0)
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
        if(num_launched==0&&rc_ctrl.rc.ch[4]<-500&&flags.is_ready_ok==1)
        {
                gimbal_dart.last_mode=gimbal_dart.mode;
                gimbal_dart.mode=DART_LAUNCH;
                flags.is_ready_ok=0;
        }else
        if(num_launched>0&&!switch_is_mid(rc_ctrl.rc.s[RC_s_R])&&flags.is_ready_ok==1)
        {
            gimbal_dart.last_mode=gimbal_dart.mode;
            gimbal_dart.mode=DART_TRIGGER;
            flags.is_ready_ok=0;
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
    //gimbal_dart.motor_yaw.absolute_angle_get=INS_angle[0]*MOTOR_RAD_TO_ANGLE;
    gimbal_dart.motor_yaw.relative_angle_get= -motor_ecd_to_angle_change(gimbal_dart.motor_yaw.motor_measure->ecd,gimbal_dart.motor_yaw.motor_measure->offset_ecd);
    launcher_dart.turn_motor.relative_angle_get= -motor_ecd_to_angle_change(launcher_dart.turn_motor.motor_measure->ecd,launcher_dart.turn_motor.motor_measure->offset_ecd);
}