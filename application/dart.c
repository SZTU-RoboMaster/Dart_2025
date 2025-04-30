#include <math.h>
#include "dart.h"
#include "cmsis_os.h"
#include "can_receive.h"
#include "Atti.h"
#include "protocol_shaob.h"
#include "stdlib.h"
#include "user_lib.h"
#include "tim.h"
#include "Auto.h"
#include "dm_8009.h"

first_order_filter_type_t turn_first_order_set;

int8_t direction=0;
uint8_t num_launched=0;//飞镖已发射数目
uint8_t dart_goal=0;//飞镖目标,1为前哨站,2为基地
uint8_t ceshi_launched=0;


uint8_t launcherable_num;//飞镖可发射数目1为两发,2为四发
struct Launch_t launcher_dart;
struct Gimbal_t gimbal_dart;
struct Thrust_t thrust_motor;
struct All_Flag flags;

struct Dm4310 turn_motor;

fp32 angle_test;
uint32_t time_test1;
fp32 time_test2;
fp32 angle_now;

fp32 time_now;

extern RC_ctrl_t rc_ctrl;
extern Eulr_t Eulr;
extern fp32 INS_angle[3];
extern fp32 INS_gyro[3];
extern fp32 INS_quat[4];

extern robot_ctrl_info_t robot_ctrl;

int32_t init_ecd_trigger;//初始时候扳机的total_ecd
int32_t init_ecd_drive_left;//初始时候推动左电机的total_ecd
int32_t init_ecd_drive_right;//初始时候推动右电机的total_ecd
int32_t init_ecd_thrust_move;//初始化推弹移动电机的total_ecd
int32_t init_ecd_thrust_angle;//初始化推弹角度电机的total_ecd
bool ready1_flag;
fp32 ready1_set_drive_right_distance;
fp32 ready1_set_drive_left_distance;

uint8_t ceshi_flag=0;

bool ready2_back_flag;
bool ready1_back_flag;
bool ready2_load_flag;//确定上膛位置标志位
bool ready2_goal_flag;//确定发射位置标志位
bool ready2_set_flag;
bool ready1_load_flag;
bool ready2_load_flag2;
bool ready_goal_flag;

uint32_t ready2_load_time;
uint32_t ready1_back_time;//解决滑台回中飞镖会掉落的参数
uint32_t ready2_back_time;
uint32_t ready1_drive_time;
fp32 ready2_load_set_drive_right_distance;
fp32 ready2_load_set_drive_left_distance;
fp32 ready2_goal_set_drive_right_distance;
fp32 ready2_goal_set_drive_left_distance;
int16_t cnt;
uint32_t ready1_load_time;//准备1函数里滑台到上弹位置时的位置

fp32 trigger_to_outposts_distance_set[4]={157.2032f,157.1997f,157.2023f,157.2049f};//到前哨站的距离
fp32 trigger_to_base_distance_set[4]={156.2032f,156.1997f,156.2023f,156.2049f};//到基地的距离
fp32 turn_motor_angle_set[6];//换弹电机角度数组
static fp32 set_dm_motor_angle = 0;
int32_t turn_angle=0;//换弹电机角度数组索引
fp32 yaw_angle_goal[2]={14.948f,-20.997f};//0表示前哨站的角度,1表示基地的角度
bool trigger_move_down_l; //确定左拨杆放下面的标志位
bool trigger_move_mid_l;  //确定左拨杆放中间的标志位
uint8_t begin_count1=1;

uint8_t flag1=false;//测试用
int16_t init_speed_thrust_move=-1500;//测试用//-1000
int16_t init_speed_drive=1000;
int16_t init_speed_trigger=-4000;
int16_t init_speed_thrust_angle=-600;//600
fp32 dart_length=715;
fp32 slide_length=80;

//初始距离和当前距离
fp32 get_trigger_distance;
fp32 get_angle_thrust_angle;
fp32 init_trigger_distance;
fp32 init_drive_left_distance;
fp32 init_drive_right_distance;
fp32 init_angle_thrust_angle;
fp32 get_drive_left_distance;
fp32 get_drive_right_distance;
fp32 set_trigger_distance;
fp32 set_drive_left_distance;
fp32 set_drive_right_distance;
fp32 set_angle_thrust_angle;
fp32 init_thrust_move_distance;
fp32 get_thrust_move_distance;
fp32 set_thrust_move_distance;
fp32 init_drive_left_distance;

fp32 back_trigger_distance;
fp32 back_drive_left_distance;
fp32 back_drive_right_distance;
fp32 back_thrust_move_distance;
fp32 back_thrust_angle_angle;
fp32 back_thrust_move_distance1;
fp32 back_yaw;

fp32 ready1_back_drive_left_distance;
fp32 ready1_back_drive_right_distance;
fp32 ready1_set_drive_left_distance1;

fp32 ready2_thrust_move_goal_distance;
fp32 ready2_thrust_angle_goal_angle;

fp32 ready2_find_drive_left_distance;

fp32 ready2_back_drive_left_distance;
bool launch_flag=0;
uint32_t launch_time;

uint32_t relax_time;
uint32_t back_finish_time;



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
static void dart_data_update();
static void yaw_control();
static void turn_control();
static void drive_control();
static void trigger_control();
static void thrust_motor_angle_control();
static void thrust_motor_move_control();
static void dart_reset();
static fp32 trigger_distance_conversion(int32_t ecd);
static fp32 drive_distance_conversion(int32_t ecd);
static fp32 ecd_to_angle(int32_t ecd);
static fp32 thrust_move_distance_conversion(int32_t ecd);
static void steer_motor_control();
static void set_load_drive_distance();
static void trigger_open();
static void trigger_off();
static void turn_motor_init();

/*      滤波      */
first_order_filter_type_t filter_yaw_in;


void dart_task(void const*pvParameters)
{
    vTaskDelay(DART_TASK_INIT_TIME);

    dart_init();
    dart_reset();
    turn_motor_init();
    set_dm8009_pos_speed(CAN_1,
                         TURN_MOTOR_ID,
                         turn_motor_angle_set[turn_angle],
                         1.6f);
    while(1)
    {
        dart_data_update();
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
                //dart_back_handle();
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
                      0,
                      thrust_motor.trigger_motor.give_current,
                      launcher_dart.push_motor_r.give_current,
                      launcher_dart.push_motor_l.give_current);
        CAN_cmd_motor(CAN_1,
                      CAN_MOTOR_0x1FF_ID,
                      gimbal_dart.motor_yaw.give_current,
                      launcher_dart.turn_motor.give_current,
                      thrust_motor.thrust_angle_motor.give_current,
                      thrust_motor.thrust_move_motor.give_current);
        vTaskDelay(2);
    }
}

static void turn_motor_init()
{

    dm8009_init(&turn_motor,TURN_MOTOR_ID);
    osDelay(10);
    set_dm8009_enable(CAN_1,TURN_MOTOR_ID);
}

static void dart_reset()
{
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);


    uint32_t reset_time;
    while(1)
    {
        if(begin_count1==1)
        {
            reset_time=HAL_GetTick();
            begin_count1=0;
        }
        if(HAL_GetTick() - reset_time > 3000)
        {
            thrust_motor.thrust_move_motor.give_current=0;
            launcher_dart.push_motor_r.give_current=0;
            launcher_dart.push_motor_l.give_current=0;
            thrust_motor.thrust_angle_motor.give_current=0;
            gimbal_dart.mode=DART_RELAX;
            break;
        }
        thrust_motor.thrust_angle_motor.give_current=init_speed_thrust_angle;
        thrust_motor.thrust_move_motor.give_current=init_speed_thrust_move;
        launcher_dart.push_motor_r.give_current=init_speed_drive;
        thrust_motor.trigger_motor.give_current=init_speed_trigger;
        launcher_dart.push_motor_l.give_current=-launcher_dart.push_motor_r.give_current;
        CAN_cmd_motor(CAN_2,
                      CAN_MOTOR_0x1FF_ID,
                      0,
                      thrust_motor.trigger_motor.give_current,
                      launcher_dart.push_motor_r.give_current,
                      launcher_dart.push_motor_l.give_current);

        CAN_cmd_motor(CAN_1,
                      CAN_MOTOR_0x1FF_ID,
                      0,
                      0,
                      thrust_motor.thrust_angle_motor.give_current,
                      thrust_motor.thrust_move_motor.give_current);
    }
    init_ecd_trigger = thrust_motor.trigger_motor.motor_measure->total_ecd;
    init_ecd_thrust_move = thrust_motor.thrust_move_motor.motor_measure->total_ecd;
    init_ecd_thrust_angle=thrust_motor.thrust_angle_motor.motor_measure->total_ecd;
    init_ecd_drive_right = launcher_dart.push_motor_r.motor_measure->total_ecd;
    init_ecd_drive_left=launcher_dart.push_motor_l.motor_measure->total_ecd;

    init_trigger_distance = trigger_distance_conversion(init_ecd_trigger);
    init_drive_right_distance = drive_distance_conversion(init_ecd_drive_right);
    init_drive_left_distance= drive_distance_conversion(init_ecd_drive_left);
    init_angle_thrust_angle= ecd_to_angle(init_ecd_thrust_angle);
    init_thrust_move_distance= thrust_move_distance_conversion(init_ecd_thrust_move);

    set_trigger_distance=init_trigger_distance;
    set_drive_left_distance=init_drive_left_distance;
    set_drive_right_distance=init_drive_right_distance;
    set_angle_thrust_angle=init_angle_thrust_angle;
    set_thrust_move_distance=init_thrust_move_distance;

    back_yaw=-6.712f;
    back_thrust_angle_angle=init_angle_thrust_angle+7;
    back_thrust_move_distance=init_thrust_move_distance+15;
    back_drive_right_distance=init_drive_right_distance-10;
    back_drive_left_distance=init_drive_left_distance+10;
    back_trigger_distance=init_trigger_distance+100;
    back_thrust_move_distance1=init_thrust_move_distance+30;
    ready1_back_drive_right_distance=init_drive_right_distance-7;
    ready1_back_drive_left_distance=init_drive_left_distance+10;//18
    ready2_back_drive_left_distance=init_drive_left_distance+10;


    ready2_thrust_angle_goal_angle=61.566f+init_angle_thrust_angle;

    ready2_thrust_move_goal_distance=96.5266f+init_thrust_move_distance;//112.5266
    trigger_to_outposts_distance_set[0]=139.29254f+init_trigger_distance;//低了
    trigger_to_outposts_distance_set[1]=136.500f+init_trigger_distance;//低了一点点
    trigger_to_outposts_distance_set[2]=138.28944f+init_trigger_distance;//
    trigger_to_outposts_distance_set[3]=138.28944f+init_trigger_distance;//
    trigger_to_base_distance_set[0]=10.0f+init_trigger_distance;//白天 28.5//晚上 白天数据下移2(所有) 22
    trigger_to_base_distance_set[1]=10.09f+init_trigger_distance;//白天 31.49//晚上 25.09
    trigger_to_base_distance_set[2]=10.59f+init_trigger_distance;//白天31.09//晚上 25.09
    trigger_to_base_distance_set[3]=10.59f+init_trigger_distance;//白天31.09//晚上 25.09
}

static fp32 ecd_to_angle(int32_t ecd)
{
    return (fp32)ecd/8192*12;
}

//基本完成差个移动一格移动多少ecd
static void dart_trigger_handle()
{
    gimbal_dart.motor_yaw.angle_p.set = gimbal_dart.motor_yaw.angle_p.get - filter_yaw_in.out/ 80;
    if(gimbal_dart.motor_yaw.angle_p.set>=25.0)
    {
        gimbal_dart.motor_yaw.angle_p.set=25;
    }
    else if(gimbal_dart.motor_yaw.angle_p.set<=-25)
    {
        gimbal_dart.motor_yaw.angle_p.set=-25;
    }
    gimbal_dart.motor_yaw.speed_p.set = pid_calc(&gimbal_dart.motor_yaw.angle_p,
                                                 gimbal_dart.motor_yaw.angle_p.get,
                                                 gimbal_dart.motor_yaw.angle_p.set);
    gimbal_dart.motor_yaw.give_current = pid_calc(&gimbal_dart.motor_yaw.speed_p,
                                                  gimbal_dart.motor_yaw.motor_measure->speed_rpm,
                                                  gimbal_dart.motor_yaw.speed_p.set);
    if(fabs(filter_yaw_in.out)<=5)
    {
        gimbal_dart.motor_yaw.give_current=0;
    }
    launcher_dart.push_motor_l.speed_p.set = pid_calc(&launcher_dart.push_motor_l.angle_p,
                                                      get_drive_left_distance,
                                                      ready2_back_drive_left_distance);
    launcher_dart.push_motor_l.give_current = pid_calc(&launcher_dart.push_motor_l.speed_p,
                                                       launcher_dart.push_motor_l.motor_measure->speed_rpm,
                                                       launcher_dart.push_motor_l.speed_p.set);
    launcher_dart.push_motor_r.give_current=-launcher_dart.push_motor_l.give_current;

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
        if(direction==1)
        {
            thrust_motor.trigger_motor.angle_p.set=get_trigger_distance+0.5f;

        }
        if(direction==-1)
        {
            thrust_motor.trigger_motor.angle_p.set=get_trigger_distance-1;
        }
        trigger_move_down_l=0;
        trigger_move_mid_l=0;
        direction=0;
    }
    if(switch_is_mid(rc_ctrl.rc.s[RC_s_L]))
    {
        trigger_move_mid_l=1;
    }
    if(switch_is_down(rc_ctrl.rc.s[RC_s_L]))
    {
        trigger_move_down_l=1;
    }
    thrust_motor.trigger_motor.speed_p.set= pid_calc(&thrust_motor.trigger_motor.angle_p,
                                                     get_trigger_distance,
                                                     thrust_motor.trigger_motor.angle_p.set);
    thrust_motor.trigger_motor.give_current= pid_calc(&thrust_motor.trigger_motor.speed_p,
                                                      thrust_motor.trigger_motor.motor_measure->speed_rpm,
                                                      thrust_motor.trigger_motor.speed_p.set);
    if(fabs(get_trigger_distance-thrust_motor.trigger_motor.angle_p.set)<0.1)
    {
        thrust_motor.trigger_motor.give_current=0;
    }
}

//差个扳机打开
static void dart_launch_handle()
{
    gimbal_dart.motor_yaw.give_current=0;
    CAN_cmd_motor(CAN_1,
                  CAN_MOTOR_0x1FF_ID,
                  gimbal_dart.motor_yaw.give_current,
                  launcher_dart.turn_motor.give_current,
                  thrust_motor.thrust_angle_motor.give_current,
                  thrust_motor.thrust_move_motor.give_current);
    trigger_open();
    launch_time=HAL_GetTick();
    launch_flag=0;
    vTaskDelay(2000);
    if(num_launched<4)
    {
        ready_goal_flag=0;
        num_launched+=1;
        ready2_load_flag=0;
        ready2_goal_flag=0;
        ready2_back_flag=false;
        ready1_flag=0;
        ready1_load_flag=false;
    }
    if(num_launched<4)
    {
        if(num_launched==2)
        {
            dart_goal=0;
            gimbal_dart.mode=DART_BACK;
        }else if(num_launched!=2) {
            gimbal_dart.mode = DART_READY;
        }
    }else
    {
        gimbal_dart.mode=DART_RELAX;
        num_launched=0;
        dart_goal=0;
        turn_angle=0;
    }
}

static void set_ready_distance()
{
    ready1_set_drive_left_distance1=116+init_drive_left_distance;
}

static void set_drive_distance()
{
    if(dart_goal==GOAL_FRONT_STATION)
    {
        ready1_set_drive_right_distance = -(dart_length - slide_length - trigger_to_outposts_distance_set[num_launched] -
                                            get_drive_right_distance);
        ready1_set_drive_left_distance=(dart_length-slide_length-trigger_to_outposts_distance_set[num_launched]-get_drive_left_distance+7);
    }else {
        ready1_set_drive_right_distance = -(dart_length - slide_length - trigger_to_base_distance_set[num_launched] -
                                            get_drive_right_distance);
        ready1_set_drive_left_distance = (dart_length - slide_length - trigger_to_base_distance_set[num_launched]+7 -
                                          get_drive_left_distance);
    }
}

static void trigger_open()
{
    __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,500);
    __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,500);
    __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,500);
    __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_4,500);
}

static void trigger_off()
{
    __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,2000);
    __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,2000);
    __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,2000);
    __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_4,2000);
}

static void dart_ready1()
{
    if(ready_goal_flag==0) {
        if (dart_goal == GOAL_FRONT_STATION) {
            gimbal_dart.motor_yaw.speed_p.set = pid_calc(&gimbal_dart.motor_yaw.angle_p,
                                                         gimbal_dart.motor_yaw.angle_p.get,
                                                         yaw_angle_goal[front]);

            gimbal_dart.motor_yaw.give_current = pid_calc(&gimbal_dart.motor_yaw.speed_p,
                                                          gimbal_dart.motor_yaw.motor_measure->speed_rpm,
                                                          gimbal_dart.motor_yaw.speed_p.set);
            if(fabs(gimbal_dart.motor_yaw.angle_p.set-yaw_angle_goal[front])<=1)
            {
                ready_goal_flag=1;
            }
        } else {
            gimbal_dart.motor_yaw.speed_p.set = pid_calc(&gimbal_dart.motor_yaw.angle_p,
                                                         gimbal_dart.motor_yaw.angle_p.get,
                                                         yaw_angle_goal[base]);

            gimbal_dart.motor_yaw.give_current = pid_calc(&gimbal_dart.motor_yaw.speed_p,
                                                          gimbal_dart.motor_yaw.motor_measure->speed_rpm,
                                                          gimbal_dart.motor_yaw.speed_p.set);
            if(fabs(gimbal_dart.motor_yaw.angle_p.set-yaw_angle_goal[base])<=1)
            {
                ready_goal_flag=1;
            }
        }
    }
    if(ready_goal_flag==1)
    {
        gimbal_dart.motor_yaw.angle_p.set = gimbal_dart.motor_yaw.angle_p.get - filter_yaw_in.out/ 80;
        if(gimbal_dart.motor_yaw.angle_p.set>=25.0)
        {
            gimbal_dart.motor_yaw.angle_p.set=25;
        }
        else if(gimbal_dart.motor_yaw.angle_p.set<=-25)
        {
            gimbal_dart.motor_yaw.angle_p.set=-25;
        }
        gimbal_dart.motor_yaw.speed_p.set = pid_calc(&gimbal_dart.motor_yaw.angle_p,
                                                     gimbal_dart.motor_yaw.angle_p.get,
                                                     gimbal_dart.motor_yaw.angle_p.set);
        gimbal_dart.motor_yaw.give_current = pid_calc(&gimbal_dart.motor_yaw.speed_p,
                                                      gimbal_dart.motor_yaw.motor_measure->speed_rpm,
                                                      gimbal_dart.motor_yaw.speed_p.set);
        if(fabs(filter_yaw_in.out)<=5)
        {
            gimbal_dart.motor_yaw.give_current=0;
        }
    }
    if (ready1_flag == 0) {
        set_drive_distance();
        set_ready_distance();
        ready1_flag = 1;
    }
    if(flags.is_ready_ok==true)
    {
        thrust_motor.trigger_motor.give_current = 0;
        launcher_dart.push_motor_l.speed_p.set = pid_calc(&launcher_dart.push_motor_l.angle_p,
                                                          get_drive_left_distance,
                                                          ready1_back_drive_left_distance);
        launcher_dart.push_motor_l.give_current = pid_calc(&launcher_dart.push_motor_l.speed_p,
                                                           launcher_dart.push_motor_l.motor_measure->speed_rpm,
                                                           launcher_dart.push_motor_l.speed_p.set);
        launcher_dart.push_motor_r.give_current=-launcher_dart.push_motor_l.give_current;

    }else
    {
        if (dart_goal == GOAL_FRONT_STATION) {
            thrust_motor.trigger_motor.speed_p.set = pid_calc(&thrust_motor.trigger_motor.angle_p,
                                                              get_trigger_distance,
                                                              trigger_to_outposts_distance_set[num_launched]);
            thrust_motor.trigger_motor.give_current = pid_calc(&thrust_motor.trigger_motor.speed_p,
                                                               thrust_motor.trigger_motor.motor_measure->speed_rpm,
                                                               thrust_motor.trigger_motor.speed_p.set);
            if (fabs(get_trigger_distance - trigger_to_outposts_distance_set[num_launched]) < 0.1) {
                flags.is_ready1_trigger_move_ok = true;
                thrust_motor.trigger_motor.give_current = 0;
            }
        } else {
            thrust_motor.trigger_motor.speed_p.set = pid_calc(&thrust_motor.trigger_motor.angle_p,
                                                              get_trigger_distance,
                                                              trigger_to_base_distance_set[num_launched]);
            thrust_motor.trigger_motor.give_current = pid_calc(&thrust_motor.trigger_motor.speed_p,
                                                               thrust_motor.trigger_motor.motor_measure->speed_rpm,
                                                               thrust_motor.trigger_motor.speed_p.set);
            if (fabs(get_trigger_distance - trigger_to_base_distance_set[num_launched]) < 1) {
                flags.is_ready1_trigger_move_ok = true;
                thrust_motor.trigger_motor.give_current = 0;

            }
        }
        if (flags.is_ready1_trigger_move_ok == true) {
            if(flags.is_ready1_trigger_on_ok==false)
            {
                trigger_open();
                flags.is_ready1_trigger_on_ok=true;
            }
        }
        if(flags.is_ready1_trigger_on_ok==true)
        {
            launcher_dart.push_motor_l.speed_p.set = pid_calc(&launcher_dart.push_motor_l.angle_p,
                                                              get_drive_left_distance,
                                                              ready1_set_drive_left_distance1);
            launcher_dart.push_motor_l.give_current = pid_calc(&launcher_dart.push_motor_l.speed_p,
                                                               launcher_dart.push_motor_l.motor_measure->speed_rpm,
                                                               launcher_dart.push_motor_l.speed_p.set);
            launcher_dart.push_motor_r.give_current=-launcher_dart.push_motor_l.give_current;
            if(fabs(get_drive_left_distance-ready1_set_drive_left_distance1)<5)
            {
                flags.is_ready1_drive_ok1=true;
            }
            if(flags.is_ready1_drive_ok1==true)
            {
                if(flags.is_ready1_drive_slow_ok==false)
                {
                    flags.is_ready1_drive_slow_ok=true;
                    ready1_drive_time=HAL_GetTick();
                }
                if(HAL_GetTick()-ready1_drive_time<1000)
                {
                    launcher_dart.push_motor_l.speed_p.set=2000;
                    launcher_dart.push_motor_l.give_current = pid_calc(&launcher_dart.push_motor_l.speed_p,
                                                                       launcher_dart.push_motor_l.motor_measure->speed_rpm,
                                                                       launcher_dart.push_motor_l.speed_p.set);
                    launcher_dart.push_motor_r.give_current=-launcher_dart.push_motor_l.give_current;
                }else
                {
                    launcher_dart.push_motor_l.speed_p.set = pid_calc(&launcher_dart.push_motor_l.angle_p,
                                                                      get_drive_left_distance,
                                                                      ready1_set_drive_left_distance);
                    launcher_dart.push_motor_l.give_current = pid_calc(&launcher_dart.push_motor_l.speed_p,
                                                                       launcher_dart.push_motor_l.motor_measure->speed_rpm,
                                                                       launcher_dart.push_motor_l.speed_p.set);
                    launcher_dart.push_motor_r.give_current=-launcher_dart.push_motor_l.give_current;
                }
                if ( fabs(get_drive_left_distance - ready1_set_drive_left_distance) < 8) {
                    if(ready1_load_flag==false) {
                        trigger_off();
                        ready1_load_time = HAL_GetTick();
                        ready1_load_flag = true;
                    }
                    if(HAL_GetTick()-ready1_load_time>500)
                    {
                        flags.is_ready1_drive_ok = true;
                    }
                }
            }
        }
        if(flags.is_ready1_drive_ok==true)
        {
            if(flags.is_ready1_trigger_off_ok==false)
            {
                trigger_off();
                flags.is_ready1_trigger_off_ok=true;
            }
        }
        if (flags.is_ready1_trigger_off_ok == true) {
            if(ready1_back_flag==false)
            {
                ready1_back_flag=true;
                ready1_back_time=HAL_GetTick();
            }
            if(HAL_GetTick()-ready1_back_time<500)
            {
                launcher_dart.push_motor_l.speed_p.set=-500;
                launcher_dart.push_motor_l.give_current = pid_calc(&launcher_dart.push_motor_l.speed_p,
                                                                   launcher_dart.push_motor_l.motor_measure->speed_rpm,
                                                                   launcher_dart.push_motor_l.speed_p.set);
            }else
            if(HAL_GetTick()-ready1_back_time>500)
            {
                launcher_dart.push_motor_l.speed_p.set = pid_calc(&launcher_dart.push_motor_l.angle_p,
                                                                  get_drive_left_distance,
                                                                  ready1_back_drive_left_distance);
                launcher_dart.push_motor_l.give_current = pid_calc(&launcher_dart.push_motor_l.speed_p,
                                                                   launcher_dart.push_motor_l.motor_measure->speed_rpm,
                                                                   launcher_dart.push_motor_l.speed_p.set);
            }
            launcher_dart.push_motor_r.give_current=-launcher_dart.push_motor_l.give_current;
            if ( //fabs(get_drive_right_distance - ready1_back_drive_right_distance) < 5 &&
                fabs(get_drive_left_distance - ready1_back_drive_left_distance) < 10) {
                flags.is_ready1_drive_init_ok = true;
            }
        }
        if (flags.is_ready1_drive_init_ok == true) {
            flags.is_ready_ok = true;
            flags.is_ready1_drive_init_ok = false;
            flags.is_ready1_trigger_on_ok = false;
            flags.is_ready1_drive_ok = false;
            flags.is_ready1_trigger_move_ok = false;
            flags.is_ready1_trigger_off_ok=false;
            flags.is_ready1_trigger_on_ok=false;
            flags.is_ready1_drive_slow_ok=false;
            flags.is_ready1_drive_ok1=false;
            ready1_load_flag=false;
            ready1_back_flag=false;
        }
    }
}

static void set_load_drive_distance()
{

    ready2_load_set_drive_left_distance=457+init_drive_left_distance;
    ready2_load_set_drive_right_distance=-500+init_drive_right_distance;
}
static void set_find_drive_distance()
{
    ready2_find_drive_left_distance=435+init_drive_left_distance;
}

static void set_goal_drive_distance()
{
    if(dart_goal==GOAL_FRONT_STATION)
    {
        ready2_goal_set_drive_left_distance=(dart_length - slide_length - trigger_to_outposts_distance_set[num_launched]
                                             - back_drive_left_distance+5);
        ready2_goal_set_drive_right_distance = -(dart_length - slide_length -
                                                 trigger_to_outposts_distance_set[num_launched] -
                                                 back_drive_right_distance);
    }else {
        ready2_goal_set_drive_right_distance = -(dart_length - slide_length -
                                                 trigger_to_base_distance_set[num_launched] -
                                                 back_drive_right_distance);
        ready2_goal_set_drive_left_distance = (dart_length - slide_length - trigger_to_base_distance_set[num_launched]-2
                                               - back_drive_left_distance+5);
    }
}

static void dart_ready2()
{
    dart_data_update();
    if(ready_goal_flag==0) {
        if (dart_goal == GOAL_FRONT_STATION) {
            gimbal_dart.motor_yaw.speed_p.set = pid_calc(&gimbal_dart.motor_yaw.angle_p,
                                                         gimbal_dart.motor_yaw.angle_p.get,
                                                         yaw_angle_goal[front]);

            gimbal_dart.motor_yaw.give_current = pid_calc(&gimbal_dart.motor_yaw.speed_p,
                                                          gimbal_dart.motor_yaw.motor_measure->speed_rpm,
                                                          gimbal_dart.motor_yaw.speed_p.set);
            if(fabs(gimbal_dart.motor_yaw.angle_p.set-yaw_angle_goal[front])<=1)
            {
                ready_goal_flag=1;
            }
        } else {
            gimbal_dart.motor_yaw.speed_p.set = pid_calc(&gimbal_dart.motor_yaw.angle_p,
                                                         gimbal_dart.motor_yaw.angle_p.get,
                                                         yaw_angle_goal[base]);

            gimbal_dart.motor_yaw.give_current = pid_calc(&gimbal_dart.motor_yaw.speed_p,
                                                          gimbal_dart.motor_yaw.motor_measure->speed_rpm,
                                                          gimbal_dart.motor_yaw.speed_p.set);
            if(fabs(gimbal_dart.motor_yaw.angle_p.set-yaw_angle_goal[base])<=1)
            {
                ready_goal_flag=1;
            }
        }
    }
    if(ready_goal_flag==1)
    {
        gimbal_dart.motor_yaw.angle_p.set = gimbal_dart.motor_yaw.angle_p.get - filter_yaw_in.out/ 80;
        if(gimbal_dart.motor_yaw.angle_p.set>=25.0)
        {
            gimbal_dart.motor_yaw.angle_p.set=25;
        }
        else if(gimbal_dart.motor_yaw.angle_p.set<=-25)
        {
            gimbal_dart.motor_yaw.angle_p.set=-25;
        }
        gimbal_dart.motor_yaw.speed_p.set = pid_calc(&gimbal_dart.motor_yaw.angle_p,
                                                     gimbal_dart.motor_yaw.angle_p.get,
                                                     gimbal_dart.motor_yaw.angle_p.set);
        gimbal_dart.motor_yaw.give_current = pid_calc(&gimbal_dart.motor_yaw.speed_p,
                                                      gimbal_dart.motor_yaw.motor_measure->speed_rpm,
                                                      gimbal_dart.motor_yaw.speed_p.set);
        if(fabs(filter_yaw_in.out)<=5)
        {
            gimbal_dart.motor_yaw.give_current=0;
        }
    }
    if(ready2_load_flag==0)
    {
        set_load_drive_distance();
        ready2_load_flag=1;
    }
    if(flags.is_ready2_ok==true)
    {
        thrust_motor.trigger_motor.give_current = 0;

        launcher_dart.push_motor_l.speed_p.set = pid_calc(&launcher_dart.push_motor_l.angle_p,
                                                          get_drive_left_distance,
                                                          ready2_back_drive_left_distance);
        launcher_dart.push_motor_l.give_current = pid_calc(&launcher_dart.push_motor_l.speed_p,
                                                           launcher_dart.push_motor_l.motor_measure->speed_rpm,
                                                           launcher_dart.push_motor_l.speed_p.set);
        launcher_dart.push_motor_r.give_current=-launcher_dart.push_motor_l.give_current;

        thrust_motor.thrust_angle_motor.speed_p.set = pid_loop_calc(&thrust_motor.thrust_angle_motor.angle_p,
                                                                    get_angle_thrust_angle,
                                                                    back_thrust_angle_angle,
                                                                    180, -180);
        thrust_motor.thrust_angle_motor.give_current = pid_calc(&thrust_motor.thrust_angle_motor.speed_p,
                                                                thrust_motor.thrust_angle_motor.motor_measure->speed_rpm,
                                                                thrust_motor.thrust_angle_motor.speed_p.set);

        thrust_motor.thrust_move_motor.speed_p.set = pid_calc(&thrust_motor.thrust_move_motor.angle_p,
                                                              get_thrust_move_distance,
                                                              back_thrust_move_distance1);
        thrust_motor.thrust_move_motor.give_current = pid_calc(&thrust_motor.thrust_move_motor.speed_p,
                                                               thrust_motor.thrust_move_motor.motor_measure->speed_rpm,
                                                               thrust_motor.thrust_move_motor.speed_p.set);
    }else
    if(flags.is_ready2_ok==false) {
        if(flags.is_ready2_trigger_ok==false) {
            if(dart_goal==GOAL_FRONT_STATION) {
                thrust_motor.trigger_motor.speed = pid_calc(&thrust_motor.trigger_motor.angle_p, get_trigger_distance,
                                                            trigger_to_outposts_distance_set[num_launched]);
                thrust_motor.trigger_motor.give_current = pid_calc(&thrust_motor.trigger_motor.speed_p,
                                                                   thrust_motor.trigger_motor.motor_measure->speed_rpm,
                                                                   thrust_motor.trigger_motor.speed);
                if (fabs(get_trigger_distance - trigger_to_outposts_distance_set[num_launched]) < 0.1) {
                    flags.is_ready2_trigger_ok = true;
                    thrust_motor.trigger_motor.give_current = 0;
                }
            }else
            {
                thrust_motor.trigger_motor.speed = pid_calc(&thrust_motor.trigger_motor.angle_p, get_trigger_distance,
                                                            trigger_to_base_distance_set[num_launched]);
                thrust_motor.trigger_motor.give_current = pid_calc(&thrust_motor.trigger_motor.speed_p,
                                                                   thrust_motor.trigger_motor.motor_measure->speed_rpm,
                                                                   thrust_motor.trigger_motor.speed);
                if (fabs(get_trigger_distance - trigger_to_base_distance_set[num_launched]) < 0.1) {
                    flags.is_ready2_trigger_ok = true;
                    thrust_motor.trigger_motor.give_current = 0;
                }
            }
        }
        if(flags.is_ready2_trigger_open_ok==false&&flags.is_ready2_trigger_ok==true)
        {
            trigger_open();
            thrust_motor.thrust_move_motor.speed_p.set = pid_calc(&thrust_motor.thrust_move_motor.angle_p,
                                                                  get_thrust_move_distance,
                                                                  back_thrust_move_distance1);
            thrust_motor.thrust_move_motor.give_current = pid_calc(&thrust_motor.thrust_move_motor.speed_p,
                                                                   thrust_motor.thrust_move_motor.motor_measure->speed_rpm,
                                                                   thrust_motor.thrust_move_motor.speed_p.set);
            if(fabs(get_thrust_move_distance-back_thrust_move_distance1)<1)
            {
                flags.is_ready2_trigger_open_ok=true;
                thrust_motor.thrust_move_motor.give_current=0;
            }

        }
        if(flags.is_ready2_trigger_open_ok==true&&flags.is_ready2_turn_init_ok==false)
        {
            set_dm8009_pos_speed(CAN_1,
                                 TURN_MOTOR_ID,
                                 turn_motor_angle_set[turn_angle],
                                 1.6f);
            if(fabs(turn_motor.pos_r-turn_motor_angle_set[turn_angle])<0.01)
            {
                flags.is_ready2_turn_init_ok=true;
            }
        }
        if (flags.is_ready2_turn_init_ok == true)
        {
            launcher_dart.push_motor_l.speed_p.set = pid_calc(&launcher_dart.push_motor_l.angle_p,
                                                              get_drive_left_distance,
                                                              ready2_load_set_drive_left_distance);
            launcher_dart.push_motor_l.give_current = pid_calc(&launcher_dart.push_motor_l.speed_p,
                                                               launcher_dart.push_motor_l.motor_measure->speed_rpm,
                                                               launcher_dart.push_motor_l.speed_p.set);
            launcher_dart.push_motor_r.give_current=-launcher_dart.push_motor_l.give_current;
            if (fabs(get_drive_left_distance - ready2_load_set_drive_left_distance) < 15.5f)
            {
                flags.is_ready2_drive_load_ok = true;
            }
        }
        if (flags.is_ready2_drive_load_ok == true&&flags.is_ready2_turn_load_ok ==false)
        {
            if(flags.is_ready2_turn_angle_ok_load==false)
            {
                turn_angle=turn_angle+1;
                if(turn_angle==6)turn_angle=4;
                flags.is_ready2_turn_angle_ok_load=true;
                time_test1=HAL_GetTick();
            }

            set_dm8009_pos_speed(CAN_1,
                                 TURN_MOTOR_ID,
                                 turn_motor_angle_set[turn_angle],
                                 1.6f);
            if(fabs(turn_motor.pos_r-turn_motor_angle_set[turn_angle])<0.01)
            {
                flags.is_ready2_turn_load_ok=true;
            }
        }
        if (flags.is_ready2_turn_load_ok == true&&flags.is_ready2_thrust_angle_goal_ok==false)
        {
            thrust_motor.thrust_angle_motor.speed_p.set = pid_loop_calc(&thrust_motor.thrust_angle_motor.angle_p,
                                                                        get_angle_thrust_angle,
                                                                        ready2_thrust_angle_goal_angle,
                                                                        180, -180);
            thrust_motor.thrust_angle_motor.give_current = pid_calc(&thrust_motor.thrust_angle_motor.speed_p,
                                                                    thrust_motor.thrust_angle_motor.motor_measure->speed_rpm,
                                                                    thrust_motor.thrust_angle_motor.speed_p.set);
            if (fabs(thrust_motor.thrust_angle_motor.angle_p.get - ready2_thrust_angle_goal_angle) < 1)
            {
                flags.is_ready2_thrust_angle_goal_ok = true;
                thrust_motor.thrust_angle_motor.give_current=0;
            }
        }

        if (flags.is_ready2_thrust_angle_goal_ok == true)
        {
            thrust_motor.thrust_move_motor.speed_p.set = pid_calc(&thrust_motor.thrust_move_motor.angle_p,
                                                                  get_thrust_move_distance,
                                                                  ready2_thrust_move_goal_distance);
            thrust_motor.thrust_move_motor.give_current = pid_calc(&thrust_motor.thrust_move_motor.speed_p,
                                                                   thrust_motor.thrust_move_motor.motor_measure->speed_rpm,
                                                                   thrust_motor.thrust_move_motor.speed_p.set);
            if (fabs(thrust_motor.thrust_move_motor.angle_p.get - ready2_thrust_move_goal_distance) < 5)
            {
                flags.is_ready2_thrust_move_goal_ok = true;
                thrust_motor.thrust_move_motor.give_current=0;
            }
        }

        if (flags.is_ready2_thrust_move_goal_ok== true)
        {
            thrust_motor.thrust_move_motor.speed_p.set = pid_calc(&thrust_motor.thrust_move_motor.angle_p,
                                                                  get_thrust_move_distance,
                                                                  back_thrust_move_distance1);
            thrust_motor.thrust_move_motor.give_current = pid_calc(&thrust_motor.thrust_move_motor.speed_p,
                                                                   thrust_motor.thrust_move_motor.motor_measure->speed_rpm,
                                                                   thrust_motor.thrust_move_motor.speed_p.set);

            if (fabs(thrust_motor.thrust_move_motor.angle_p.get - back_thrust_move_distance1) < 1)
            {
                flags.is_ready2_thrust_move_back_ok = true;
                thrust_motor.thrust_move_motor.give_current=0;
            }
        }

        if (flags.is_ready2_thrust_move_back_ok == true&&flags.is_ready2_thrust_angle_back_ok==false)
        {
            thrust_motor.thrust_angle_motor.speed_p.set = pid_loop_calc(&thrust_motor.thrust_angle_motor.angle_p,
                                                                        get_angle_thrust_angle,
                                                                        back_thrust_angle_angle,
                                                                        180, -180);
            thrust_motor.thrust_angle_motor.give_current = pid_calc(&thrust_motor.thrust_angle_motor.speed_p,
                                                                    thrust_motor.thrust_angle_motor.motor_measure->speed_rpm,
                                                                    thrust_motor.thrust_angle_motor.speed_p.set);
            if (fabs(thrust_motor.thrust_angle_motor.angle_p.get - back_thrust_angle_angle) < 1)
            {
                flags.is_ready2_thrust_angle_back_ok = true;
                thrust_motor.thrust_angle_motor.give_current=0;
            }
        }
//将滑台推下去锁住
        if (flags.is_ready2_thrust_move_goal_ok == true)
        {
            if (ready2_set_flag == 0) {
                time_now=HAL_GetTick();
                ready2_set_flag = 1;
                set_goal_drive_distance();
            }
            if(HAL_GetTick()-time_now>=1500) {
                launcher_dart.push_motor_l.speed_p.set = pid_calc(&launcher_dart.push_motor_l.angle_p,
                                                                  get_drive_left_distance,
                                                                  ready2_goal_set_drive_left_distance);
                launcher_dart.push_motor_l.give_current = pid_calc(&launcher_dart.push_motor_l.speed_p,
                                                                   launcher_dart.push_motor_l.motor_measure->speed_rpm,
                                                                   launcher_dart.push_motor_l.speed_p.set);
                launcher_dart.push_motor_r.give_current = -launcher_dart.push_motor_l.give_current;
                if (fabs(get_drive_left_distance - ready2_goal_set_drive_left_distance) < 8.0f) {
                    trigger_off();
                    if (ready2_load_flag2 == false) {
                        ready2_load_time = HAL_GetTick();
                        ready2_load_flag2 = true;
                    } else if (HAL_GetTick() - ready2_load_time > 500) {
                        flags.is_ready2_drive_goal_ok = true;
                    }
                }
            }
        }
        if(flags.is_ready2_thrust_move_goal_ok==true)
        {
            flags.is_ready2_trigger_off_ok=true;
        }
        //锁住后， 换弹动
        if (flags.is_ready2_trigger_off_ok == true&&flags.is_ready2_turn_end_ok ==false&&flags.is_ready2_thrust_angle_back_ok==true&&flags.is_ready2_drive_goal_ok==true)
        {
            if(flags.is_ready2_turn_end_ok==false)
            {
                turn_angle=turn_angle+1;
                if(turn_angle==6)turn_angle=4;
                flags.is_ready2_turn_end_ok=true;
            }
            set_dm8009_pos_speed(CAN_1,
                                 TURN_MOTOR_ID,
                                 turn_motor_angle_set[turn_angle],
                                 1.6f);
            if(fabs(turn_motor.pos_r-turn_motor_angle_set[turn_angle])<0.01)
            {
                flags.is_ready2_turn_end_ok=true;
            }
        }
        //换弹电机到位后好后，滑台上去
        if (flags.is_ready2_turn_end_ok == true)
        {
            if(ready2_back_flag==false)
            {
                ready2_back_flag=true;
                ready2_back_time=HAL_GetTick();
            }
            if(HAL_GetTick()-ready2_back_time<500)
            {
                launcher_dart.push_motor_l.speed_p.set=-500;
                launcher_dart.push_motor_l.give_current = pid_calc(&launcher_dart.push_motor_l.speed_p,
                                                                   launcher_dart.push_motor_l.motor_measure->speed_rpm,
                                                                   launcher_dart.push_motor_l.speed_p.set);
            }
            if(HAL_GetTick()-ready2_back_time>500)
            {
                launcher_dart.push_motor_l.speed_p.set = pid_calc(&launcher_dart.push_motor_l.angle_p,
                                                                  get_drive_left_distance,
                                                                  ready2_back_drive_left_distance);
                launcher_dart.push_motor_l.give_current = pid_calc(&launcher_dart.push_motor_l.speed_p,
                                                                   launcher_dart.push_motor_l.motor_measure->speed_rpm,
                                                                   launcher_dart.push_motor_l.speed_p.set);
            }
            launcher_dart.push_motor_r.give_current=-launcher_dart.push_motor_l.give_current;
            if (fabs(get_drive_left_distance - ready2_back_drive_left_distance) < 10)
            {
                flags.is_ready2_drive_back_ok = true;
            }
        }

        if (flags.is_ready2_drive_back_ok == true)
        {
            flags.is_ready2_ok = true;
        }
        if (flags.is_ready2_ok == true) {

            flags.is_ready2_thrust_back_ok = false;
            flags.is_ready2_drive_back_ok = false;
            flags.is_ready2_drive_goal_ok = false;
            flags.is_ready2_turn_end_ok = false;
            flags.is_ready2_thrust_angle_back_ok = false;
            flags.is_ready2_thrust_move_back_ok = false;
            flags.is_ready2_thrust_move_goal_ok = false;
            flags.is_ready2_turn_load_ok = false;
            flags.is_ready2_drive_load_ok = false;
            flags.is_ready2_turn_init_ok = false;
            flags.is_ready2_trigger_off_ok=false;
            flags.is_ready2_turn_angle_ok_end=false;
            flags.is_ready2_turn_angle_ok_init=false;
            flags.is_ready2_turn_angle_ok_load=false;
            flags.is_ready2_thrust_angle_goal_ok=false;
            flags.is_ready2_trigger_open_ok=false;
            flags.is_ready2_trigger_open_first=false;
            flags.is_ready2_trigger_ok=false;
            flags.is_ready2_drive_find_ok=false;
            ready2_load_flag2=false;
            ready2_set_flag=0;
            ready2_load_flag=false;
            ready2_goal_flag=false;
            ready2_back_flag=false;
            ready2_load_flag=false;
        }
    }
}

static void dart_ready_handle()
{
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

    if(dart_goal==GOAL_FRONT_STATION)
    {
        gimbal_dart.motor_yaw.angle_p.set=yaw_angle_goal[front];
        thrust_motor.trigger_motor.speed= pid_calc(&thrust_motor.trigger_motor.angle_p,get_trigger_distance,trigger_to_outposts_distance_set[num_launched]);
        thrust_motor.trigger_motor.give_current= pid_calc(&thrust_motor.trigger_motor.speed_p,thrust_motor.trigger_motor.motor_measure->speed_rpm,thrust_motor.trigger_motor.speed);

        gimbal_dart.motor_yaw.speed_p.set = pid_calc(&gimbal_dart.motor_yaw.angle_p,
                                                     gimbal_dart.motor_yaw.angle_p.get,
                                                     gimbal_dart.motor_yaw.angle_p.set);
        gimbal_dart.motor_yaw.give_current = pid_calc(&gimbal_dart.motor_yaw.speed_p,
                                                      gimbal_dart.motor_yaw.motor_measure->speed_rpm,
                                                      gimbal_dart.motor_yaw.speed_p.set);
    }else
    {
        gimbal_dart.motor_yaw.angle_p.set=yaw_angle_goal[base];
        thrust_motor.trigger_motor.speed= pid_calc(&thrust_motor.trigger_motor.angle_p,get_trigger_distance,trigger_to_base_distance_set[num_launched]);
        thrust_motor.trigger_motor.give_current= pid_calc(&thrust_motor.trigger_motor.speed_p,thrust_motor.trigger_motor.motor_measure->speed_rpm,thrust_motor.trigger_motor.speed);

        gimbal_dart.motor_yaw.speed_p.set = pid_calc(&gimbal_dart.motor_yaw.angle_p,
                                                     gimbal_dart.motor_yaw.angle_p.get,
                                                     gimbal_dart.motor_yaw.angle_p.set);
        gimbal_dart.motor_yaw.give_current = pid_calc(&gimbal_dart.motor_yaw.speed_p,
                                                      gimbal_dart.motor_yaw.motor_measure->speed_rpm,
                                                      gimbal_dart.motor_yaw.speed_p.set);
    }
}

static void ready_control()
{
    if(ready2_load_flag==0)
    {
        set_load_drive_distance();
        ready2_load_flag=1;
    }
    if(flags.is_ceshi_ok==false&&ceshi_flag==1) {
        if (flags.is_ceshi_trigger_ok == false) {
                thrust_motor.trigger_motor.speed = pid_calc(&thrust_motor.trigger_motor.angle_p, get_trigger_distance,
                                                            trigger_to_base_distance_set[ceshi_launched]);
                thrust_motor.trigger_motor.give_current = pid_calc(&thrust_motor.trigger_motor.speed_p,
                                                                   thrust_motor.trigger_motor.motor_measure->speed_rpm,
                                                                   thrust_motor.trigger_motor.speed);
                if (fabs(get_trigger_distance - trigger_to_base_distance_set[ceshi_launched]) < 1) {
                    flags.is_ceshi_trigger_ok = true;
                    thrust_motor.trigger_motor.give_current = 0;
                }
        }
        if (flags.is_ceshi_trigger_open_ok == false && flags.is_ceshi_trigger_ok == true) {
            trigger_open();
            flags.is_ceshi_trigger_open_ok = true;
        }
        if (flags.is_ceshi_trigger_open_ok == true && flags.is_ceshi_turn_init_ok == false) {
            set_dm8009_pos_speed(CAN_1,
                                 TURN_MOTOR_ID,
                                 turn_motor_angle_set[turn_angle],
                                 1.6f);
            if (fabs(turn_motor.pos_r - turn_motor_angle_set[turn_angle]) < 0.01) {
                flags.is_ceshi_turn_init_ok = true;
            }
        }
        if (flags.is_ceshi_turn_init_ok == true) {
            launcher_dart.push_motor_l.speed_p.set = pid_calc(&launcher_dart.push_motor_l.angle_p,
                                                              get_drive_left_distance,
                                                              ready2_load_set_drive_left_distance);
            launcher_dart.push_motor_l.give_current = pid_calc(&launcher_dart.push_motor_l.speed_p,
                                                               launcher_dart.push_motor_l.motor_measure->speed_rpm,
                                                               launcher_dart.push_motor_l.speed_p.set);
            launcher_dart.push_motor_r.give_current = -launcher_dart.push_motor_l.give_current;
            if (fabs(get_drive_left_distance - ready2_load_set_drive_left_distance) < 15.5f) {
                flags.is_ceshi_drive_load_ok = true;
            }
        }
        if (flags.is_ceshi_drive_load_ok == true && flags.is_ceshi_turn_load_ok == false) {
            if (flags.is_ceshi_turn_angle_ok_load == false) {
                turn_angle = turn_angle + 1;
                if (turn_angle == 6)turn_angle = 0;
                angle_now = turn_motor.pos_r;
                flags.is_ceshi_turn_angle_ok_load = true;
                time_test1 = HAL_GetTick();
            }
            set_dm8009_pos_speed(CAN_1,
                                 TURN_MOTOR_ID,
                                 turn_motor_angle_set[turn_angle],
                                 1.6f);
            if (fabs(turn_motor.pos_r - turn_motor_angle_set[turn_angle]) < 0.01) {
                flags.is_ceshi_turn_load_ok = true;
            }
        }
        if (flags.is_ceshi_turn_load_ok == true && flags.is_ceshi_thrust_angle_goal_ok == false) {
            thrust_motor.thrust_angle_motor.speed_p.set = pid_loop_calc(&thrust_motor.thrust_angle_motor.angle_p,
                                                                        get_angle_thrust_angle,
                                                                        ready2_thrust_angle_goal_angle,
                                                                        180, -180);
            thrust_motor.thrust_angle_motor.give_current = -pid_calc(&thrust_motor.thrust_angle_motor.speed_p,
                                                                     thrust_motor.thrust_angle_motor.motor_measure->speed_rpm,
                                                                     thrust_motor.thrust_angle_motor.speed_p.set);
            if (fabs(thrust_motor.thrust_angle_motor.angle_p.get - ready2_thrust_angle_goal_angle) < 1) {
                flags.is_ceshi_thrust_angle_goal_ok = true;
                thrust_motor.thrust_angle_motor.give_current = 0;
            }
        }

        if (flags.is_ceshi_thrust_angle_goal_ok == true) {

            thrust_motor.thrust_move_motor.speed_p.set = pid_calc(&thrust_motor.thrust_move_motor.angle_p,
                                                                  get_thrust_move_distance,
                                                                  ready2_thrust_move_goal_distance);
            thrust_motor.thrust_move_motor.give_current = pid_calc(&thrust_motor.thrust_move_motor.speed_p,
                                                                   thrust_motor.thrust_move_motor.motor_measure->speed_rpm,
                                                                   thrust_motor.thrust_move_motor.speed_p.set);
            if (fabs(thrust_motor.thrust_move_motor.angle_p.get - ready2_thrust_move_goal_distance) < 5) {
                flags.is_ceshi_thrust_move_goal_ok = true;
            }
        }

        if (flags.is_ceshi_thrust_move_goal_ok == true) {

            thrust_motor.thrust_move_motor.speed_p.set = pid_calc(&thrust_motor.thrust_move_motor.angle_p,
                                                                  get_thrust_move_distance,
                                                                  back_thrust_move_distance1);
            thrust_motor.thrust_move_motor.give_current = pid_calc(&thrust_motor.thrust_move_motor.speed_p,
                                                                   thrust_motor.thrust_move_motor.motor_measure->speed_rpm,
                                                                   thrust_motor.thrust_move_motor.speed_p.set);

            if (fabs(thrust_motor.thrust_move_motor.angle_p.get - back_thrust_move_distance1) < 1) {
                flags.is_ceshi_thrust_move_back_ok = true;
            }
        }

        if (flags.is_ceshi_thrust_move_back_ok == true && flags.is_ceshi_thrust_angle_back_ok == false) {

            thrust_motor.thrust_angle_motor.speed_p.set = pid_loop_calc(&thrust_motor.thrust_angle_motor.angle_p,
                                                                        get_angle_thrust_angle,
                                                                        back_thrust_angle_angle,
                                                                        180, -180);
            thrust_motor.thrust_angle_motor.give_current = -pid_calc(&thrust_motor.thrust_angle_motor.speed_p,
                                                                     thrust_motor.thrust_angle_motor.motor_measure->speed_rpm,
                                                                     thrust_motor.thrust_angle_motor.speed_p.set);
            if (fabs(thrust_motor.thrust_angle_motor.angle_p.get - back_thrust_angle_angle) < 1) {
                flags.is_ceshi_thrust_angle_back_ok = true;
                thrust_motor.thrust_angle_motor.give_current = 0;
            }
        }

//将滑台推下去锁住
        if (flags.is_ceshi_thrust_angle_back_ok == true) {
            if (ready2_set_flag == 0) {
                ready2_goal_flag = 1;
                set_goal_drive_distance();
            }
            launcher_dart.push_motor_l.speed_p.set = pid_calc(&launcher_dart.push_motor_l.angle_p,
                                                              get_drive_left_distance,
                                                              ready2_goal_set_drive_left_distance);
            launcher_dart.push_motor_l.give_current = pid_calc(&launcher_dart.push_motor_l.speed_p,
                                                               launcher_dart.push_motor_l.motor_measure->speed_rpm,
                                                               launcher_dart.push_motor_l.speed_p.set);
            launcher_dart.push_motor_r.give_current = -launcher_dart.push_motor_l.give_current;
            if (fabs(get_drive_left_distance - ready2_goal_set_drive_left_distance) < 10.5f) {
                if (ready2_load_flag2 == false) {
                    ready2_load_time = HAL_GetTick();
                    ready2_load_flag2 = true;
                } else if (HAL_GetTick() - ready2_load_time > 500) {
                    flags.is_ceshi_drive_goal_ok = true;
                }
            }
        }
        if (flags.is_ceshi_drive_goal_ok == true) {
            flags.is_ceshi_trigger_off_ok = true;
        }
        //锁住后， 换弹动
        if (flags.is_ceshi_trigger_off_ok == true && flags.is_ceshi_turn_end_ok == false) {
            if (flags.is_ceshi_turn_angle_ok_end == false) {
                turn_angle = turn_angle + 1;
                if (turn_angle == 6)turn_angle = 0;
                flags.is_ceshi_turn_angle_ok_end = true;
            }
            set_dm8009_pos_speed(CAN_1,
                                 TURN_MOTOR_ID,
                                 turn_motor_angle_set[turn_angle],
                                 1.6f);
            if (fabs(turn_motor.pos_r - turn_motor_angle_set[turn_angle]) < 0.01) {
                flags.is_ceshi_turn_end_ok = true;
            }
        }
        //换弹电机到位后好后，滑台上去
        if (flags.is_ceshi_turn_end_ok == true) {

            if (ready2_back_flag == false) {
                ready2_back_flag = true;
                ready2_back_time = HAL_GetTick();
            }
            if (HAL_GetTick() - ready2_back_time < 500) {
                launcher_dart.push_motor_l.speed_p.set = -500;
                launcher_dart.push_motor_l.give_current = pid_calc(&launcher_dart.push_motor_l.speed_p,
                                                                   launcher_dart.push_motor_l.motor_measure->speed_rpm,
                                                                   launcher_dart.push_motor_l.speed_p.set);
            }
            if (HAL_GetTick() - ready2_back_time > 500) {
                launcher_dart.push_motor_l.speed_p.set = pid_calc(&launcher_dart.push_motor_l.angle_p,
                                                                  get_drive_left_distance,
                                                                  ready2_back_drive_left_distance);
                launcher_dart.push_motor_l.give_current = pid_calc(&launcher_dart.push_motor_l.speed_p,
                                                                   launcher_dart.push_motor_l.motor_measure->speed_rpm,
                                                                   launcher_dart.push_motor_l.speed_p.set);
            }
            launcher_dart.push_motor_r.give_current = -launcher_dart.push_motor_l.give_current;
            if (fabs(get_drive_left_distance - ready2_back_drive_left_distance) < 10) {
                flags.is_ceshi_drive_back_ok = true;
                launcher_dart.push_motor_r.give_current=0;
                launcher_dart.push_motor_l.give_current=0;
            }
        }
        if(flags.is_ceshi_drive_back_ok==true)
        {
            flags.is_ceshi_ok=false;
            flags.is_ceshi_thrust_back_ok = false;
            flags.is_ceshi_drive_back_ok = false;
            flags.is_ceshi_drive_goal_ok = false;
            flags.is_ceshi_turn_end_ok = false;
            flags.is_ceshi_thrust_angle_back_ok = false;
            flags.is_ceshi_thrust_move_back_ok = false;
            flags.is_ceshi_thrust_move_goal_ok = false;
            flags.is_ceshi_turn_load_ok = false;
            flags.is_ceshi_drive_load_ok = false;
            flags.is_ceshi_turn_init_ok = false;
            flags.is_ceshi_trigger_off_ok=false;
            flags.is_ceshi_turn_angle_ok_end=false;
            flags.is_ceshi_turn_angle_ok_init=false;
            flags.is_ceshi_turn_angle_ok_load=false;
            flags.is_ceshi_thrust_angle_goal_ok=false;
            flags.is_ceshi_trigger_open_ok=false;
            flags.is_ceshi_trigger_open_first=false;
            flags.is_ceshi_trigger_ok=false;
            flags.is_ceshi_drive_find_ok=false;
            ready2_load_flag2=false;
            ready2_load_flag=false;
            ready2_goal_flag=false;
            ready2_back_flag=false;
            ready2_load_flag=false;
            ceshi_flag=0;
            launcher_dart.push_motor_r.give_current=0;
            launcher_dart.push_motor_l.give_current=0;
        }
    }
}

static fp32 yaw_distance_conversion(int32_t ecd)
{
    return (fp32)ecd/8189*5;
}

//已完成
static void dart_control_handle()
{
//    if (fabs(filter_yaw_in.out) > 150) {
//        gimbal_dart.motor_yaw.angle_p.set = gimbal_dart.motor_yaw.angle_p.get - filter_yaw_in.out/ 70;
//    } else if (fabs(filter_yaw_in.out) > 50 && fabs(filter_yaw_in.out) < 150) {
//        gimbal_dart.motor_yaw.angle_p.set = gimbal_dart.motor_yaw.angle_p.get - filter_yaw_in.out / 60;
//    } else if (fabs(filter_yaw_in.out) < 50 && fabs(filter_yaw_in.out) > 30) {
//        gimbal_dart.motor_yaw.angle_p.set = gimbal_dart.motor_yaw.angle_p.get - filter_yaw_in.out / 55;
//    } else if (fabs(filter_yaw_in.out) < 30) {
//        gimbal_dart.motor_yaw.angle_p.set = gimbal_dart.motor_yaw.angle_p.get - filter_yaw_in.out / 55;
//    }
    gimbal_dart.motor_yaw.angle_p.set = gimbal_dart.motor_yaw.angle_p.get - filter_yaw_in.out/ 80;
    if(gimbal_dart.motor_yaw.angle_p.set>=25.0)
    {
        gimbal_dart.motor_yaw.angle_p.set=25;
    }
    else if(gimbal_dart.motor_yaw.angle_p.set<=-25)
    {
        gimbal_dart.motor_yaw.angle_p.set=-25;
    }
    gimbal_dart.motor_yaw.speed_p.set = pid_calc(&gimbal_dart.motor_yaw.angle_p,
                                                 gimbal_dart.motor_yaw.angle_p.get,
                                                 gimbal_dart.motor_yaw.angle_p.set);
    gimbal_dart.motor_yaw.give_current = pid_calc(&gimbal_dart.motor_yaw.speed_p,
                                                  gimbal_dart.motor_yaw.motor_measure->speed_rpm,
                                                  gimbal_dart.motor_yaw.speed_p.set);
    if(fabs(filter_yaw_in.out)<=5)
    {
        gimbal_dart.motor_yaw.give_current=0;
    }

    if(rc_ctrl.rc.ch[4]<-500)
    {
        trigger_off();
    }
    if(rc_ctrl.rc.ch[4]>500)
    {
        trigger_open();
    }
//    if(rc_ctrl.rc.ch[2]>0&&rc_ctrl.rc.ch[3]>0&&rc_ctrl.rc.ch[0]<0&&rc_ctrl.rc.ch[1]>0)
//    {
//        ceshi_flag=1;
//    }
//    if(ceshi_flag==1)
//    {
//        ready_control();
//    }
    //yaw_control();
    set_dm8009_enable(CAN_1,TURN_MOTOR_ID);
    turn_control();
    drive_control();
    //trigger_control();
    //thrust_motor_angle_control();
    //thrust_motor_move_control();
    //steer_motor_control();
}

static void steer_motor_control()
{
    cnt=rc_ctrl.rc.ch[1]*10;
    if(cnt>2500)
    {
        cnt=2500;
    }
    if(cnt<500)
    {
        cnt=500;
    }
    __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,cnt);

    vTaskDelay(10);
}

static void thrust_motor_move_control()
{
    if(rc_ctrl.rc.ch[0]<20&&rc_ctrl.rc.ch[0]>-20)
    {
        rc_ctrl.rc.ch[0]=0;
    }
    set_thrust_move_distance += rc_ctrl.rc.ch[0]*0.0005;
    get_thrust_move_distance= thrust_move_distance_conversion(thrust_motor.thrust_move_motor.motor_measure->total_ecd);
    thrust_motor.thrust_move_motor.speed_p.set= pid_calc(&thrust_motor.thrust_move_motor.angle_p,
                                                         get_thrust_move_distance,
                                                         set_thrust_move_distance);
    //thrust_motor.thrust_move_motor.speed_p.set=rc_ctrl.rc.ch[2]*0.5;
    thrust_motor.thrust_move_motor.give_current= pid_calc(&thrust_motor.thrust_move_motor.speed_p,
                                                          thrust_motor.thrust_move_motor.motor_measure->speed_rpm,
                                                          thrust_motor.thrust_move_motor.speed_p.set);
    //thrust_motor.thrust_move_motor.give_current=(int16_t)thrust_motor.thrust_move_motor.give_current;

}

static void thrust_motor_angle_control()
{
    set_angle_thrust_angle-=rc_ctrl.rc.ch[1]*0.0001;
//    if(set_angle_thrust_angle<4.59)
//    {
//        set_angle_thrust_angle = 4.59f;
//    }
//    if(set_angle_thrust_angle > 60.02)
//    {
//        set_angle_thrust_angle =60.02f;
//    }
    //get_angle_thrust_angle= ecd_to_angle(thrust_motor.thrust_angle_motor.motor_measure->total_ecd);
    thrust_motor.thrust_angle_motor.speed_p.set= pid_loop_calc(&thrust_motor.thrust_angle_motor.angle_p,
                                                               get_angle_thrust_angle,
                                                               set_angle_thrust_angle,
                                                               180,-180);
    thrust_motor.thrust_angle_motor.give_current= pid_calc(&thrust_motor.thrust_angle_motor.speed_p,
                                                           thrust_motor.thrust_angle_motor.motor_measure->speed_rpm,
                                                           thrust_motor.thrust_angle_motor.speed_p.set);
}

static fp32 trigger_distance_conversion(int32_t ecd)
{
    return (fp32)ecd/8192/36*4;
}

static fp32 drive_distance_conversion(int32_t ecd)
{
    return (fp32)ecd/8192/3591*187*27*PI;
}
static fp32 thrust_move_distance_conversion(int32_t ecd)
{
    return (fp32)ecd/8192/36*18.34*PI;
}

static void trigger_control()
{
    get_trigger_distance=trigger_distance_conversion(thrust_motor.trigger_motor.motor_measure->total_ecd);
    //init_distance=distance(init_ecd_trigger);
    if(rc_ctrl.rc.ch[1]<=20&&rc_ctrl.rc.ch[1]>=-20)
    {
        rc_ctrl.rc.ch[1]=0;
    }
    set_trigger_distance += rc_ctrl.rc.ch[1]*0.0005;
    if(set_trigger_distance<= trigger_distance_conversion(init_ecd_trigger)+10)
    {
        set_trigger_distance= trigger_distance_conversion(init_ecd_trigger)+10;
    }

    thrust_motor.trigger_motor.speed_p.set= pid_calc(&thrust_motor.trigger_motor.angle_p,
                                                     get_trigger_distance,
                                                     set_trigger_distance);
    thrust_motor.trigger_motor.give_current= pid_calc(&thrust_motor.trigger_motor.speed_p,
                                                      thrust_motor.trigger_motor.motor_measure->speed_rpm,
                                                      thrust_motor.trigger_motor.speed_p.set);
//    thrust_motor.trigger_motor.give_current=rc_ctrl.rc.ch[3]*16;
//    abs_limit(&thrust_motor.trigger_motor.give_current,9000);
//    thrust_motor.trigger_motor.give_current=init_speed_trigger;
}

static void drive_control()
{
    get_drive_right_distance= drive_distance_conversion(launcher_dart.push_motor_r.motor_measure->total_ecd);
    get_drive_left_distance= drive_distance_conversion(launcher_dart.push_motor_l.motor_measure->total_ecd);
    if(rc_ctrl.rc.ch[3]<=20&&rc_ctrl.rc.ch[3]>=-20)
    {
        rc_ctrl.rc.ch[3]=0;
    }
    set_drive_right_distance += rc_ctrl.rc.ch[3]*0.001;
    set_drive_left_distance -= rc_ctrl.rc.ch[3]*0.001;
    launcher_dart.push_motor_l.speed_p.set= pid_calc(&launcher_dart.push_motor_l.angle_p,
                                                     get_drive_left_distance,
                                                     set_drive_left_distance);
    launcher_dart.push_motor_l.give_current= pid_calc(&launcher_dart.push_motor_l.speed_p,
                                                      launcher_dart.push_motor_l.motor_measure->speed_rpm,
                                                      launcher_dart.push_motor_l.speed_p.set);
    launcher_dart.push_motor_r.give_current=-launcher_dart.push_motor_l.give_current;
}

bool fl=0;
fp32 goal_angle=0;
fp32 goal=0;
fp32 err;
static void turn_control()
{
    if(rc_ctrl.rc.ch[1]<=20&&rc_ctrl.rc.ch[1]>=-20)
    {
        rc_ctrl.rc.ch[1]=0;
    }
    goal+=rc_ctrl.rc.ch[1]*0.001*0.1;

    set_dm8009_pos_speed(CAN_1,
                         TURN_MOTOR_ID,
                         goal,
                         1.6f);

}

static void yaw_control()
{
    if(rc_ctrl.rc.ch[2]<=40&&rc_ctrl.rc.ch[2]>=-40)
    {
        rc_ctrl.rc.ch[2]=0;
    }
        gimbal_dart.motor_yaw.angle_p.set += rc_ctrl.rc.ch[2] * 0.03*0.02;
        gimbal_dart.motor_yaw.speed_p.set = pid_calc(&gimbal_dart.motor_yaw.angle_p,
                                                     gimbal_dart.motor_yaw.angle_p.get,
                                                     gimbal_dart.motor_yaw.angle_p.set);
        gimbal_dart.motor_yaw.give_current = pid_calc(&gimbal_dart.motor_yaw.speed_p,
                                                      gimbal_dart.motor_yaw.motor_measure->speed_rpm,
                                                      gimbal_dart.motor_yaw.speed_p.set);
}

//已完成
static void dart_back_handle()
{
    gimbal_dart.motor_yaw.speed_p.set= pid_calc(&gimbal_dart.motor_yaw.angle_p,
                                                gimbal_dart.motor_yaw.angle_p.get,
                                                back_yaw);
    gimbal_dart.motor_yaw.give_current= pid_calc(&gimbal_dart.motor_yaw.speed_p,
                                                 gimbal_dart.motor_yaw.motor_measure->speed_rpm,
                                                 gimbal_dart.motor_yaw.speed_p.set);

    thrust_motor.trigger_motor.speed_p.set = pid_calc(&thrust_motor.trigger_motor.angle_p,
                                                      get_trigger_distance,
                                                      back_trigger_distance);
    thrust_motor.trigger_motor.give_current = pid_calc(&thrust_motor.trigger_motor.speed_p,
                                                       thrust_motor.trigger_motor.motor_measure->speed_rpm,
                                                       thrust_motor.trigger_motor.speed_p.set);


    launcher_dart.push_motor_l.speed_p.set = pid_calc(&launcher_dart.push_motor_l.angle_p,
                                                      get_drive_left_distance,
                                                      back_drive_left_distance);
    launcher_dart.push_motor_l.give_current = pid_calc(&launcher_dart.push_motor_l.speed_p,
                                                       launcher_dart.push_motor_l.motor_measure->speed_rpm,
                                                       launcher_dart.push_motor_l.speed_p.set);
    launcher_dart.push_motor_r.give_current = -launcher_dart.push_motor_l.give_current;



    if(flags.is_back_ok==false) {
        if (flags.is_back_turn_ok == false) {
            set_dm8009_enable(CAN_1, TURN_MOTOR_ID);
            set_dm8009_pos_speed(CAN_1,
                                 TURN_MOTOR_ID,
                                 turn_motor_angle_set[turn_angle],
                                 1.6f);
            if (fabs(turn_motor.pos_r - turn_motor_angle_set[turn_angle]) < 0.01) {
                flags.is_back_turn_ok = true;
            }
        }
        if (flags.is_back_turn_ok == true) {
            thrust_motor.thrust_angle_motor.speed_p.set= pid_loop_calc(&thrust_motor.thrust_angle_motor.angle_p,
                                                                       get_angle_thrust_angle,
                                                                       back_thrust_angle_angle,
                                                                       180,-180);
            thrust_motor.thrust_angle_motor.give_current= pid_calc(&thrust_motor.thrust_angle_motor.speed_p,
                                                                   thrust_motor.thrust_angle_motor.motor_measure->speed_rpm,
                                                                   thrust_motor.thrust_angle_motor.speed_p.set);
            if (fabs(back_thrust_angle_angle - get_angle_thrust_angle) < 1) {
                //thrust_motor.thrust_angle_motor.give_current = 0;
                flags.is_thrust_angle_ok = true;
            }
        }
        if (flags.is_thrust_angle_ok == true) {
            thrust_motor.thrust_move_motor.speed_p.set = pid_calc(&thrust_motor.thrust_move_motor.angle_p,
                                                                  get_thrust_move_distance,
                                                                  back_thrust_move_distance);
            thrust_motor.thrust_move_motor.give_current = pid_calc(&thrust_motor.thrust_move_motor.speed_p,
                                                                   thrust_motor.thrust_move_motor.motor_measure->speed_rpm,
                                                                   thrust_motor.thrust_move_motor.speed_p.set);
            if (fabs(back_thrust_move_distance - get_thrust_move_distance) < 10) {
                flags.is_thrust_move_ok = true;
                thrust_motor.thrust_move_motor.give_current=0;
            }
            flags.is_thrust_move_ok=true;
        }
        if (flags.is_thrust_move_ok == true) {
            if (fabs(get_trigger_distance - back_trigger_distance) < 1) {
                flags.is_back_ok = true;
            }
        }
    }
    if (flags.is_back_ok == true) {
        flags.is_back_drive_ok = false;
        flags.is_trigger_move_ok = false;
        flags.is_thrust_angle_ok = false;
        flags.is_thrust_move_ok = false;
        flags.is_back_turn_ok = false;
        flags.is_back_Drive_first = false;
    }
}

static void dart_relax_handle()
{
    set_dm8009_disable(CAN_1,TURN_MOTOR_ID);
    launcher_dart.push_motor_r.give_current=0;
    launcher_dart.push_motor_l.give_current=0;
    launcher_dart.turn_motor.give_current=0;
    thrust_motor.thrust_move_motor.give_current=0;
    thrust_motor.thrust_angle_motor.give_current=0;
    thrust_motor.trigger_motor.give_current=0;
    gimbal_dart.motor_yaw.give_current=0;
}

static void dart_init()
{

    set_dm_motor_angle=0;

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
             DRIVE_ANGLE_right_PID_KP,
             DRIVE_ANGLE_right_PID_KI,
             DRIVE_ANGLE_right_PID_KD);

    pid_init(&launcher_dart.push_motor_r.speed_p,
             DRIVE_SPEED_MAX_OUT,
             DRIVE_SPEED_MAX_IOUT,
             DRIVE_SPEED_right_PID_KP,
             DRIVE_SPEED_right_PID_KI,
             DRIVE_SPEED_right_PID_KD);

    pid_init(&launcher_dart.push_motor_l.angle_p,
             DRIVE_ANGLE_MAX_OUT,
             DRIVE_ANGLE_MAX_IOUT,
             DRIVE_ANGLE_left_PID_KP,
             DRIVE_ANGLE_left_PID_KI,
             DRIVE_ANGLE_left_PID_KD);

    pid_init(&launcher_dart.push_motor_l.speed_p,
             DRIVE_SPEED_MAX_OUT,
             DRIVE_SPEED_MAX_IOUT,
             DRIVE_SPEED_left_PID_KP,
             DRIVE_SPEED_left_PID_KI,
             DRIVE_SPEED_left_PID_KD);

    pid_init(&launcher_dart.turn_motor.angle_p,
             TURN_ANGLE_MAX_OUT,
             TURN_ANGLE_MAX_IOUT,
             TURN_ANGLE_PID_KP_3,
             TURN_ANGLE_PID_KI_3,
             TURN_ANGLE_PID_KD_3);

    pid_init(&launcher_dart.turn_motor.speed_p,
             TURN_SPEED_MAX_OUT,
             TURN_SPEED_MAX_IOUT,
             TURN_SPEED_PID_KP_3,
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

    first_order_filter_init(&turn_first_order_set, 5,  30);
    gimbal_dart.motor_yaw.motor_measure->offset_ecd=12214;
    launcher_dart.turn_motor.motor_measure->offset_ecd=5459;

    turn_motor_angle_set[0]=2.20414f;
    turn_motor_angle_set[1]=1.1857f;//-1.94964f
    turn_motor_angle_set[2]=0.0872f;
    turn_motor_angle_set[3]=-0.9062f;//2.21143f
    turn_motor_angle_set[4]=-1.81956f;
    turn_motor_angle_set[5]=-2.988288f;//

    launcher_dart.turn_motor.angle_p.set=turn_motor_angle_set[0];
    angle_now=turn_motor_angle_set[0];

}
//已完成
static void dart_mode_set()
{
//    if(switch_is_down(rc_ctrl.rc.s[RC_s_L]) && switch_is_down(rc_ctrl.rc.s[RC_s_R]))
//    {
//        gimbal_dart.last_mode=gimbal_dart.mode;
//        gimbal_dart.mode=DART_RELAX;
//    }
//    if(gimbal_dart.mode==DART_RELAX || gimbal_dart.mode==DART_CONTROL) {
//        if (switch_is_mid(rc_ctrl.rc.s[RC_s_L]) && switch_is_mid(rc_ctrl.rc.s[RC_s_R])) {
//            gimbal_dart.last_mode = gimbal_dart.mode;
//            gimbal_dart.mode = DART_BACK;
//        }
//    }
//    if(gimbal_dart.mode==DART_BACK&&flags.is_back_ok == true&&dart_goal!=0)
//    {
//        flags.is_back_ok=false;
//        gimbal_dart.mode=DART_READY;
//        flags.is_back_ok=false;
//    }
//    if(gimbal_dart.mode==DART_READY&&(flags.is_ready_ok==true||flags.is_ready2_ok==true))
//    {
//        gimbal_dart.mode=DART_TRIGGER;
//        flags.is_ready_ok=false;
//        flags.is_ready2_ok=false;
//    }

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
            //flags.is_back_ok=false;
        }
    }
    if(gimbal_dart.mode==DART_GOAL_SET)
    {
        if(rc_ctrl.rc.ch[2]>400&&rc_ctrl.rc.ch[3]>400&&rc_ctrl.rc.ch[0]<-400&&rc_ctrl.rc.ch[1]>400)
        {
            gimbal_dart.last_mode=gimbal_dart.mode;
            gimbal_dart.mode=DART_READY;
        }
    }
    if(gimbal_dart.mode==DART_READY)
    {
        //num_launched==0记得加回来
        if(switch_is_up(rc_ctrl.rc.s[RC_s_L])&&flags.is_ready_ok==1&&num_launched==0)
        {
            gimbal_dart.last_mode=gimbal_dart.mode;
            gimbal_dart.mode=DART_TRIGGER;
            flags.is_ready_ok=0;
        }
        if(num_launched>0&&!switch_is_up(rc_ctrl.rc.s[RC_s_L])&&flags.is_ready2_ok==1)
        {
            gimbal_dart.last_mode=gimbal_dart.mode;
            gimbal_dart.mode=DART_TRIGGER;
            flags.is_ready2_ok=0;
        }
        else if(num_launched>0&&switch_is_up(rc_ctrl.rc.s[RC_s_L])&&flags.is_ready_ok==1)
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

static void dart_data_update()
{
//    if(switch_is_up(rc_ctrl.rc.s[RC_s_R])&&(gimbal_dart.mode!=DART_TRIGGER&&gimbal_dart.mode!=DART_READY&&gimbal_dart.mode!=DART_RELAX))
//    {
//        gimbal_dart.motor_yaw.angle_p.set=yaw_angle_goal[front];
//        dart_goal=GOAL_FRONT_STATION;
//    }
//    if(switch_is_up(rc_ctrl.rc.s[RC_s_L])&&(gimbal_dart.mode!=DART_TRIGGER&&gimbal_dart.mode!=DART_READY&&gimbal_dart.mode!=DART_RELAX))
//    {
//        gimbal_dart.motor_yaw.angle_p.set=yaw_angle_goal[base];
//        dart_goal=GOAL_BASE_STATION;
//    }
    gimbal_dart.motor_yaw.angle_p.get=yaw_distance_conversion(gimbal_dart.motor_yaw.motor_measure->total_ecd);
    first_order_filter_cali(&filter_yaw_in,robot_ctrl.yaw);

    launcher_dart.turn_motor.angle_p.get= -motor_ecd_to_angle_change(launcher_dart.turn_motor.motor_measure->ecd,launcher_dart.turn_motor.motor_measure->offset_ecd);
    thrust_motor.thrust_angle_motor.angle_p.get=ecd_to_angle(thrust_motor.thrust_angle_motor.motor_measure->total_ecd);
    get_drive_right_distance= drive_distance_conversion(launcher_dart.push_motor_r.motor_measure->total_ecd);
    get_drive_left_distance= drive_distance_conversion(launcher_dart.push_motor_l.motor_measure->total_ecd);
    get_trigger_distance=trigger_distance_conversion(thrust_motor.trigger_motor.motor_measure->total_ecd);
    get_thrust_move_distance= thrust_move_distance_conversion(thrust_motor.thrust_move_motor.motor_measure->total_ecd);
    get_angle_thrust_angle= ecd_to_angle(thrust_motor.thrust_angle_motor.motor_measure->total_ecd);
}