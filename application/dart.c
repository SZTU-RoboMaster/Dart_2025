#include <math.h>
#include "dart.h"
#include "cmsis_os.h"
#include "can_receive.h"
#include "Atti.h"
#include "protocol_shaob.h"
#include "stdlib.h"
#include "user_lib.h"
#include "tim.h"
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
int32_t init_ecd_drive_left;//初始时候推动左电机的total_ecd
int32_t init_ecd_drive_right;//初始时候推动右电机的total_ecd
int32_t init_ecd_thrust_move;//初始化推弹移动电机的total_ecd
int32_t init_ecd_thrust_angle;//初始化推弹角度电机的total_ecd
bool ready1_flag;
fp32 ready1_set_drive_right_distance;
fp32 ready1_set_drive_left_distance;

bool ready2_load_flag;//确定上膛位置标志位
bool ready2_goal_flag;//确定发射位置标志位
bool ready2_set_flag;
fp32 ready2_load_set_drive_right_distance;
fp32 ready2_load_set_drive_left_distance;
fp32 ready2_goal_set_drive_right_distance;
fp32 ready2_goal_set_drive_left_distance;
int16_t cnt;

fp32 trigger_to_outposts_distance_set[4]={9,9,9,9};//到前哨站的距离
fp32 trigger_to_base_distance_set[4]={9,9,9,9};//到基地的距离
fp32 turn_motor_angle_set[6];//换弹电机角度数组
int8_t turn_angle=0;//换弹电机角度数组索引
fp32 thrust_motor_angle_set[2];//推弹电机角度数组
fp32 yaw_angle_goal[2]={-10.318f,-10.318f};//0表示前哨站的角度,1表示基地的角度
bool trigger_move_down_l; //确定左拨杆放下面的标志位
bool trigger_move_mid_l;  //确定左拨杆放中间的标志位
uint8_t trigger_move;      //确定扳机移动的格数
uint8_t begin_count1=1;

int64_t time_now;//测试用
uint8_t flag1=false;//测试用
int16_t init_speed_thrust_move=-1500;//测试用//-1000
int16_t init_speed_drive=900;
int16_t init_speed_trigger=-4000;
int16_t init_speed_thrust_angle=600;//-500
fp32 dart_length=715;
fp32 slide_length=75;

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

fp32 back_trigger_distance;
fp32 back_drive_left_distance;
fp32 back_drive_right_distance;
fp32 back_thrust_move_distance;
fp32 back_thrust_angle_angle;

fp32 ready1_back_drive_left_distance;
fp32 ready1_back_drive_right_distance;

fp32 ready2_thrust_move_goal_distance;
fp32 ready2_thrust_angle_goal_angle;

fp32 trigger_move_distance;


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

/*      滤波      */
first_order_filter_type_t filter_yaw_in;

void dart_task(void const*pvParameters)
{
    vTaskDelay(DART_TASK_INIT_TIME);

    dart_init();
    dart_reset();
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

    back_thrust_angle_angle=init_angle_thrust_angle+10;
    back_thrust_move_distance=init_thrust_move_distance+15;
    back_drive_right_distance=init_drive_right_distance-10;
    back_drive_left_distance=init_drive_left_distance+10;
    back_trigger_distance=init_trigger_distance+40;
    ready1_back_drive_right_distance=init_drive_right_distance-15;
    ready1_back_drive_left_distance=init_drive_left_distance+15;

//    set_drive_left_distance=550+init_drive_left_distance;
//    set_drive_right_distance=-550+init_drive_right_distance;

    ready2_thrust_angle_goal_angle=66.6942+init_angle_thrust_angle;

    ready2_thrust_move_goal_distance=115.5266+init_thrust_move_distance;
}

static fp32 ecd_to_angle(int32_t ecd)
{
    return (fp32)ecd/8192*12;
}

//基本完成差个移动一格移动多少ecd
static void dart_trigger_handle()
{
    gimbal_dart.motor_yaw.angle_p.set-=rc_ctrl.rc.ch[0]*0.01*0.03f;
    if(gimbal_dart.motor_yaw.angle_p.set>=50.0)
    {
        gimbal_dart.motor_yaw.angle_p.set=50;
    }
    else if(gimbal_dart.motor_yaw.angle_p.set<=-51)
    {
        gimbal_dart.motor_yaw.angle_p.set=-51;
    }
    //gimbal_dart.motor_yaw.relative_angle_set=0.96f;
    gimbal_dart.motor_yaw.speed_p.set= pid_loop_calc(&gimbal_dart.motor_yaw.angle_p,
                                                  gimbal_dart.motor_yaw.angle_p.get,
                                                  gimbal_dart.motor_yaw.angle_p.set,
                                                  180,-180);

    first_order_filter_cali(&filter_yaw_in,gimbal_dart.motor_yaw.speed_p.set);

    gimbal_dart.motor_yaw.give_current= -pid_calc(&gimbal_dart.motor_yaw.speed_p,
                                                  gimbal_dart.motor_yaw.motor_measure->speed_rpm,
                                                  filter_yaw_in.out);

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
        if(direction==1)
        {
            thrust_motor.trigger_motor.angle_p.set=get_trigger_distance+5;
        }
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
    thrust_motor.trigger_motor.speed_p.set= pid_calc(&thrust_motor.trigger_motor.angle_p,
                                                     get_trigger_distance,
                                                     thrust_motor.trigger_motor.angle_p.set);
    thrust_motor.trigger_motor.give_current= pid_calc(&thrust_motor.trigger_motor.speed_p,
                                                      thrust_motor.trigger_motor.motor_measure->speed_rpm,
                                                      thrust_motor.trigger_motor.speed_p.set);
}

//差个扳机打开
static void dart_launch_handle()
{
    //确定打前哨还是基地并且接收视觉数据,接受完后调整当前数组里的值
    trigger_open();
    if(launcherable_num==1 && num_launched<2)
    {
        num_launched+=1;
        ready1_flag=0;
    }
    if(launcherable_num==2 && num_launched<4)
    {
        num_launched+=1;
        ready2_load_flag=0;
        ready2_goal_flag=0;
    }
    if(num_launched<4)
    {
        gimbal_dart.mode=DART_READY;
    }else
    {
        gimbal_dart.mode=DART_BACK;
    }

}

static void set_drive_distance()
{
    ready1_set_drive_right_distance=-(dart_length-slide_length-trigger_to_base_distance_set[num_launched]-10-get_drive_right_distance);
    ready1_set_drive_left_distance=(dart_length-slide_length-trigger_to_base_distance_set[num_launched]-10+get_drive_left_distance);
}

static void trigger_open()
{
    __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,500);
    //vTaskDelay(500);
}

static void trigger_off()
{
    __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,1600);
    //vTaskDelay(500);
}

static void dart_ready1()
{
    if (ready1_flag == 0) {
        set_drive_distance();
        ready1_flag = 1;
    }
    if(flags.is_ready_ok==true)
    {
        launcher_dart.push_motor_r.speed_p.set = pid_calc(&launcher_dart.push_motor_r.angle_p,
                                                          get_drive_right_distance,
                                                          ready1_back_drive_right_distance);
        launcher_dart.push_motor_r.give_current = pid_calc(&launcher_dart.push_motor_r.speed_p,
                                                           launcher_dart.push_motor_r.motor_measure->speed_rpm,
                                                           launcher_dart.push_motor_r.speed_p.set);
        launcher_dart.push_motor_l.speed_p.set = pid_calc(&launcher_dart.push_motor_r.angle_p,
                                                          get_drive_left_distance,
                                                          ready1_back_drive_left_distance);
        launcher_dart.push_motor_l.give_current = pid_calc(&launcher_dart.push_motor_r.speed_p,
                                                           launcher_dart.push_motor_l.motor_measure->speed_rpm,
                                                           launcher_dart.push_motor_l.speed_p.set);
    }else
    {
        if (dart_goal == GOAL_FRONT_STATION) {
            thrust_motor.trigger_motor.speed_p.set = pid_calc(&thrust_motor.trigger_motor.angle_p,
                                                              get_trigger_distance,
                                                              trigger_to_outposts_distance_set[num_launched]);
            thrust_motor.trigger_motor.give_current = pid_calc(&thrust_motor.trigger_motor.speed_p,
                                                               thrust_motor.trigger_motor.motor_measure->speed_rpm,
                                                               thrust_motor.trigger_motor.speed_p.set);
            if (fabs(get_trigger_distance - trigger_to_outposts_distance_set[num_launched]) < 1) {
                flags.is_ready1_trigger_move_ok = true;
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
            launcher_dart.push_motor_r.speed_p.set = pid_calc(&launcher_dart.push_motor_r.angle_p,
                                                              get_drive_right_distance,
                                                              ready1_set_drive_right_distance);
            launcher_dart.push_motor_r.give_current = pid_calc(&launcher_dart.push_motor_r.speed_p,
                                                               launcher_dart.push_motor_r.motor_measure->speed_rpm,
                                                               launcher_dart.push_motor_r.speed_p.set);
            launcher_dart.push_motor_l.speed_p.set = pid_calc(&launcher_dart.push_motor_r.angle_p,
                                                              get_drive_left_distance,
                                                              ready1_set_drive_left_distance);
            launcher_dart.push_motor_l.give_current = pid_calc(&launcher_dart.push_motor_r.speed_p,
                                                               launcher_dart.push_motor_l.motor_measure->speed_rpm,
                                                               launcher_dart.push_motor_l.speed_p.set);
            if (fabs(get_drive_right_distance - ready1_set_drive_right_distance) < 20 &&
                fabs(get_drive_left_distance - ready1_set_drive_left_distance) < 20) {
                flags.is_ready1_drive_ok = true;
            }
        }
        if(flags.is_ready1_drive_ok==true)
        {
            if(flags.is_ready1_trigger_off_ok==false)
            {
                trigger_off();
                if(rc_ctrl.rc.ch[0]<-600&&rc_ctrl.rc.ch[1]<-600&&rc_ctrl.rc.ch[2]>600&&rc_ctrl.rc.ch[3]<-600)
                {
                    flags.is_ready1_trigger_off_ok=true;
                }
            }
        }
        if (flags.is_ready1_trigger_off_ok == true) {
            launcher_dart.push_motor_r.speed_p.set = pid_calc(&launcher_dart.push_motor_r.angle_p,
                                                              get_drive_right_distance,
                                                              ready1_back_drive_right_distance);
            launcher_dart.push_motor_r.give_current = pid_calc(&launcher_dart.push_motor_r.speed_p,
                                                               launcher_dart.push_motor_r.motor_measure->speed_rpm,
                                                               launcher_dart.push_motor_r.speed_p.set);
            launcher_dart.push_motor_l.speed_p.set = pid_calc(&launcher_dart.push_motor_r.angle_p,
                                                              get_drive_left_distance,
                                                              ready1_back_drive_left_distance);
            launcher_dart.push_motor_l.give_current = pid_calc(&launcher_dart.push_motor_r.speed_p,
                                                               launcher_dart.push_motor_l.motor_measure->speed_rpm,
                                                               launcher_dart.push_motor_l.speed_p.set);
            if (fabs(get_drive_right_distance - ready1_back_drive_right_distance) < 5 &&
                fabs(get_drive_left_distance - ready1_back_drive_left_distance) < 5) {
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
        }
    }
}

static void set_load_drive_distance()
{
    ready2_load_set_drive_left_distance=384+init_drive_left_distance;
    ready2_load_set_drive_right_distance=-392+init_drive_right_distance;
}

static void set_goal_drive_distance()
{
    ready2_goal_set_drive_right_distance=-(dart_length-slide_length-trigger_to_base_distance_set[num_launched]-25)+back_drive_right_distance;
    ready2_goal_set_drive_left_distance=(dart_length-slide_length-trigger_to_base_distance_set[num_launched]-25)+back_drive_left_distance;
}

static void dart_ready2()
{
    //扳机打开
    if(ready2_load_flag==0)
    {
        set_load_drive_distance();
        ready2_load_flag=1;
    }
    if(flags.is_ready2_ok==true)
    {
        launcher_dart.push_motor_r.speed_p.set = pid_calc(&launcher_dart.push_motor_r.angle_p,
                                                          get_drive_right_distance,
                                                          back_drive_right_distance);
        launcher_dart.push_motor_r.give_current = pid_calc(&launcher_dart.push_motor_r.speed_p,
                                                           launcher_dart.push_motor_r.motor_measure->speed_rpm,
                                                           launcher_dart.push_motor_r.speed_p.set);

        launcher_dart.push_motor_l.speed_p.set = pid_calc(&launcher_dart.push_motor_r.angle_p,
                                                          get_drive_left_distance,
                                                          back_drive_left_distance);
        launcher_dart.push_motor_l.give_current = pid_calc(&launcher_dart.push_motor_r.speed_p,
                                                           launcher_dart.push_motor_l.motor_measure->speed_rpm,
                                                           launcher_dart.push_motor_l.speed_p.set);
        thrust_motor.thrust_angle_motor.speed_p.set = pid_loop_calc(&thrust_motor.thrust_angle_motor.angle_p,
                                                                    get_angle_thrust_angle,
                                                                    back_thrust_angle_angle,
                                                                    180, -180);
        thrust_motor.thrust_angle_motor.give_current = -pid_calc(&thrust_motor.thrust_angle_motor.speed_p,
                                                                thrust_motor.thrust_angle_motor.motor_measure->speed_rpm,
                                                                thrust_motor.thrust_angle_motor.speed_p.set);

        thrust_motor.thrust_move_motor.speed_p.set = pid_calc(&thrust_motor.thrust_move_motor.angle_p,
                                                              get_thrust_move_distance,
                                                              back_thrust_move_distance);
        thrust_motor.thrust_move_motor.give_current = pid_calc(&thrust_motor.thrust_move_motor.speed_p,
                                                               thrust_motor.thrust_move_motor.motor_measure->speed_rpm,
                                                               thrust_motor.thrust_move_motor.speed_p.set);
    }else
    if(flags.is_ready2_ok==false) {
        if(flags.is_ready2_trigger_open_ok==false)
        {
            trigger_open();
            flags.is_ready2_trigger_open_ok=true;
        }
        if(flags.is_ready2_turn_angle_ok_init==false && flags.is_ready2_turn_init_ok==false&&flags.is_ready2_trigger_open_ok==true)
        {
            launcher_dart.turn_motor.angle_p.set=turn_motor_angle_set[turn_angle];
        launcher_dart.turn_motor.speed_p.set = pid_loop_calc(&launcher_dart.turn_motor.angle_p,
                                                             launcher_dart.turn_motor.angle_p.get,
                                                             launcher_dart.turn_motor.angle_p.set,
                                                             180, -180);
        launcher_dart.turn_motor.give_current = -pid_calc(&launcher_dart.turn_motor.speed_p,
                                                         launcher_dart.turn_motor.motor_measure->speed_rpm,
                                                         launcher_dart.turn_motor.speed_p.set);

        if (fabs(launcher_dart.turn_motor.angle_p.get - launcher_dart.turn_motor.angle_p.set) < 0.5) {
            flags.is_ready2_turn_init_ok = true;
            flags.is_ready2_turn_angle_ok_init=true;
            turn_angle = (turn_angle + 1) % 6;
        }
        }
        if (flags.is_ready2_turn_init_ok == true) {
            launcher_dart.push_motor_r.speed_p.set = pid_calc(&launcher_dart.push_motor_r.angle_p,
                                                              get_drive_right_distance,
                                                              ready2_load_set_drive_right_distance);
            launcher_dart.push_motor_r.give_current = pid_calc(&launcher_dart.push_motor_r.speed_p,
                                                               launcher_dart.push_motor_r.motor_measure->speed_rpm,
                                                               launcher_dart.push_motor_r.speed_p.set);

            launcher_dart.push_motor_l.speed_p.set = pid_calc(&launcher_dart.push_motor_r.angle_p,
                                                              get_drive_left_distance,
                                                              ready2_load_set_drive_left_distance);
            launcher_dart.push_motor_l.give_current = pid_calc(&launcher_dart.push_motor_r.speed_p,
                                                               launcher_dart.push_motor_l.motor_measure->speed_rpm,
                                                               launcher_dart.push_motor_l.speed_p.set);
            if (fabs(get_drive_right_distance - ready2_load_set_drive_right_distance) < 15 &&
                fabs(get_drive_left_distance - ready2_load_set_drive_left_distance) < 15) {
                flags.is_ready2_drive_load_ok = true;
            }
        }

        if (flags.is_ready2_drive_load_ok == true) {
            if(flags.is_ready2_turn_angle_ok_load==false) {
                launcher_dart.turn_motor.angle_p.set = turn_motor_angle_set[turn_angle];
                launcher_dart.turn_motor.speed_p.set = pid_loop_calc(&launcher_dart.turn_motor.angle_p,
                                                                     launcher_dart.turn_motor.angle_p.get,
                                                                     launcher_dart.turn_motor.angle_p.set,
                                                                     180, -180);
                launcher_dart.turn_motor.give_current = -pid_calc(&launcher_dart.turn_motor.speed_p,
                                                                  launcher_dart.turn_motor.motor_measure->speed_rpm,
                                                                  launcher_dart.turn_motor.speed_p.set);
                if (fabs(launcher_dart.turn_motor.angle_p.get - launcher_dart.turn_motor.angle_p.set) < 0.5) {
                    flags.is_ready2_turn_load_ok = true;
                    flags.is_ready2_turn_angle_ok_load = true;
                    turn_angle = (turn_angle + 1) % 6;
                }
            }
        }

        if (flags.is_ready2_turn_load_ok == true) {
            thrust_motor.thrust_angle_motor.speed_p.set = pid_loop_calc(&thrust_motor.thrust_angle_motor.angle_p,
                                                                        get_angle_thrust_angle,
                                                                        ready2_thrust_angle_goal_angle,
                                                                        180, -180);
            thrust_motor.thrust_angle_motor.give_current = -pid_calc(&thrust_motor.thrust_angle_motor.speed_p,
                                                                    thrust_motor.thrust_angle_motor.motor_measure->speed_rpm,
                                                                    thrust_motor.thrust_angle_motor.speed_p.set);
            if (fabs(thrust_motor.thrust_angle_motor.angle_p.get - ready2_thrust_angle_goal_angle) < 0.5) {
                flags.is_ready2_thrust_angle_goal_ok = true;
            }
        }

        if (flags.is_ready2_thrust_angle_goal_ok == true) {
            thrust_motor.thrust_move_motor.speed_p.set = pid_calc(&thrust_motor.thrust_move_motor.angle_p,
                                                                  get_thrust_move_distance,
                                                                  ready2_thrust_move_goal_distance);
            thrust_motor.thrust_move_motor.give_current = pid_calc(&thrust_motor.thrust_move_motor.speed_p,
                                                                   thrust_motor.thrust_move_motor.motor_measure->speed_rpm,
                                                                   thrust_motor.thrust_move_motor.speed_p.set);
            if (fabs(thrust_motor.thrust_move_motor.angle_p.get - ready2_thrust_move_goal_distance) < 10) {
                flags.is_ready2_thrust_move_goal_ok = true;
            }
        }

        if (flags.is_ready2_thrust_move_goal_ok == true) {
            thrust_motor.thrust_move_motor.speed_p.set = pid_calc(&thrust_motor.thrust_move_motor.angle_p,
                                                                  get_thrust_move_distance,
                                                                  back_thrust_move_distance);
            thrust_motor.thrust_move_motor.give_current = pid_calc(&thrust_motor.thrust_move_motor.speed_p,
                                                                   thrust_motor.thrust_move_motor.motor_measure->speed_rpm,
                                                                   thrust_motor.thrust_move_motor.speed_p.set);

            if (fabs(thrust_motor.thrust_move_motor.angle_p.get - back_thrust_move_distance) < 10) {
                flags.is_ready2_thrust_move_back_ok = true;
            }
        }

        if (flags.is_ready2_thrust_move_back_ok == true) {
            thrust_motor.thrust_angle_motor.speed_p.set = pid_loop_calc(&thrust_motor.thrust_angle_motor.angle_p,
                                                                        get_angle_thrust_angle,
                                                                        back_thrust_angle_angle,
                                                                        180, -180);
            thrust_motor.thrust_angle_motor.give_current = -pid_calc(&thrust_motor.thrust_angle_motor.speed_p,
                                                                    thrust_motor.thrust_angle_motor.motor_measure->speed_rpm,
                                                                    thrust_motor.thrust_angle_motor.speed_p.set);

            if (fabs(thrust_motor.thrust_angle_motor.angle_p.get - back_thrust_angle_angle) < 1) {
                flags.is_ready2_thrust_angle_back_ok = true;
            }
        }

        if (flags.is_ready2_thrust_angle_back_ok == true) {
            if(flags.is_ready2_turn_angle_ok_end==false) {
                launcher_dart.turn_motor.angle_p.set = turn_motor_angle_set[turn_angle];
                launcher_dart.turn_motor.speed_p.set = pid_loop_calc(&launcher_dart.turn_motor.angle_p,
                                                                     launcher_dart.turn_motor.angle_p.get,
                                                                     launcher_dart.turn_motor.angle_p.set, 180, -180);
                launcher_dart.turn_motor.give_current = -pid_calc(&launcher_dart.turn_motor.speed_p,
                                                                  launcher_dart.turn_motor.motor_measure->speed_rpm,
                                                                  launcher_dart.turn_motor.speed_p.set);

                if (fabs(launcher_dart.turn_motor.angle_p.get - launcher_dart.turn_motor.angle_p.set) < 0.5) {
                    flags.is_ready2_turn_end_ok = true;
                    flags.is_ready2_turn_angle_ok_end = true;
                }
            }
        }

        if (flags.is_ready2_turn_end_ok == true) {
            if (ready2_set_flag == 0) {
                ready2_goal_flag = 1;
                set_goal_drive_distance();
            }
            launcher_dart.push_motor_r.speed_p.set = pid_calc(&launcher_dart.push_motor_r.angle_p,
                                                              get_drive_right_distance,
                                                              ready2_goal_set_drive_right_distance);
            launcher_dart.push_motor_r.give_current = pid_calc(&launcher_dart.push_motor_r.speed_p,
                                                               launcher_dart.push_motor_r.motor_measure->speed_rpm,
                                                               launcher_dart.push_motor_r.speed_p.set);

            launcher_dart.push_motor_l.speed_p.set = pid_calc(&launcher_dart.push_motor_r.angle_p,
                                                              get_drive_left_distance,
                                                              ready2_goal_set_drive_left_distance);
            launcher_dart.push_motor_l.give_current = pid_calc(&launcher_dart.push_motor_l.speed_p,
                                                               launcher_dart.push_motor_l.motor_measure->speed_rpm,
                                                               launcher_dart.push_motor_l.speed_p.set);
            if (fabs(get_drive_right_distance - ready2_goal_set_drive_right_distance) < 15 &&
                fabs(get_drive_left_distance - ready2_goal_set_drive_left_distance) < 15) {
                flags.is_ready2_drive_goal_ok = true;
            }
        }
        //扳机闭合
        if(flags.is_ready2_drive_goal_ok==true)
        {
            trigger_off();
            if(rc_ctrl.rc.ch[0]<-600&&rc_ctrl.rc.ch[1]<-600&&rc_ctrl.rc.ch[2]>600&&rc_ctrl.rc.ch[3]<-600)
            {
                flags.is_ready2_trigger_off_ok=true;
            }
        }
        if (flags.is_ready2_trigger_off_ok == true) {
            launcher_dart.push_motor_r.speed_p.set = pid_calc(&launcher_dart.push_motor_r.angle_p,
                                                              get_drive_right_distance,
                                                              back_drive_right_distance);
            launcher_dart.push_motor_r.give_current = pid_calc(&launcher_dart.push_motor_r.speed_p,
                                                               launcher_dart.push_motor_r.motor_measure->speed_rpm,
                                                               launcher_dart.push_motor_r.speed_p.set);

            launcher_dart.push_motor_l.speed_p.set = pid_calc(&launcher_dart.push_motor_r.angle_p,
                                                              get_drive_left_distance,
                                                              back_drive_left_distance);
            launcher_dart.push_motor_l.give_current = pid_calc(&launcher_dart.push_motor_r.speed_p,
                                                               launcher_dart.push_motor_l.motor_measure->speed_rpm,
                                                               launcher_dart.push_motor_l.speed_p.set);
            if (fabs(get_drive_right_distance - back_drive_right_distance) < 10 &&
                fabs(get_drive_left_distance - back_drive_left_distance) < 10) {
                flags.is_ready2_drive_back_ok = true;
            }

        }
        if (flags.is_ready2_drive_back_ok == true) {
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
        }
    }

}

static void dart_ready_handle()
{
    vTaskDelay(10);
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
        gimbal_dart.motor_yaw.angle_p.set=yaw_angle_goal[front];
        thrust_motor.trigger_motor.speed= pid_calc(&thrust_motor.trigger_motor.angle_p,get_trigger_distance,trigger_to_outposts_distance_set[num_launched]);
        thrust_motor.trigger_motor.give_current= pid_calc(&thrust_motor.trigger_motor.speed_p,thrust_motor.trigger_motor.motor_measure->speed_rpm,thrust_motor.trigger_motor.speed);

        gimbal_dart.motor_yaw.speed_p.set= pid_loop_calc(&gimbal_dart.motor_yaw.angle_p,
                                                         gimbal_dart.motor_yaw.angle_p.get,
                                                         gimbal_dart.motor_yaw.angle_p.set,
                                                         180,-180);

        gimbal_dart.motor_yaw.give_current= -pid_calc(&gimbal_dart.motor_yaw.speed_p,
                                                     gimbal_dart.motor_yaw.motor_measure->speed_rpm,
                                                     gimbal_dart.motor_yaw.speed_p.set);
    }else
    {
        gimbal_dart.motor_yaw.angle_p.set=yaw_angle_goal[base];
        thrust_motor.trigger_motor.speed= pid_calc(&thrust_motor.trigger_motor.angle_p,get_trigger_distance,trigger_to_base_distance_set[num_launched]);
        thrust_motor.trigger_motor.give_current= pid_calc(&thrust_motor.trigger_motor.speed_p,thrust_motor.trigger_motor.motor_measure->speed_rpm,thrust_motor.trigger_motor.speed);

        gimbal_dart.motor_yaw.speed_p.set= pid_loop_calc(&gimbal_dart.motor_yaw.angle_p,
                                                         gimbal_dart.motor_yaw.angle_p.get,
                                                         gimbal_dart.motor_yaw.angle_p.set,
                                                         180,-180);

        gimbal_dart.motor_yaw.give_current= -pid_calc(&gimbal_dart.motor_yaw.speed_p,
                                                     gimbal_dart.motor_yaw.motor_measure->speed_rpm,
                                                     gimbal_dart.motor_yaw.speed_p.set);
    }

}

//已完成
static void dart_control_handle()
{
    yaw_control();
    //turn_control();
    drive_control();
    trigger_control();
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
    if(rc_ctrl.rc.ch[4]<20&&rc_ctrl.rc.ch[4]>-20)
    {
        rc_ctrl.rc.ch[4]=0;
    }
    set_thrust_move_distance += rc_ctrl.rc.ch[4]*0.0005;
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
    set_angle_thrust_angle-=rc_ctrl.rc.ch[4]*0.001;
    if(set_angle_thrust_angle<4.59)
    {
        set_angle_thrust_angle = 4.59f;
    }
    if(set_angle_thrust_angle > 60.02)
    {
        set_angle_thrust_angle =60.02f;
    }
    //get_angle_thrust_angle= ecd_to_angle(thrust_motor.thrust_angle_motor.motor_measure->total_ecd);
    thrust_motor.thrust_angle_motor.speed_p.set= pid_loop_calc(&thrust_motor.thrust_angle_motor.angle_p,
                                                               get_angle_thrust_angle,
                                                               set_angle_thrust_angle,
                                                               180,-180);
    thrust_motor.thrust_angle_motor.give_current= -pid_calc(&thrust_motor.thrust_angle_motor.speed_p,
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
//    get_drive_right_distance= drive_distance_conversion(launcher_dart.push_motor_r.motor_measure->total_ecd);
//    get_drive_left_distance= drive_distance_conversion(launcher_dart.push_motor_l.motor_measure->total_ecd);
//    if(rc_ctrl.rc.ch[3]<=20&&rc_ctrl.rc.ch[3]>=-20)
//    {
//        rc_ctrl.rc.ch[3]=0;
//    }
//    set_drive_right_distance += rc_ctrl.rc.ch[3]*0.001;
//    set_drive_left_distance -= rc_ctrl.rc.ch[3]*0.001;
//    if(set_drive_right_distance >= drive_distance_conversion(init_ecd_drive_right) - 10)
//    {
//        set_drive_right_distance = drive_distance_conversion(init_ecd_drive_right) - 10;
//    }
//    if(set_drive_left_distance <= drive_distance_conversion(init_ecd_drive_left)- 10)
//    {
//        set_drive_left_distance = drive_distance_conversion(init_ecd_drive_left)- 10;
//    }
//    launcher_dart.push_motor_r.speed_p.set= pid_calc(&launcher_dart.push_motor_r.angle_p,
//                                                     get_drive_right_distance,
//                                                     set_drive_right_distance);
//    launcher_dart.push_motor_l.speed_p.set= pid_calc(&launcher_dart.push_motor_r.angle_p,
//                                                     get_drive_left_distance,
//                                                     set_drive_left_distance);
//    launcher_dart.push_motor_r.give_current= pid_calc(&launcher_dart.push_motor_r.speed_p,
//                                                      launcher_dart.push_motor_r.motor_measure->speed_rpm,
//                                                      launcher_dart.push_motor_r.speed_p.set);
//    launcher_dart.push_motor_l.give_current= pid_calc(&launcher_dart.push_motor_r.speed_p,
//                                                      launcher_dart.push_motor_l.motor_measure->speed_rpm,
//                                                      launcher_dart.push_motor_l.speed_p.set);
    launcher_dart.push_motor_r.give_current=rc_ctrl.rc.ch[3]*25;
    launcher_dart.push_motor_l.give_current=-rc_ctrl.rc.ch[3]*25;
    if(launcher_dart.push_motor_r.give_current>16000)
    {
        launcher_dart.push_motor_r.give_current=16000;
    }
    if(launcher_dart.push_motor_l.give_current<-16000)
    {
        launcher_dart.push_motor_l.give_current=-16000;
    }
}

static void turn_control()
{
    //launcher_dart.turn_motor.angle_p.get-=rc_ctrl.rc.ch[2]*0.01*0.03f;
    //launcher_dart.turn_motor.angle_p.get= -motor_ecd_to_angle_change(launcher_dart.turn_motor.motor_measure->ecd,launcher_dart.turn_motor.motor_measure->offset_ecd);
    //launcher_dart.turn_motor.angle_p.set-=rc_ctrl.rc.ch[2]*0.01*0.03f;
//    launcher_dart.turn_motor.angle_p.set=69.7850f;
//    launcher_dart.turn_motor.angle_p.set=turn_motor_angle_set[turn_angle];
    if(launcher_dart.turn_motor.angle_p.get>180)
    {
        launcher_dart.turn_motor.angle_p.get=launcher_dart.turn_motor.angle_p.get-360;
    }
    else if(launcher_dart.turn_motor.angle_p.get<-180)
    {
        launcher_dart.turn_motor.angle_p.get=launcher_dart.turn_motor.angle_p.get+360;
    }
    launcher_dart.turn_motor.gyro_set= pid_loop1_calc(&launcher_dart.turn_motor.angle_p,
                                                      launcher_dart.turn_motor.angle_p.get,
                                                      launcher_dart.turn_motor.angle_p.set,
                                                      180,-180);
    launcher_dart.turn_motor.give_current= -pid_calc(&launcher_dart.turn_motor.speed_p,
                                                     launcher_dart.turn_motor.motor_measure->speed_rpm,
                                                    launcher_dart.turn_motor.gyro_set);
//    if((fabs(launcher_dart.turn_motor.angle_p.set-launcher_dart.turn_motor.angle_p.get)<1))
//    {
//        vTaskDelay(5000);
//        turn_angle+=1;
//        turn_angle%=6;
//        time_now=HAL_GetTick();
//        flag1=true;
//    }
}

static void yaw_control()
{
    gimbal_dart.motor_yaw.angle_p.set-=rc_ctrl.rc.ch[0]*0.01*0.03f;
    if(gimbal_dart.motor_yaw.angle_p.set>=45)
    {
        gimbal_dart.motor_yaw.angle_p.set=45;
    }
    else if(gimbal_dart.motor_yaw.angle_p.set<=-83)
    {
        gimbal_dart.motor_yaw.angle_p.set=-83;
    }
    //gimbal_dart.motor_yaw.relative_angle_set=0.96f;
    gimbal_dart.motor_yaw.speed_p.set= pid_loop_calc(&gimbal_dart.motor_yaw.angle_p,
                                                  gimbal_dart.motor_yaw.angle_p.get,
                                                  gimbal_dart.motor_yaw.angle_p.set,
                                                  180,-180);

    //first_order_filter_cali(&filter_yaw_in,gimbal_dart.motor_yaw.speed_p.set);

    gimbal_dart.motor_yaw.give_current= -pid_calc(&gimbal_dart.motor_yaw.speed_p,
                                                  gimbal_dart.motor_yaw.motor_measure->speed_rpm,
                                                  gimbal_dart.motor_yaw.speed_p.set);
}

//已完成
static void dart_back_handle()
{

    thrust_motor.trigger_motor.speed_p.set= pid_calc(&thrust_motor.trigger_motor.angle_p,
                                                     get_trigger_distance,
                                                     back_trigger_distance);
    thrust_motor.trigger_motor.give_current= pid_calc(&thrust_motor.trigger_motor.speed_p,
                                                      thrust_motor.trigger_motor.motor_measure->speed_rpm,
                                                      thrust_motor.trigger_motor.speed_p.set);
    if(fabs(back_trigger_distance-get_trigger_distance)<1)
    {
        flags.is_trigger_move_ok=true;
    }
    if(flags.is_trigger_move_ok==true)
    {
        launcher_dart.push_motor_l.speed_p.set= pid_calc(&launcher_dart.push_motor_r.angle_p,
                                                         get_drive_left_distance,
                                                         back_drive_left_distance);
        launcher_dart.push_motor_l.give_current= pid_calc(&launcher_dart.push_motor_l.speed_p,
                                                          launcher_dart.push_motor_l.motor_measure->speed_rpm,
                                                          launcher_dart.push_motor_l.speed_p.set);
        launcher_dart.push_motor_r.speed_p.set= pid_calc(&launcher_dart.push_motor_r.angle_p,
                                                         get_drive_right_distance,
                                                         back_drive_right_distance);
        launcher_dart.push_motor_r.give_current= pid_calc(&launcher_dart.push_motor_r.speed_p,
                                                          launcher_dart.push_motor_r.motor_measure->speed_rpm,
                                                          launcher_dart.push_motor_r.speed_p.set);
        if(fabs(get_drive_right_distance-back_drive_right_distance)<1)
        {
            flags.is_back_drive_ok=true;
        }
    }
    if(flags.is_back_drive_ok==true)
    {
        launcher_dart.turn_motor.speed_p.set= pid_loop_calc(&launcher_dart.turn_motor.angle_p,
                                                            launcher_dart.turn_motor.angle_p.get,
                                                            turn_motor_angle_set[turn_angle],
                                                            180,-180);
        launcher_dart.turn_motor.give_current= -pid_calc(&launcher_dart.turn_motor.speed_p,
                                                        launcher_dart.turn_motor.motor_measure->speed_rpm,
                                                        launcher_dart.turn_motor.speed_p.set);
        if(fabs(launcher_dart.turn_motor.angle_p.get-turn_motor_angle_set[turn_angle])<0.5)
        {
            flags.is_back_turn_ok=true;
        }
    }
    if(flags.is_back_turn_ok==true)
    {
        thrust_motor.thrust_move_motor.speed_p.set= pid_calc(&thrust_motor.thrust_move_motor.angle_p,
                                                             get_thrust_move_distance,
                                                             back_thrust_move_distance);
        thrust_motor.thrust_move_motor.give_current= pid_calc(&thrust_motor.thrust_move_motor.speed_p,
                                                              thrust_motor.thrust_move_motor.motor_measure->speed_rpm,
                                                              thrust_motor.thrust_move_motor.speed_p.set);
        if(fabs(back_thrust_move_distance-get_thrust_move_distance)<1)
        {
            flags.is_thrust_move_ok=true;
        }
    }
    if(flags.is_thrust_move_ok==true)
    {
        thrust_motor.thrust_angle_motor.speed_p.set= pid_loop_calc(&thrust_motor.thrust_angle_motor.angle_p,
                                                                   get_angle_thrust_angle,
                                                                   back_thrust_angle_angle,
                                                                   180,-180);
        thrust_motor.thrust_angle_motor.give_current= -pid_calc(&thrust_motor.thrust_angle_motor.speed_p,
                                                               thrust_motor.thrust_angle_motor.motor_measure->speed_rpm,
                                                               thrust_motor.thrust_angle_motor.speed_p.set);
        if(fabs(back_thrust_angle_angle-get_angle_thrust_angle)<1)
        {
            flags.is_back_ok=true;
        }
    }
    if(flags.is_back_ok==true)
    {
        flags.is_back_drive_ok=false;
        flags.is_trigger_move_ok=false;
        flags.is_thrust_angle_ok=false;
        flags.is_thrust_move_ok=false;
        flags.is_back_ok=false;
        flags.is_back_turn_ok=false;
    }
}

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

    gimbal_dart.motor_yaw.motor_measure->offset_ecd=4841;
    launcher_dart.turn_motor.motor_measure->offset_ecd=5459;

    turn_motor_angle_set[0]=179.912f;
    turn_motor_angle_set[1]=122.0800f;
    turn_motor_angle_set[2]=59.633f;
    turn_motor_angle_set[3]=-0.922f;
    turn_motor_angle_set[4]=-63.0615f;
    turn_motor_angle_set[5]=-116.5429f;

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
        //num_launched==0记得加回来
        if(switch_is_up(rc_ctrl.rc.s[RC_s_L])&&flags.is_ready_ok==1&&num_launched==0)
        {
            gimbal_dart.last_mode=gimbal_dart.mode;
            gimbal_dart.mode=DART_LAUNCH;
            flags.is_ready_ok=0;
        }else
        if(num_launched>0&&!switch_is_mid(rc_ctrl.rc.s[RC_s_R])&&flags.is_ready2_ok==1)
        {
            gimbal_dart.last_mode=gimbal_dart.mode;
            gimbal_dart.mode=DART_TRIGGER;
            flags.is_ready2_ok=0;
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
    //gimbal_dart.motor_yaw.absolute_angle_get=INS_angle[0]*MOTOR_RAD_TO_ANGLE;
    gimbal_dart.motor_yaw.angle_p.get= -motor_ecd_to_angle_change(gimbal_dart.motor_yaw.motor_measure->ecd,gimbal_dart.motor_yaw.motor_measure->offset_ecd);
    launcher_dart.turn_motor.angle_p.get= -motor_ecd_to_angle_change(launcher_dart.turn_motor.motor_measure->ecd,launcher_dart.turn_motor.motor_measure->offset_ecd);
    thrust_motor.thrust_angle_motor.angle_p.get=ecd_to_angle(thrust_motor.thrust_angle_motor.motor_measure->total_ecd);
    get_drive_right_distance= drive_distance_conversion(launcher_dart.push_motor_r.motor_measure->total_ecd);
    get_drive_left_distance= drive_distance_conversion(launcher_dart.push_motor_l.motor_measure->total_ecd);
    get_trigger_distance=trigger_distance_conversion(thrust_motor.trigger_motor.motor_measure->total_ecd);
    get_thrust_move_distance= thrust_move_distance_conversion(thrust_motor.thrust_move_motor.motor_measure->total_ecd);
    get_angle_thrust_angle= ecd_to_angle(thrust_motor.thrust_angle_motor.motor_measure->total_ecd);
}