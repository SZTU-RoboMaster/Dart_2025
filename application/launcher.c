/*轮子电机id：                            四个摩擦轮                     解算坐标：      x(前)
 *                                           前
                ****         ****                     ****               ****                              |
                          FL                                     BL                          |
                ****         ****                     ****               ****                              |
                                                                                                                |
    左                                                                                         右
                                                                                                                |
                ****              ****                     ****               ****                              |
                           BR                                       FR                 |
                ****              ****                     ****               ****
                                                 后
    */

#include "launcher.h"
#include "Gimbal.h"
#include "can_receive.h"
#include "Chassis.h"
#include "cmsis_os.h"
#include "remote.h"
#include"bsp_laser.h"
#include"gpio.h"
#include "key_board.h"
#include"stdlib.h"
#include "referee.h"
#include "bsp_led.h"
#define frition_rate  0.00066139
#define frition_speed 1
#define M2006_POSITION_TO_ECD (36 * 8192)
#define M2006_ECD_TO_POSITION  (1/M2006_POSITION_TO_ECD)

uint8_t rc_last_sw_L;

uint32_t continue_shoot_time;//遥控器左边拨杆down的持续时间 或者 鼠标左键按下的持续时间

uint8_t trigger_flag=0;
int64_t turn_start_time;
extern Key_board_t KeyBoard;
extern RC_ctrl_t rc_ctrl;
extern gimbal_t gimbal;
extern motor_measure_t motor_2006_measure[3];
extern chassis_t chassis;
extern motor_measure_t motor_3508_dart[4];
extern motor_measure_t motor_turn_measure;
//int v1=2500,v2=2700;
//通过赋值进行发射机构的初始化
launcher_t launcher;
fp32 set_position;
float goal__ecd;//设定推弹电机的ecd值
float get__position;//当前推弹电机的ecd值
float up_speed_out;


float turn[4];//数组4个容量对应4个换弹电机角度
int turn_i;   //当前设定角度的索引
int launcher_speed[4][4] = {{3000, -3000, -5000, 5000}, //东风1号
                            {3000, -3000, -5000, 5000}, //东风2号
                            {3000, -3000, -5000, 5000}, //东风3号
                            {3000, -3000, -5000, 5000}}; //东风4号

int32_t pushbeam_state=0;
int32_t init_ecd=0;
#define bool	_Bool
#define true	1
#define false	0

uint8_t blocked_flag;
uint8_t reverse_flag;
uint32_t continue_shoot_time;//遥控器左边拨杆down的持续时间 或者 鼠标左键按下的持续时间


uint32_t begin_count=1,reset_time;//一个flag标志位       //一个记录上电时的时刻
uint32_t reverse_start_time;
uint32_t blocked_start_time;
int32_t shoot_2=0;              //用于判断飞镖是否将要发射，1为即将发射

uint8_t dart_door_status,dart_door_last_status;
//#define TRIGGER_CONTINUES_SPEED -4000


#define TRIGGER_REVERSE_SPEED 3000       //推弹电机堵转时的反转速度
#define CONTINUES_BLOCKED_JUDGE() (HAL_GetTick()-blocked_start_time>500)
#define TRIGGER_REVERSE_TIME_JUDGE() (HAL_GetTick()-reverse_start_time<100)
bool is_blocked(){
    if(TRIGGER_CONTINUES_SPEED>0){


        if (blocked_flag==false && launcher.trigger.motor_measure->speed_rpm <= 0.3*TRIGGER_CONTINUES_SPEED){
            blocked_start_time=HAL_GetTick();//获取堵转开始时间
            blocked_flag=true;
        }


        //标识位为1时，已经开始堵转，判断是否堵转达到一定时间，若达到，则判定堵转
        if (blocked_flag==true && launcher.trigger.motor_measure->speed_rpm <= 0.3*TRIGGER_CONTINUES_SPEED){
            if(CONTINUES_BLOCKED_JUDGE()){
                blocked_flag=false;
                return true;
            }
        }
        return false;
    }

    else if(TRIGGER_CONTINUES_SPEED<0){


        //在标识位为0时，电机转速低于阈值时，判定堵转开始
        if (blocked_flag==false && abs(chassis.motor_chassis[RF].motor_measure->speed_rpm- up_speed_out)>1000){
            blocked_start_time=HAL_GetTick();//获取堵转开始时间
            blocked_flag=true;
        }

        //标识位为1时，已经开始堵转，判断是否堵转达到一定时间，若达到，则判定堵转
        if (blocked_flag==true && abs(chassis.motor_chassis[RF].motor_measure->speed_rpm -up_speed_out)>1000){
            if(CONTINUES_BLOCKED_JUDGE()){
                blocked_flag=false;
                return true;
            }
        }
        else {
            blocked_flag=false;
        }
        return false;
    }
}
//当推弹电机堵转，推不动时处理函数
void  trigger_block_handle(){
    //判断堵转并且反转标识为0时
    if((is_blocked() && reverse_flag==false)){
        reverse_flag=true;//判定开始反转
        reverse_start_time=HAL_GetTick();//获取开始反转时间
    }

    //判定反转开始并且时间没有达到反转结束时间
    if(reverse_flag==true && TRIGGER_REVERSE_TIME_JUDGE()){
        chassis.motor_chassis[RF].give_current=TRIGGER_REVERSE_SPEED;//拨单电机设置为反转速度
    }

    else{
        reverse_flag=false;
    }
}
//处理推弹电机状态改变
void pushbeam_handle()
{

    switch (pushbeam_state)
    {
        case 0:
            goal__ecd=init_ecd;
            break;
        case 1:
            goal__ecd=init_ecd+185000;//调试过，退弹电机从初始位置到推动飞镖位置，电机转过的ecd值为185000左右
            break;
    }
}
//发射一枚飞镖完整逻辑
//包括换弹，推弹电机推弹，推弹电机复位
//相当于在线程中开线程
void single_shoot_handle()
{
    turn_i++;
    if(turn_i>=4)turn_i=0;
    turn_start_time=HAL_GetTick();
    //这个循环用于换弹
    while(1)
    {

        if(launcher.fire_mode==Fire_OFF)
            break;

        get__position=motor_2006_measure[0].total_ecd;
        launcher.relative_angle_get=motor_ecd_to_angle_change(launcher.trigger.motor_measure->ecd,
                                                              launcher.trigger.motor_measure->offset_ecd);
//推弹电机2006的串级Pid
        up_speed_out = pid_calc(&chassis.motor_chassis[RF].angle_p,//内环
                                get__position,
                                goal__ecd);
        chassis.motor_chassis[RF].give_current= (int16_t) pid_calc(&chassis.motor_chassis[RF].speed_p,//外环
                                                                   chassis.motor_chassis[RF].motor_measure->speed_rpm,
                                                                   up_speed_out);

        chassis.motor_chassis[RF].give_current = (int16_t) (- chassis.motor_chassis[RF].give_current);

        launcher.relative_angle_set = turn[turn_i];//实测角度、
        //todo 这里做一个角度处理
        if(launcher.relative_angle_set>180)
        {
            launcher.relative_angle_set=launcher.relative_angle_set-360;
        }
        else if(launcher.relative_angle_set<-180)
        {
            launcher.relative_angle_set=launcher.relative_angle_set+360;
        }

//换弹电机6020串级Pid
        launcher.trigger.speed = pid_loop_calc(&launcher.trigger.angle_p, launcher.relative_angle_get,
                                               launcher.relative_angle_set,180,-180);
        launcher.trigger.give_current= pid_calc_my(&launcher.trigger.speed_p,launcher.trigger.motor_measure->speed_rpm,launcher.trigger.speed);
        //当换弹电机设定角度与目标角度相差小于0.1°并且换弹时间超过4秒钟时，认为换弹成功
        if( abs(launcher.relative_angle_set-launcher.relative_angle_get)<0.1&&HAL_GetTick()-turn_start_time>4000)
            break;

        osDelay(2);
    }
    //更换推弹电机状态，
    pushbeam_state=1;
    //改变推弹电机的设定位置，
    pushbeam_handle();
//这个循环用于推弹
    while(1)
    {

        if(launcher.fire_mode==Fire_OFF)
            break;
        launcher.relative_angle_get=motor_ecd_to_angle_change(launcher.trigger.motor_measure->ecd,
                                                              launcher.trigger.motor_measure->offset_ecd);
        get__position=chassis.motor_chassis[RF].motor_measure->total_ecd;
        //当推弹电机大概接触到飞镖时，通过减小电机最大输出限幅来减慢推弹电机推动速度，防止给飞镖初速度造成误差
        if(get__position>init_ecd+80000)
        {
            chassis.motor_chassis[RF].angle_p.max_output=1500;//1000
            chassis.motor_chassis[RF].speed_p.max_output=3500;//2700
        }

        up_speed_out = pid_calc(&chassis.motor_chassis[RF].angle_p,
                                get__position,
                                goal__ecd);
        chassis.motor_chassis[RF].give_current= (int16_t) pid_calc(&chassis.motor_chassis[RF].speed_p,
                                                                   chassis.motor_chassis[RF].motor_measure->speed_rpm,
                                                                   up_speed_out);

        chassis.motor_chassis[RF].give_current = (int16_t) (- chassis.motor_chassis[RF].give_current);
        trigger_block_handle();//推弹时判断堵转

        launcher.trigger.speed = pid_loop_calc(&launcher.trigger.angle_p, launcher.relative_angle_get,
                                               launcher.relative_angle_set,180,-180);
        launcher.trigger.give_current= pid_calc(&launcher.trigger.speed_p,launcher.trigger.motor_measure->speed_rpm,launcher.trigger.speed);
//当推弹电机ecd位置大概回到初始位置时，认为
        if(goal__ecd-20000<=get__position)
        {
            break;
        }
        osDelay(2);

    }
    pushbeam_state=0;//初始位置状态
    pushbeam_handle();
    //这个循环用于推弹电机复位
    while(1)
    {
        //推弹完成后，推弹电机速度恢复如初
        chassis.motor_chassis[RF].angle_p.max_output=3000;
        chassis.motor_chassis[RF].speed_p.max_output=6000;
        launcher.relative_angle_get=motor_ecd_to_angle_change(launcher.trigger.motor_measure->ecd,
                                                              launcher.trigger.motor_measure->offset_ecd);
        if(launcher.fire_mode==Fire_OFF)
            break;
        get__position = chassis.motor_chassis->motor_measure->total_ecd;

        up_speed_out = pid_calc(&chassis.motor_chassis[RF].angle_p,
                                get__position,
                                goal__ecd);


        chassis.motor_chassis[RF].give_current= (int16_t) pid_calc(&chassis.motor_chassis[RF].speed_p,
                                                                   chassis.motor_chassis[RF].motor_measure->speed_rpm,
                                                                   up_speed_out);

        chassis.motor_chassis[RF].give_current = (int16_t) (- chassis.motor_chassis[RF].give_current);


        launcher.trigger.speed = pid_loop_calc(&launcher.trigger.angle_p, launcher.relative_angle_get,
                                               launcher.relative_angle_set, 180, -180);
        launcher.trigger.give_current = pid_calc(&launcher.trigger.speed_p, launcher.trigger.motor_measure->speed_rpm,
                                                 launcher.trigger.speed);
        //当转速小于5r/并且推弹电机到达接近初始位置时，认为推弹电机已复位
        if (get__position < 50000 && abs(chassis.motor_chassis->motor_measure->speed_rpm) <5)
        {
            init_ecd = get__position;//由于每次的推动复位，推弹电机在初始位置的ecd值可能会改变，所以每次复位都改变设定的初始ecd值
            break;
        }
        osDelay(2);
    }

}

void double_shoot_handle()
{


    if(launcher.fire_mode==Fire_ON)
    {

        single_shoot_handle();//发射一枚飞镖
        single_shoot_handle();
        shoot_2=0;            //发射完毕，发射状态为失能
        launcher.fire_mode=Fire_OFF;//关闭摩擦轮
    }

}
//如下代码主要用于手动调试，手动发射
void launcher_mode_set(){

    launcher.relative_angle_get=motor_ecd_to_angle_change(launcher.trigger.motor_measure->ecd,
                                                          launcher.trigger.motor_measure->offset_ecd);
    //摩擦轮关闭时,做拨杆向上拨一下开启摩擦轮
    if(!switch_is_up(rc_last_sw_L)&&switch_is_up(rc_ctrl.rc.s[RC_s_L])&&launcher.fire_mode==Fire_OFF)
    {
        launcher.fire_mode=Fire_ON;
    }
        //摩擦轮开启时,做拨杆向上拨一下关闭摩擦轮  控制达到妙电机旋转九十度
    else if(!switch_is_up(rc_last_sw_L)&&switch_is_up(rc_ctrl.rc.s[RC_s_L])&&launcher.fire_mode==Fire_ON)
    {
        launcher.fire_mode=Fire_OFF;
    }

    //拨轮控制
    if(launcher.fire_mode==Fire_ON&&switch_is_down(rc_ctrl.rc.s[RC_s_L]))
    {
        if((!switch_is_down(rc_last_sw_L)&&switch_is_down(rc_ctrl.rc.s[RC_s_L])||KeyBoard.Mouse_l.status==KEY_CLICK))
        {
            // launcher.trigger_cmd=SHOOT_SINGLE;
            shoot_2=1;
        }

    }
    else
    {
        //  launcher.trigger_cmd=SHOOT_CLOSE;
    }

    rc_last_sw_L=rc_ctrl.rc.s[RC_s_L];

}
int once=0;
int doorstartcnt=0;
int doorOpeningcnt=0;
int doorClosecnt=0;
uint8_t monitorDartDoorStatus=1;
//飞镖系统服务器模拟器，可忽略
void control_in_referee()
{
    doorstartcnt++;
    if(once==0)
    {
        if(monitorDartDoorStatus==1)
        {
            aRGB_led_show(0xffff0000);//红灯
            doorstartcnt++;
            if(doorstartcnt>=5000)
                monitorDartDoorStatus=2;//闸门正在打开
        }
        if(monitorDartDoorStatus==2)
        {
            aRGB_led_show(0xff00ff00);//绿灯
            doorOpeningcnt++;
            if(doorOpeningcnt>=5000)
            {
                monitorDartDoorStatus=0;//闸门开启完毕
            }
        }
        if(monitorDartDoorStatus==0)//闸门开启计时
        {
            aRGB_led_show(0xff0000ff);//蓝灯
            doorClosecnt++;
            if(doorClosecnt>=5000)
            {
                once=1;
            }
        }
    }else
    {
        monitorDartDoorStatus=1;//飞镖门关闭
        aRGB_led_show(0xffffffff);//蓝灯

    }

    dart_door_status=Referee.DartClient.dart_launch_opening_status;
    //  dart_door_status=monitorDartDoorStatus;
    if((dart_door_status==0&&dart_door_last_status!=0))
    {
        launcher.fire_mode=Fire_ON;
        shoot_2=1;
    }
    dart_door_last_status= dart_door_status;
}

//发射机构控制
void launcher_control()
{

    if(launcher.fire_mode==Fire_ON&&shoot_2==1)
    {

        launcher.relative_angle_get=motor_ecd_to_angle_change(launcher.trigger.motor_measure->ecd,
                                                              launcher.trigger.motor_measure->offset_ecd);

//黑2 turn=3
//黑5号，一号位 turn=1
        launcher.FL.speed=launcher_speed[turn_i][0];//+  左前
        launcher.BL.speed=launcher_speed[turn_i][1];//-  右前
        launcher.FR.speed=launcher_speed[turn_i][2];//-   右后
        launcher.BR.speed=launcher_speed[turn_i][3];//+   左后

        launcher.trigger.speed = pid_loop_calc(&launcher.trigger.angle_p, launcher.relative_angle_get,
                                               launcher.relative_angle_set,180,-180);
        launcher.trigger.give_current= pid_calc(&launcher.trigger.speed_p,launcher.trigger.motor_measure->speed_rpm,launcher.trigger.speed);

    }

    else if(launcher.fire_mode==Fire_OFF)
    {
        launcher.FL.speed=0;
        launcher.FR.speed=0;
        launcher.BL.speed=0;
        launcher.BR.speed=0;
        launcher.trigger.give_current=0;

        chassis.motor_chassis[RF].give_current=0;
    }


    launcher.BL.give_current= pid_calc(&launcher.BL.speed_p,launcher.BL.motor_measure->speed_rpm, launcher.BL.speed);
    launcher.BR.give_current= pid_calc(&launcher.BR.speed_p,launcher.BR.motor_measure->speed_rpm, launcher.BR.speed);
    launcher.FL.give_current= pid_calc(&launcher.FL.speed_p,launcher.FL.motor_measure->speed_rpm, launcher.FL.speed);
    launcher.FR.give_current= pid_calc(&launcher.FR.speed_p,launcher.FR.motor_measure->speed_rpm, launcher.FR.speed);

}
//使推弹电机回到最初位置，用于一上电时启用，且只启用一次
void push_bullet_reset()
{
    while(1)
    {
//保证这个if判断句只执行一次
        if(begin_count==1)
        {
            reset_time=HAL_GetTick();
            begin_count=0;
        }
//给推弹电机0.8秒的复位时间
        if( HAL_GetTick()- reset_time>800) {

            break;
        }
        up_speed_out =-1500;//设置推弹电机归位速度
        chassis.motor_chassis[RF].give_current = (int16_t) pid_calc(&chassis.motor_chassis[RF].speed_p,
                                                                    chassis.motor_chassis[RF].motor_measure->speed_rpm,
                                                                    up_speed_out);

        chassis.motor_chassis[RF].give_current = (int16_t) (-chassis.motor_chassis[RF].give_current);
        CAN_cmd_motor(CAN_2,
                      CAN_MOTOR_0x1FF_ID,
                      0,     //推弹电机
                      0,
                      0,
                      chassis.motor_chassis[RF].give_current);
        osDelay(2);
    }
    init_ecd=chassis.motor_chassis[RF].motor_measure->total_ecd;//回到最初位置时，将该位置的ecd设置为最初ecd

}

void launcher_init(){

    launcher.trigger_cmd=SHOOT_CLOSE;//初始时发射机构默认关闭

    launcher.fire_last_mode=Fire_OFF;//初始时摩擦轮默认关闭

    launcher.fire_mode=Fire_OFF;//初始时摩擦轮默认关闭
    //Till=0;
    //摩擦轮电机初始化
    launcher.FL.motor_measure=&motor_3508_dart[0];  //左前1
    launcher.FR.motor_measure=&motor_3508_dart[1];  //右后2
    launcher.BL.motor_measure=&motor_3508_dart[2];  //右前3
    launcher.BR.motor_measure=&motor_3508_dart[3];  //左后4

    chassis.motor_chassis[0].motor_measure=&motor_2006_measure[0];//上弹电机初始化

    launcher.trigger.motor_measure=&motor_turn_measure;//翻转电机初始化


    //上弹电机pid
    chassis.motor_chassis[0].speed_p.max_output=up_speed_PID_MAX_OUT;
    chassis.motor_chassis[0].speed_p.integral_limit=up_speed_PID_MAX_IOUT;
    chassis.motor_chassis[0].speed_p.p=up_speed_PID_KP;
    chassis.motor_chassis[0].speed_p.i=up_speed_PID_KI;
    chassis.motor_chassis[0].speed_p.d=up_speed_PID_KD;

    chassis.motor_chassis[0].angle_p.max_output=up_angle_PID_MAX_OUT;
    chassis.motor_chassis[0].angle_p.integral_limit=up_angle_PID_MAX_IOUT;
    chassis.motor_chassis[0].angle_p.p=up_angle_PID_KP;
    chassis.motor_chassis[0].angle_p.i=up_angle_PID_KI;
    chassis.motor_chassis[0].angle_p.d=up_angle_PID_KD;

    //初始化2006的ecd值
    chassis.motor_chassis[RF].motor_measure->offset_ecd=motor_2006_measure->ecd;
    chassis.motor_chassis[RF].motor_measure->total_ecd=motor_2006_measure->ecd;
    //摩擦轮pid
    launcher.BR.speed_p.p=SHOOT_FIRE_R_PID_KP;
    launcher.BR.speed_p.i=SHOOT_FIRE_R_PID_KI;
    launcher.BR.speed_p.d=SHOOT_FIRE_R_PID_KD;
    launcher.BR.speed_p.max_output=SHOOT_FIRE_R_PID_MAX_OUT;
    launcher.BR.speed_p.integral_limit=SHOOT_FIRE_R_PID_MAX_IOUT;

    launcher.BL.speed_p.p=SHOOT_FIRE_R_PID_KP;
    launcher.BL.speed_p.i=SHOOT_FIRE_R_PID_KI;
    launcher.BL.speed_p.d=SHOOT_FIRE_R_PID_KD;
    launcher.BL.speed_p.max_output=SHOOT_FIRE_R_PID_MAX_OUT;
    launcher.BL.speed_p.integral_limit=SHOOT_FIRE_R_PID_MAX_IOUT;

    launcher.FL.speed_p.p=SHOOT_FIRE_R_PID_KP;
    launcher.FL.speed_p.i=SHOOT_FIRE_R_PID_KI;
    launcher.FL.speed_p.d=SHOOT_FIRE_R_PID_KD;
    launcher.FL.speed_p.max_output=SHOOT_FIRE_R_PID_MAX_OUT;
    launcher.FL.speed_p.integral_limit=SHOOT_FIRE_R_PID_MAX_IOUT;

    launcher.FR.speed_p.p=SHOOT_FIRE_R_PID_KP;
    launcher.FR.speed_p.i=SHOOT_FIRE_R_PID_KI;
    launcher.FR.speed_p.d=SHOOT_FIRE_R_PID_KD;
    launcher.FR.speed_p.max_output=SHOOT_FIRE_R_PID_MAX_OUT;
    launcher.FR.speed_p.integral_limit=SHOOT_FIRE_R_PID_MAX_IOUT;

    //翻转电机pid
    launcher.trigger.angle_p.p=SHOOT_TRI_ANGLE_PID_KP;
    launcher.trigger.angle_p.i=SHOOT_TRI_ANGLE_PID_KI;
    launcher.trigger.angle_p.d=SHOOT_TRI_ANGLE_PID_KD;
    launcher.trigger.angle_p.max_output=SHOOT_TRI_ANGLE_PID_MAX_OUT;
    launcher.trigger.angle_p.integral_limit=SHOOT_TRI_ANGLE_PID_MAX_IOUT;

    launcher.trigger.speed_p.p=SHOOT_TRI_SPEED_PID_KP;
    launcher.trigger.speed_p.i=SHOOT_TRI_SPEED_PID_KI;
    launcher.trigger.speed_p.d=SHOOT_TRI_SPEED_PID_KD;
    launcher.trigger.speed_p.max_output=SHOOT_TRI_SPEED_PID_MAX_OUT;
    launcher.trigger.speed_p.integral_limit=SHOOT_TRI_SPEED_PID_MAX_IOUT;


    launcher.trigger.motor_measure->offset_ecd=8167;
    launcher.relative_angle_set-27.68732;

    //电机要转的调度
    launcher.relative_angle_set=-27.68732;

    //电机要转的调度
    turn[0]=(float)-120.146484;//-27.801875
    turn[1]=(float)58.886718;//-120.1094
    turn[2]=(float)-27.905273;//152.757578
    turn[3]=(float)152.709960;
    turn_i=1;

}

//上弹电机控制

//            if(turn_count==0&&rc_ctrl.rc.ch[4]==0)
//            {
//                chassis.motor_chassis[RF].give_current=0;
//            }

//            else if(turn_count==1 && rc_ctrl.rc.ch[4]!=0) {
//    set_position = 0;

// 推弹电机速度的杆量控制
/*
  set_position=  (float) rc_ctrl.rc.ch[4] / 1000.0f;
      goal_ecd = set_position * M2006_POSITION_TO_ECD;

      up_speed_out = pid_calc(&chassis.motor_chassis[RF].angle_p,
                              get_position,
                              goal_ecd);
      chassis.motor_chassis[RF].give_current = (int16_t) pid_calc(&chassis.motor_chassis[RF].speed_p,
                                                                  chassis.motor_chassis[RF].motor_measure->speed_rpm,
                                                                  up_speed_out);

      chassis.motor_chassis[RF].give_current = (int16_t) (-chassis.motor_chassis[RF].give_current);

*/