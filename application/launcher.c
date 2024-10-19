/*���ӵ��id��                            �ĸ�Ħ����                     �������꣺      x(ǰ)
 *                                           ǰ
                ****         ****                     ****               ****                              |
                          FL                                     BL                          |
                ****         ****                     ****               ****                              |
                                                                                                                |
    ��                                                                                         ��
                                                                                                                |
                ****              ****                     ****               ****                              |
                           BR                                       FR                 |
                ****              ****                     ****               ****
                                                 ��
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

uint32_t continue_shoot_time;//ң������߲���down�ĳ���ʱ�� ���� ���������µĳ���ʱ��

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
//ͨ����ֵ���з�������ĳ�ʼ��
launcher_t launcher;
fp32 set_position;
float goal__ecd;//�趨�Ƶ������ecdֵ
float get__position;//��ǰ�Ƶ������ecdֵ
float up_speed_out;


float turn[4];//����4��������Ӧ4����������Ƕ�
int turn_i;   //��ǰ�趨�Ƕȵ�����
int launcher_speed[4][4] = {{3000, -3000, -5000, 5000}, //����1��
                            {3000, -3000, -5000, 5000}, //����2��
                            {3000, -3000, -5000, 5000}, //����3��
                            {3000, -3000, -5000, 5000}}; //����4��

int32_t pushbeam_state=0;
int32_t init_ecd=0;
#define bool	_Bool
#define true	1
#define false	0

uint8_t blocked_flag;
uint8_t reverse_flag;
uint32_t continue_shoot_time;//ң������߲���down�ĳ���ʱ�� ���� ���������µĳ���ʱ��


uint32_t begin_count=1,reset_time;//һ��flag��־λ       //һ����¼�ϵ�ʱ��ʱ��
uint32_t reverse_start_time;
uint32_t blocked_start_time;
int32_t shoot_2=0;              //�����жϷ����Ƿ�Ҫ���䣬1Ϊ��������

uint8_t dart_door_status,dart_door_last_status;
//#define TRIGGER_CONTINUES_SPEED -4000


#define TRIGGER_REVERSE_SPEED 3000       //�Ƶ������תʱ�ķ�ת�ٶ�
#define CONTINUES_BLOCKED_JUDGE() (HAL_GetTick()-blocked_start_time>500)
#define TRIGGER_REVERSE_TIME_JUDGE() (HAL_GetTick()-reverse_start_time<100)
bool is_blocked(){
    if(TRIGGER_CONTINUES_SPEED>0){


        if (blocked_flag==false && launcher.trigger.motor_measure->speed_rpm <= 0.3*TRIGGER_CONTINUES_SPEED){
            blocked_start_time=HAL_GetTick();//��ȡ��ת��ʼʱ��
            blocked_flag=true;
        }


        //��ʶλΪ1ʱ���Ѿ���ʼ��ת���ж��Ƿ��ת�ﵽһ��ʱ�䣬���ﵽ�����ж���ת
        if (blocked_flag==true && launcher.trigger.motor_measure->speed_rpm <= 0.3*TRIGGER_CONTINUES_SPEED){
            if(CONTINUES_BLOCKED_JUDGE()){
                blocked_flag=false;
                return true;
            }
        }
        return false;
    }

    else if(TRIGGER_CONTINUES_SPEED<0){


        //�ڱ�ʶλΪ0ʱ�����ת�ٵ�����ֵʱ���ж���ת��ʼ
        if (blocked_flag==false && abs(chassis.motor_chassis[RF].motor_measure->speed_rpm- up_speed_out)>1000){
            blocked_start_time=HAL_GetTick();//��ȡ��ת��ʼʱ��
            blocked_flag=true;
        }

        //��ʶλΪ1ʱ���Ѿ���ʼ��ת���ж��Ƿ��ת�ﵽһ��ʱ�䣬���ﵽ�����ж���ת
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
//���Ƶ������ת���Ʋ���ʱ������
void  trigger_block_handle(){
    //�ж϶�ת���ҷ�ת��ʶΪ0ʱ
    if((is_blocked() && reverse_flag==false)){
        reverse_flag=true;//�ж���ʼ��ת
        reverse_start_time=HAL_GetTick();//��ȡ��ʼ��תʱ��
    }

    //�ж���ת��ʼ����ʱ��û�дﵽ��ת����ʱ��
    if(reverse_flag==true && TRIGGER_REVERSE_TIME_JUDGE()){
        chassis.motor_chassis[RF].give_current=TRIGGER_REVERSE_SPEED;//�����������Ϊ��ת�ٶ�
    }

    else{
        reverse_flag=false;
    }
}
//�����Ƶ����״̬�ı�
void pushbeam_handle()
{

    switch (pushbeam_state)
    {
        case 0:
            goal__ecd=init_ecd;
            break;
        case 1:
            goal__ecd=init_ecd+185000;//���Թ����˵�����ӳ�ʼλ�õ��ƶ�����λ�ã����ת����ecdֵΪ185000����
            break;
    }
}
//����һö���������߼�
//�����������Ƶ�����Ƶ����Ƶ������λ
//�൱�����߳��п��߳�
void single_shoot_handle()
{
    turn_i++;
    if(turn_i>=4)turn_i=0;
    turn_start_time=HAL_GetTick();
    //���ѭ�����ڻ���
    while(1)
    {

        if(launcher.fire_mode==Fire_OFF)
            break;

        get__position=motor_2006_measure[0].total_ecd;
        launcher.relative_angle_get=motor_ecd_to_angle_change(launcher.trigger.motor_measure->ecd,
                                                              launcher.trigger.motor_measure->offset_ecd);
//�Ƶ����2006�Ĵ���Pid
        up_speed_out = pid_calc(&chassis.motor_chassis[RF].angle_p,//�ڻ�
                                get__position,
                                goal__ecd);
        chassis.motor_chassis[RF].give_current= (int16_t) pid_calc(&chassis.motor_chassis[RF].speed_p,//�⻷
                                                                   chassis.motor_chassis[RF].motor_measure->speed_rpm,
                                                                   up_speed_out);

        chassis.motor_chassis[RF].give_current = (int16_t) (- chassis.motor_chassis[RF].give_current);

        launcher.relative_angle_set = turn[turn_i];//ʵ��Ƕȡ�
        //todo ������һ���Ƕȴ���
        if(launcher.relative_angle_set>180)
        {
            launcher.relative_angle_set=launcher.relative_angle_set-360;
        }
        else if(launcher.relative_angle_set<-180)
        {
            launcher.relative_angle_set=launcher.relative_angle_set+360;
        }

//�������6020����Pid
        launcher.trigger.speed = pid_loop_calc(&launcher.trigger.angle_p, launcher.relative_angle_get,
                                               launcher.relative_angle_set,180,-180);
        launcher.trigger.give_current= pid_calc_my(&launcher.trigger.speed_p,launcher.trigger.motor_measure->speed_rpm,launcher.trigger.speed);
        //����������趨�Ƕ���Ŀ��Ƕ����С��0.1�㲢�һ���ʱ�䳬��4����ʱ����Ϊ�����ɹ�
        if( abs(launcher.relative_angle_set-launcher.relative_angle_get)<0.1&&HAL_GetTick()-turn_start_time>4000)
            break;

        osDelay(2);
    }
    //�����Ƶ����״̬��
    pushbeam_state=1;
    //�ı��Ƶ�������趨λ�ã�
    pushbeam_handle();
//���ѭ�������Ƶ�
    while(1)
    {

        if(launcher.fire_mode==Fire_OFF)
            break;
        launcher.relative_angle_get=motor_ecd_to_angle_change(launcher.trigger.motor_measure->ecd,
                                                              launcher.trigger.motor_measure->offset_ecd);
        get__position=chassis.motor_chassis[RF].motor_measure->total_ecd;
        //���Ƶ������ŽӴ�������ʱ��ͨ����С����������޷��������Ƶ�����ƶ��ٶȣ���ֹ�����ڳ��ٶ�������
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
        trigger_block_handle();//�Ƶ�ʱ�ж϶�ת

        launcher.trigger.speed = pid_loop_calc(&launcher.trigger.angle_p, launcher.relative_angle_get,
                                               launcher.relative_angle_set,180,-180);
        launcher.trigger.give_current= pid_calc(&launcher.trigger.speed_p,launcher.trigger.motor_measure->speed_rpm,launcher.trigger.speed);
//���Ƶ����ecdλ�ô�Żص���ʼλ��ʱ����Ϊ
        if(goal__ecd-20000<=get__position)
        {
            break;
        }
        osDelay(2);

    }
    pushbeam_state=0;//��ʼλ��״̬
    pushbeam_handle();
    //���ѭ�������Ƶ������λ
    while(1)
    {
        //�Ƶ���ɺ��Ƶ�����ٶȻָ����
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
        //��ת��С��5r/�����Ƶ��������ӽ���ʼλ��ʱ����Ϊ�Ƶ�����Ѹ�λ
        if (get__position < 50000 && abs(chassis.motor_chassis->motor_measure->speed_rpm) <5)
        {
            init_ecd = get__position;//����ÿ�ε��ƶ���λ���Ƶ�����ڳ�ʼλ�õ�ecdֵ���ܻ�ı䣬����ÿ�θ�λ���ı��趨�ĳ�ʼecdֵ
            break;
        }
        osDelay(2);
    }

}

void double_shoot_handle()
{


    if(launcher.fire_mode==Fire_ON)
    {

        single_shoot_handle();//����һö����
        single_shoot_handle();
        shoot_2=0;            //������ϣ�����״̬Ϊʧ��
        launcher.fire_mode=Fire_OFF;//�ر�Ħ����
    }

}
//���´�����Ҫ�����ֶ����ԣ��ֶ�����
void launcher_mode_set(){

    launcher.relative_angle_get=motor_ecd_to_angle_change(launcher.trigger.motor_measure->ecd,
                                                          launcher.trigger.motor_measure->offset_ecd);
    //Ħ���ֹر�ʱ,���������ϲ�һ�¿���Ħ����
    if(!switch_is_up(rc_last_sw_L)&&switch_is_up(rc_ctrl.rc.s[RC_s_L])&&launcher.fire_mode==Fire_OFF)
    {
        launcher.fire_mode=Fire_ON;
    }
        //Ħ���ֿ���ʱ,���������ϲ�һ�¹ر�Ħ����  ���ƴﵽ������ת��ʮ��
    else if(!switch_is_up(rc_last_sw_L)&&switch_is_up(rc_ctrl.rc.s[RC_s_L])&&launcher.fire_mode==Fire_ON)
    {
        launcher.fire_mode=Fire_OFF;
    }

    //���ֿ���
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
//����ϵͳ������ģ�������ɺ���
void control_in_referee()
{
    doorstartcnt++;
    if(once==0)
    {
        if(monitorDartDoorStatus==1)
        {
            aRGB_led_show(0xffff0000);//���
            doorstartcnt++;
            if(doorstartcnt>=5000)
                monitorDartDoorStatus=2;//բ�����ڴ�
        }
        if(monitorDartDoorStatus==2)
        {
            aRGB_led_show(0xff00ff00);//�̵�
            doorOpeningcnt++;
            if(doorOpeningcnt>=5000)
            {
                monitorDartDoorStatus=0;//բ�ſ������
            }
        }
        if(monitorDartDoorStatus==0)//բ�ſ�����ʱ
        {
            aRGB_led_show(0xff0000ff);//����
            doorClosecnt++;
            if(doorClosecnt>=5000)
            {
                once=1;
            }
        }
    }else
    {
        monitorDartDoorStatus=1;//�����Źر�
        aRGB_led_show(0xffffffff);//����

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

//�����������
void launcher_control()
{

    if(launcher.fire_mode==Fire_ON&&shoot_2==1)
    {

        launcher.relative_angle_get=motor_ecd_to_angle_change(launcher.trigger.motor_measure->ecd,
                                                              launcher.trigger.motor_measure->offset_ecd);

//��2 turn=3
//��5�ţ�һ��λ turn=1
        launcher.FL.speed=launcher_speed[turn_i][0];//+  ��ǰ
        launcher.BL.speed=launcher_speed[turn_i][1];//-  ��ǰ
        launcher.FR.speed=launcher_speed[turn_i][2];//-   �Һ�
        launcher.BR.speed=launcher_speed[turn_i][3];//+   ���

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
//ʹ�Ƶ�����ص����λ�ã�����һ�ϵ�ʱ���ã���ֻ����һ��
void push_bullet_reset()
{
    while(1)
    {
//��֤���if�жϾ�ִֻ��һ��
        if(begin_count==1)
        {
            reset_time=HAL_GetTick();
            begin_count=0;
        }
//���Ƶ����0.8��ĸ�λʱ��
        if( HAL_GetTick()- reset_time>800) {

            break;
        }
        up_speed_out =-1500;//�����Ƶ������λ�ٶ�
        chassis.motor_chassis[RF].give_current = (int16_t) pid_calc(&chassis.motor_chassis[RF].speed_p,
                                                                    chassis.motor_chassis[RF].motor_measure->speed_rpm,
                                                                    up_speed_out);

        chassis.motor_chassis[RF].give_current = (int16_t) (-chassis.motor_chassis[RF].give_current);
        CAN_cmd_motor(CAN_2,
                      CAN_MOTOR_0x1FF_ID,
                      0,     //�Ƶ����
                      0,
                      0,
                      chassis.motor_chassis[RF].give_current);
        osDelay(2);
    }
    init_ecd=chassis.motor_chassis[RF].motor_measure->total_ecd;//�ص����λ��ʱ������λ�õ�ecd����Ϊ���ecd

}

void launcher_init(){

    launcher.trigger_cmd=SHOOT_CLOSE;//��ʼʱ�������Ĭ�Ϲر�

    launcher.fire_last_mode=Fire_OFF;//��ʼʱĦ����Ĭ�Ϲر�

    launcher.fire_mode=Fire_OFF;//��ʼʱĦ����Ĭ�Ϲر�
    //Till=0;
    //Ħ���ֵ����ʼ��
    launcher.FL.motor_measure=&motor_3508_dart[0];  //��ǰ1
    launcher.FR.motor_measure=&motor_3508_dart[1];  //�Һ�2
    launcher.BL.motor_measure=&motor_3508_dart[2];  //��ǰ3
    launcher.BR.motor_measure=&motor_3508_dart[3];  //���4

    chassis.motor_chassis[0].motor_measure=&motor_2006_measure[0];//�ϵ������ʼ��

    launcher.trigger.motor_measure=&motor_turn_measure;//��ת�����ʼ��


    //�ϵ����pid
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

    //��ʼ��2006��ecdֵ
    chassis.motor_chassis[RF].motor_measure->offset_ecd=motor_2006_measure->ecd;
    chassis.motor_chassis[RF].motor_measure->total_ecd=motor_2006_measure->ecd;
    //Ħ����pid
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

    //��ת���pid
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

    //���Ҫת�ĵ���
    launcher.relative_angle_set=-27.68732;

    //���Ҫת�ĵ���
    turn[0]=(float)-120.146484;//-27.801875
    turn[1]=(float)58.886718;//-120.1094
    turn[2]=(float)-27.905273;//152.757578
    turn[3]=(float)152.709960;
    turn_i=1;

}

//�ϵ��������

//            if(turn_count==0&&rc_ctrl.rc.ch[4]==0)
//            {
//                chassis.motor_chassis[RF].give_current=0;
//            }

//            else if(turn_count==1 && rc_ctrl.rc.ch[4]!=0) {
//    set_position = 0;

// �Ƶ�����ٶȵĸ�������
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