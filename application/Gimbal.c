/*  Include */
#include "Gimbal.h"
#include "main.h"
#include "cmsis_os.h"
#include "launcher.h"
#include "can_receive.h"
#include "user_lib.h"
#include "Atti.h"
#include "Chassis.h"
#include "protocol_shaob.h"
#include "packet.h"

uint8_t safe_lock = 0;
uint16_t flag=0;


/*      变量      */
gimbal_t gimbal;
extern RC_ctrl_t rc_ctrl;

extern launcher_t launcher;
extern Eulr_t Eulr;
extern fp32 INS_angle[3];
extern fp32 INS_gyro[3];
extern fp32 INS_quat[4];
extern chassis_t chassis;
extern robot_ctrl_info_t robot_ctrl;

vision_t vision_data;
first_order_filter_type_t pitch_first_order_set;
first_order_filter_type_t pitch_current_first_order_set;
first_order_filter_type_t yaw_first_order_set;
first_order_filter_type_t yaw_current_first_order_set;

fp32 pitch_absolute_angle_get;
fp32 yaw_absolute_angle_get;
fp32 turn_relative_angle_get;
/*      函数及声明   */
static void gimbal_init();
static void gimbal_mode_set();
static void gimbal_active_handle();
static void gimbal_relax_handle();
static void gimbal_ctrl_loop_cal();
static void gimbal_angle_update();
static void gimbal_mode_change();
static void gimbal_auto_handle();

void gimbal_task(void const*pvParameters)
{
    //任务初始化时间
    vTaskDelay(GIMBAL_TASK_INIT_TIME);

    //云台初始化
    gimbal_init();

    //发射机构初始化
    launcher_init();
    push_bullet_reset();//推弹电机上电初始化，复位
    //TODO:判断电机是否上线
    motor_2006_measure->total_ecd=0;

    //主任务体
    while(1){

        gimbal_angle_update();//更新绝对、相对角度接收值

        gimbal_mode_set();//根据遥控器设置云台控制模式
     control_in_referee();//飞镖系统服务器模拟器，可忽略
        launcher_mode_set();

        launcher_control();
        gimbal_active_handle();
        //todo 这额Switch实际没有
        switch (gimbal.mode) {

            case GIMBAL_RELAX:
            {
                gimbal_relax_handle();
                break;
            }

            case GIMBAL_BACK:
            case GIMBAL_ACTIVE:
            {
                gimbal_active_handle();  //得到遥控器对云台电机的控制
                gimbal_ctrl_loop_cal();
                break;
            }

            case GIMBAL_AUTO:
            {
                gimbal_auto_handle();
                gimbal_ctrl_loop_cal();
            }


            default:{

                break;
            }

        }



        CAN_cmd_motor(CAN_2,
                      CAN_MOTOR_0x1FF_ID,
                      launcher.trigger.give_current,     //6020换弹电机
                      gimbal.pitch.give_current,
                      gimbal.yaw.give_current,
                      chassis.motor_chassis[RF].give_current);  //2006推弹电机

        CAN_cmd_motor(CAN_1,
                      CAN_MOTOR_0x200_ID,
                      launcher.FL.give_current,
                      launcher.FR.give_current,
                      launcher.BL.give_current,
                      launcher.BR.give_current);


        vTaskDelay(2);
    }
}


static void gimbal_init(){

    gimbal.yaw.motor_measure=&motor_yaw_measure;
    gimbal.pitch.motor_measure=&motor_pitch_measure;

    gimbal.mode=gimbal.last_mode=GIMBAL_RELAX;
    //pid初始化
    pid_init(&gimbal.yaw.angle_p,
             GIMBAL_YAW_ANGLE_MAX_OUT,
             GIMBAL_YAW_ANGLE_MAX_IOUT,
             GIMBAL_YAW_ANGLE_PID_KP,
             GIMBAL_YAW_ANGLE_PID_KI,
             GIMBAL_YAW_ANGLE_PID_KD);

    pid_init(&gimbal.yaw.speed_p,
             GIMBAL_YAW_SPEED_MAX_OUT,
             GIMBAL_YAW_SPEED_MAX_IOUT,
             GIMBAL_YAW_SPEED_PID_KP,
             GIMBAL_YAW_SPEED_PID_KI,
             GIMBAL_YAW_SPEED_PID_KD);

    pid_init(&gimbal.pitch.angle_p,
             GIMBAL_PITCH_ANGLE_MAX_OUT,
             GIMBAL_PITCH_ANGLE_MAX_IOUT,
             GIMBAL_PITCH_ANGLE_PID_KP,
             GIMBAL_PITCH_ANGLE_PID_KI,
             GIMBAL_PITCH_ANGLE_PID_KD);

    pid_init(&gimbal.pitch.speed_p,
             GIMBAL_PITCH_SPEED_MAX_OUT,
             GIMBAL_PITCH_SPEED_MAX_IOUT,
             GIMBAL_PITCH_SPEED_PID_KP,
             GIMBAL_PITCH_SPEED_PID_KI,
             GIMBAL_PITCH_SPEED_PID_KD);




    first_order_filter_init(&pitch_first_order_set, 0.f, (const fp32 *) 500);
    first_order_filter_init(&pitch_current_first_order_set, 5, (const fp32 *) 30);
    first_order_filter_init(&yaw_first_order_set, 5, (const fp32 *) 30);
    //初始化时 云台设为未回中状态
    gimbal.yaw_is_back=0;
    gimbal.pitch_is_back=0;

    //yaw轴和pitch轴电机的校准编码值
    gimbal.yaw.motor_measure->offset_ecd=8040;
    gimbal.pitch.motor_measure->offset_ecd=5547;

    gimbal.yaw.relative_angle_set = gimbal.yaw.relative_angle_get;
}

static void gimbal_mode_set(){

    switch (rc_ctrl.rc.s[RC_s_R]) {

        case RC_SW_DOWN:
        {
            gimbal.last_mode=gimbal.mode;
            gimbal.mode=GIMBAL_RELAX;
            break;
        }

        case RC_SW_MID:
        case RC_SW_UP:
        {
            gimbal.last_mode=gimbal.mode;
            if(gimbal.last_mode==GIMBAL_RELAX)
            {
                gimbal.mode=GIMBAL_BACK;
                gimbal.yaw_is_back=0;
                gimbal.pitch_is_back=0;
            }
            else if(gimbal.mode==GIMBAL_BACK)
            {
                gimbal.mode=GIMBAL_ACTIVE;
            }
            break;
        }

        default:
            break;

    }
        gimbal_mode_change();
}


static void gimbal_mode_change() {
    if (gimbal.mode == GIMBAL_ACTIVE || gimbal.mode == GIMBAL_BACK)
    {   //自瞄判定
        if (rc_ctrl.rc.ch[AUTO_CHANNEL] > 50)
        {
            vision_data.mode = 0x21;
        }
        else
        {
            vision_data.mode = 0;
        }

        if ( (robot_ctrl.target_lock == 0x31) || (rc_ctrl.rc.ch[AUTO_CHANNEL]>50) )
        {
            gimbal.last_mode = GIMBAL_ACTIVE;
            gimbal.mode = GIMBAL_AUTO;
        }
    }
    else if (gimbal.mode == GIMBAL_AUTO)
    {   //0x32表示自瞄数据无效
        if (robot_ctrl.target_lock == 0x32) {
            gimbal.last_mode = GIMBAL_AUTO;
            gimbal.mode = GIMBAL_ACTIVE;//默认回到一般模式
            vision_data.mode = 0;
        }
    }
    // TODO: 离线检测问题 记得开关
//    if(detect_list[DETECT_LAUNCHER_2006_SINGLE_SHOT].status==OFFLINE)
//    {
//        gimbal.mode=GIMBAL_RELAX;
//    }
}

static void gimbal_active_handle()
{
    gimbal.pitch.absolute_angle_set=(float)(rc_ctrl.rc.ch[PITCH_CHANNEL]*5);
    if(gimbal.pitch.relative_angle_get>44)
    {
        gimbal.pitch.absolute_angle_set=-gimbal.pitch.absolute_angle_set;
    }
    else if(gimbal.pitch.relative_angle_get<34.8)
    {
        gimbal.pitch.absolute_angle_set=-gimbal.pitch.absolute_angle_set;
    }

    gimbal.yaw.absolute_angle_set-=(float)(rc_ctrl.rc.ch[YAW_CHANNEL]*RC_TO_PITCH*GIMBAL_RC_MOVE_RATIO_PIT);

}

static void gimbal_relax_handle(){
    gimbal.yaw.give_current=0;
    gimbal.pitch.give_current=0;

    launcher.trigger.give_current=0;
    motor_2006_measure[0].given_current=0;
    chassis.motor_chassis[RF].give_current=0;

    launcher.FL.give_current=0;
    launcher.FR.give_current=0;
    launcher.BL.give_current=0;
    launcher.BR.give_current=0;
    //TODO: 拨弹电机电流 gimbal.trigger.give_current=0;
}

static void gimbal_auto_handle()
{
    gimbal.yaw.absolute_angle_set=robot_ctrl.yaw;
//    gimbal.pitch.absolute_angle_set=robot_ctrl.pitch;
    gimbal.pitch.absolute_angle_set=-3000;

}


static void gimbal_ctrl_loop_cal(){

    gimbal.pitch.give_current= pid_calc(&gimbal.pitch.speed_p,
                                      gimbal.pitch.motor_measure->speed_rpm,
                                        gimbal.pitch.absolute_angle_set);

    if(rc_ctrl.rc.s[RC_s_R]==3){

        if(flag==0)
        {
            launcher.trigger_cmd=SHOOT_CLOSE;
            launcher.fire_mode=Fire_OFF;
          //  gimbal.yaw.relative_angle_set=0;
            flag=1;
        }
        gimbal.yaw.relative_angle_set-=rc_ctrl.rc.ch[YAW_CHANNEL]*RC_TO_PITCH*0.03f;
        if(gimbal.yaw.relative_angle_set>57){
            gimbal.yaw.relative_angle_set =57;
        }else if(gimbal.yaw.relative_angle_set<-53){
            gimbal.yaw.relative_angle_set =-53;
        }
        gimbal.yaw.gyro_set= pid_loop_calc(&gimbal.yaw.angle_p,
                                           gimbal.yaw.relative_angle_get,
                                           gimbal.yaw.relative_angle_set,
                                          180, -180);
        gimbal.yaw.give_current=pid_calc(&gimbal.yaw.speed_p,
                                            gimbal.yaw.motor_measure->speed_rpm,
                                            -gimbal.yaw.gyro_set);
    }
    else if(rc_ctrl.rc.s[RC_s_R]==1)
    {
        flag=0;
        launcher.fire_mode=Fire_ON;
        launcher.trigger_cmd=SHOOT_SINGLE;
        gimbal.yaw.relative_angle_set=30;
        gimbal.yaw.gyro_set= pid_loop_calc(&gimbal.yaw.angle_p,
                                           gimbal.yaw.relative_angle_get,
                                           gimbal.yaw.relative_angle_set,
                                           180, -180);
        gimbal.yaw.give_current= pid_calc(&gimbal.yaw.speed_p,
                                          gimbal.yaw.motor_measure->speed_rpm,
                                          -gimbal.yaw.gyro_set);
    }

}





static void gimbal_angle_update(){//用pit相对角判断是否达到限位
     yaw_absolute_angle_get=INS_angle[0]*MOTOR_RAD_TO_ANGLE;
    gimbal.yaw.absolute_angle_get=yaw_absolute_angle_get;
    gimbal.yaw.relative_angle_get= -motor_ecd_to_angle_change(gimbal.yaw.motor_measure->ecd,gimbal.yaw.motor_measure->offset_ecd);

    pitch_absolute_angle_get=-INS_angle[2]*MOTOR_RAD_TO_ANGLE;
    gimbal.pitch.absolute_angle_get=pitch_absolute_angle_get;
    gimbal.pitch.relative_angle_get= -motor_ecd_to_angle_change(gimbal.pitch.motor_measure->ecd,gimbal.pitch.motor_measure->offset_ecd);

    vision_data.yaw = gimbal.yaw.absolute_angle_get;
    vision_data.pitch = gimbal.pitch.absolute_angle_get;
    vision_data.shoot_speed = Referee.ShootData.bullet_speed;
    vision_data.roll = 0;
    for (int i = 0; i < 4; ++i) {
        vision_data.quaternion[i] = INS_quat[i];
    }

//    rm_queue_data(VISION_ID,&vision_data,sizeof(vision_t));
}

//)

//        CAN_cmd_motor(CAN_2,
//                      CAN_MOTOR_0x1FF_ID,
//                      0,     //拨弹电机
//                      gimbal.pitch.give_current,
//                      gimbal.yaw.give_current,  //2006电机
//                      chassis.motor_chassis[RF].give_current);

//        CAN_cmd_motor(CAN_2,
//                      CAN_MOTOR_0x1FF_ID,
//                      0,
//                      gimbal.pitch.give_current,
//                      gimbal.yaw.give_current,  //2006电机
//                      chassis.motor_chassis[RF].give_current);
