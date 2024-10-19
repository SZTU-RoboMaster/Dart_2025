//
// Created by xhuanc on 2021/12/5.
//

// 这个文件好像没用到，后面可以删掉
#include "balance_ctrl.h"
#include "struct_typedef.h"
#include "PID.h"
#include "Chassis.h"
#include "Atti.h"
extern fp32 INS_angle[3] ;
extern chassis_t chassis;
extern Eulr_t Eulr;
extern fp32 INS_gyro[3];
fp32  Vertical_in_l;
fp32 Vertical_in_r;
fp32 balance_out;
fp32 velocity_out;
fp32 sum_out;

void balance_control(){
#if isBalance
//    输出轴逆时针是正电流 顺时针是反电流

//
//    Vertical_in_l=-pid_calc(&chassis.motor_balance_wheel[0].speed_p,
//                           chassis.motor_balance_wheel[0].motor_measure->speed_rpm,
//                           0);
//
//   chassis.motor_balance_wheel[0].give_current= pid_calc_balance(&chassis.Vertical_pid,Eulr.pitch,Vertical_in_l,INS_gyro[1]);
//
//    Vertical_in_r=pid_calc(&chassis.motor_balance_wheel[1].speed_p,
//                           chassis.motor_balance_wheel[1].motor_measure->speed_rpm,
//                           0);
//
//   chassis.motor_balance_wheel[1].give_current= -pid_calc_balance(&chassis.Vertical_pid,Eulr.pitch,Vertical_in_r,INS_gyro[1]);
//
//    if(Eulr.pitch>35||Eulr.pitch<-35){
//        chassis.motor_balance_wheel[0].give_current=pid_calc(&chassis.motor_balance_wheel[0].speed_p,
//                                                             chassis.motor_balance_wheel[0].motor_measure->speed_rpm,
//                                                             0);
//
//        chassis.motor_balance_wheel[1].give_current=pid_calc(&chassis.motor_balance_wheel[1].speed_p,
//                                                             chassis.motor_balance_wheel[1].motor_measure->speed_rpm,
//                                                             0);
//
//    }
    balance_out= pid_calc_balance(&chassis.Vertical_pid,-Eulr.pitch,0,-INS_gyro[1]);
    velocity_out= 0;
//            pid_calc(&chassis.motor_balance_wheel[0].speed_p,-chassis.motor_balance_wheel[0].motor_measure->speed_rpm
//                                    +chassis.motor_balance_wheel[1].motor_measure->speed_rpm,0);
    sum_out=balance_out-chassis.Vertical_pid.p*velocity_out;
    chassis.motor_balance_wheel[0].give_current=-sum_out*1.2;
    chassis.motor_balance_wheel[1].give_current=sum_out;

    if(Eulr.pitch>35||Eulr.pitch<-35){
        chassis.motor_balance_wheel[0].give_current=pid_calc(&chassis.motor_balance_wheel[0].speed_p,
                                                             chassis.motor_balance_wheel[0].motor_measure->speed_rpm,
                                                             0);

        chassis.motor_balance_wheel[1].give_current=pid_calc(&chassis.motor_balance_wheel[1].speed_p,
                                                             chassis.motor_balance_wheel[1].motor_measure->speed_rpm,
                                                             0);

    }
#endif
}
