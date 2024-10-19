//
// Created by xhuanc on 2021/12/3.
//

#include "key_board.h"

Key_board_t KeyBoard;
extern RC_ctrl_t rc_ctrl;
static void key_update(Key*key,uint16_t key_status,uint16_t cnt);
void update_pc_info(){

    //根据遥控器更改键鼠的按键状态
    key_update(&KeyBoard.W,rc_ctrl.key.v&KEY_W,DEFAULT_CNT);//DEFAULT_CNT 为按键检测为长按的默认时间
    key_update(&KeyBoard.A,rc_ctrl.key.v&KEY_A,DEFAULT_CNT);
    key_update(&KeyBoard.S,rc_ctrl.key.v&KEY_S,DEFAULT_CNT);
    key_update(&KeyBoard.D,rc_ctrl.key.v&KEY_D,DEFAULT_CNT);

/*    Key_Update(&KeyBoard.SHIFT,rc_ctrl.key.v&KEY_SHIFT);
    Key_Update(&KeyBoard.CTRL,rc_ctrl.key.v&KEY_CTRL);
    Key_Update(&KeyBoard.Q,rc_ctrl.key.v&KEY_Q);
    Key_Update(&KeyBoard.E,rc_ctrl.key.v&KEY_E);
    Key_Update(&KeyBoard.F,rc_ctrl.key.v&KEY_F);
    Key_Update(&KeyBoard.G,rc_ctrl.key.v&KEY_G);
    Key_Update(&KeyBoard.Z,rc_ctrl.key.v&KEY_Z);
    Key_Update(&KeyBoard.X,rc_ctrl.key.v&KEY_X);
    Key_Update(&KeyBoard.C,rc_ctrl.key.v&KEY_C);
    Key_Update(&KeyBoard.V,rc_ctrl.key.v&KEY_V);
    Key_Update(&KeyBoard.B,rc_ctrl.key.v&KEY_B);
    */

    /*鼠标部分*/
    key_update(&KeyBoard.Mouse_l,rc_ctrl.mouse.press_l&MOUSE_YES,MOUSE_CLICK_L_CNT);
    key_update(&KeyBoard.Mouse_r,rc_ctrl.mouse.press_r&MOUSE_YES,MOUSE_CLICK_R_CNT);

}

//根据遥控器更改键鼠的按键状态
static void key_update(Key*key,uint16_t key_status,uint16_t cnt){
    if(key_status)
    {
        key->press_cnt++;
        if (key->status==KEY_RELAX)
        {
            key->last_status=KEY_RELAX;
            key->status=KEY_DOWN;
        } else if(key->status==KEY_DOWN&&(HAL_GetTick()-key->click_cnt)>325)
        {
            key->click_cnt=HAL_GetTick();
            key->last_status=KEY_DOWN;
            key->status=KEY_CLICK;
            if(key->click_flag==1)
            {
                key->click_flag=0;
            }
            else if(key->click_flag==0)
            {
                key->click_flag=1;
            }
        }else{
            key->status=KEY_RELAX;
        }

        if(key->press_cnt>cnt)
        {
            key->last_status=key->status;
            key->status=KEY_PRESS;
        }

    }
    else
    {
        key->last_status=key->status;
        key->status=KEY_RELAX;
        key->press_cnt=0;
    }

}
