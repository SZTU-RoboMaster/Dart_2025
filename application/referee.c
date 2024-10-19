#include "Referee.h"
#include "string.h"
#include "CRC8_CRC16.h"
#include "bsp_usart.h"
#include "cmsis_os.h"
#include "Gimbal.h"
#include "Chassis.h"
#include "Detection.h"
#include "launcher.h"
#include "key_board.h"

extern UART_HandleTypeDef huart6;//
extern gimbal_t gimbal;//获取云台模式
extern chassis_t chassis;//获取底盘模式
extern Key_board_t KeyBoard;//获取键盘信息
extern launcher_t launcher;//获取发射机构信息
extern int32_t cap_percentage;//电容百分比，在can_receive.c文件中可见
ext_ui_color uiColor;//判断ui颜色
Graphic_Operate update_flag = UI_ADD;
Graphic_Operate cir_cover_update_flag = UI_ADD;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart1;


Graphic_Operate static_update_flag=UI_ADD;
Graphic_Operate one_layer_update_flag=UI_ADD;
Graphic_Operate two_layer_update_flag=UI_ADD;
Graphic_Operate three_layer_update_flag=UI_ADD;

uint8_t usart6_buf[REFEREE_BUFFER_SIZE]={0};  //缓存从串口接受的数据
uint8_t usart1_buf[REFEREE_BUFFER_SIZE]={0};

Referee_info_t Referee;


/*
 * UI更新状态
 */
ui_robot_status_t ui_robot_status={

        .static_update=true,
        .gimbal_mode=GIMBAL_RELAX,
        .chassis_mode=CHASSIS_RELAX,
        .block_warning=false,
        .super_cap_value=0.f,
        .shoot_heat_limit=0,

};

/*函数和声明*/
static void referee_unpack_fifo_data(void);
static bool_t Referee_read_data(uint8_t *ReadFromUsart);
static void ui_static_draw();
/*裁判系统主任务*/

extern fp32 INS_angle[3];

//串口中断函数
void USART6_IRQHandler(void)
{
    static volatile uint8_t res;
    if(USART6->SR & UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart6);//读取UART6-SR 和UART6-DR; 清除中断标志位

        __HAL_DMA_DISABLE(huart6.hdmarx); //使能dma_rx

        Referee_read_data(&usart6_buf[0]);

        memset(&usart6_buf[0],0,REFEREE_BUFFER_SIZE);//置0

        __HAL_DMA_CLEAR_FLAG(huart6.hdmarx,DMA_LISR_TCIF1); //清除传输完成标志位

        __HAL_DMA_SET_COUNTER(huart6.hdmarx, REFEREE_BUFFER_SIZE);//设置DMA 搬运数据大小 单位为字节

        __HAL_DMA_ENABLE(huart6.hdmarx); //使能DMARx

    }
}


//根据裁判系统信息判断机器人的ID和对应客户端的ID
void judge_team_client(){
    //本机器人为红方
    if(Referee.GameRobotStat.robot_id<10)
    {
        Referee.ids.teammate_hero 	   = 1;
        Referee.ids.teammate_engineer  = 2;
        Referee.ids.teammate_infantry3 = 3;
        Referee.ids.teammate_infantry4 = 4;
        Referee.ids.teammate_infantry5 = 5;
        Referee.ids.teammate_plane	   = 6;
        Referee.ids.teammate_sentry		= 7;

        Referee.ids.client_hero 	 = 0x0101;
        Referee.ids.client_engineer  = 0x0102;
        Referee.ids.client_infantry3 = 0x0103;
        Referee.ids.client_infantry4 = 0x0104;
        Referee.ids.client_infantry5 = 0x0105;
        Referee.ids.client_plane	 = 0x0106;

        switch (Referee.GameRobotStat.robot_id) {
            case Referee_hero_red:{
                Referee.SelfClient=Referee.ids.client_hero;
            }break;

            case Referee_engineer_red:{
                Referee.SelfClient=Referee.ids.client_engineer;
            }break;

            case Referee_infantry3_red:{
                Referee.SelfClient=Referee.ids.client_infantry3;
            }break;

            case Referee_infantry4_red:{
                Referee.SelfClient=Referee.ids.client_infantry4;
            }break;

            case Referee_infantry5_red:{
                Referee.SelfClient=Referee.ids.client_infantry5;
            }break;

            case Referee_plane_red:{
                Referee.SelfClient=Referee.ids.client_plane;
            }break;

            default:{

            }break;
        }

    }//本机器人为蓝方
    else{
        Referee.ids.teammate_hero 		 	= 101;
        Referee.ids.teammate_engineer  = 102;
        Referee.ids.teammate_infantry3 = 103;
        Referee.ids.teammate_infantry4 = 104;
        Referee.ids.teammate_infantry5 = 105;
        Referee.ids.teammate_plane		 = 106;
        Referee.ids.teammate_sentry		= 107;

        Referee.ids.client_hero 	 = 0x0165;
        Referee.ids.client_engineer  = 0x0166;
        Referee.ids.client_infantry3 = 0x0167;
        Referee.ids.client_infantry4 = 0x0168;
        Referee.ids.client_infantry5 = 0x0169;
        Referee.ids.client_plane	 = 0x016A;

        switch (Referee.GameRobotStat.robot_id) {
            case Referee_hero_blue:{
                Referee.SelfClient=Referee.ids.client_hero;
            }break;

            case Referee_engineer_blue:{
                Referee.SelfClient=Referee.ids.client_engineer;
            }break;

            case Referee_infantry3_blue:{
                Referee.SelfClient=Referee.ids.client_infantry3;
            }break;

            case Referee_infantry4_blue:{
                Referee.SelfClient=Referee.ids.client_infantry4;
            }break;

            case Referee_infantry5_blue:{
                Referee.SelfClient=Referee.ids.client_infantry5;
            }break;

            case Referee_plane_blue:{
                Referee.SelfClient=Referee.ids.client_plane;
            }break;

            default:{

            }break;
        }

    }
}
float all_rpm_mul_current = 0;
float all_current_pingfang = 0;
float power_nihe = 0;
bool_t Referee_read_data(uint8_t *ReadFromUsart)
{
    int CmdID=0;//数据命令码解析

    uint16_t judge_length;
    Referee.RobotHurt.being_hurt = false;
    if(ReadFromUsart==NULL)
        return 0 ;

    memcpy(&Referee.FrameHeader,ReadFromUsart,Referee_LEN_FRAME_HEAD);

    if(ReadFromUsart[SOF]==REFREE_HEADER_SOF) //判断帧头是否为0xA5
    {
        if(verify_CRC8_check_sum(ReadFromUsart,LEN_HEADER)) //CRC 帧头校验
        {
            judge_length=ReadFromUsart[DATA_LENGTH]+LEN_HEADER+Referee_LEN_CMD_ID+Referee_LEN_FRAME_TAIL;
            if(verify_CRC16_check_sum(ReadFromUsart,judge_length))  //帧尾校验
            {
//                retval_tf=1;//表示数据可用
                CmdID = (ReadFromUsart[6] << 8 | ReadFromUsart[5]);//解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)

                switch (CmdID)
                {

                    case Referee_ID_game_state://0x0001 比赛状态 1HZ
                        memcpy(&Referee.GameState,ReadFromUsart+DATA,Referee_LEN_game_state);
                        break;

                    case Referee_ID_game_result://0x0002 比赛结果   比赛结束后发送
                        memcpy(&Referee.GameResult,ReadFromUsart+DATA,Referee_LEN_game_result);
                        Referee.GameResult.game_over = true;
                        break;

                    case Referee_ID_game_robot_HP://0x0003 机器人状态HP   1HZ
                        memcpy(&Referee.GameRobotHP,ReadFromUsart+DATA,Referee_LEN_game_robot_HP);
                        break;

//V1.6.1删除
//                    case Referee_ID_game_dart_state: //0x0004 飞镖发射状态
//                        memcpy(&Referee.GameDartStatus,ReadFromUsart+DATA,Referee_LED_game_missile_state);
//                        break;
//
//                    case Referee_ID_game_buff: //0x0005 ICRA_BUFF状态     1HZ
//                        memcpy(&Referee.GameICRABuff,ReadFromUsart+DATA,Referee_LED_game_buff);
//                        break;

                    case Referee_ID_event_data://0x0101 场地事件数据      1HZ
                        memcpy(&Referee.EventData,ReadFromUsart+DATA,Referee_LEN_event_data);
                        break;

                    case Referee_ID_supply_projectile_action://0x0102 场地补给站动作标识数据   动作改变之后发送
                        memcpy(&Referee.SupplyProjectileAction,ReadFromUsart+DATA,Referee_LEN_supply_projectile_action);
                        break;

                    case Referee_ID_supply_warm://0x0104    裁判系统警告数据    己方警告之后发送
                        memcpy(&Referee.RefereeWarning,ReadFromUsart+DATA,Referee_LEN_supply_warm);
                        break;

                    case Referee_ID_dart_info://0x0105    飞镖发射口倒计时    1HZ
                        memcpy(&Referee.DartRemainingTime,ReadFromUsart+DATA,Referee_LEN_dart_info);
                        break;

                    case Referee_ID_game_robot_state://0x0201   机器人状态数据     10HZ
                        memcpy(&Referee.GameRobotStat,ReadFromUsart+DATA,Referee_LEN_game_robot_state);
                        judge_team_client();//判断一下机器人所属的队伍和类型 以及对应的机械人id和客户端id
                        break;

                    case Referee_ID_power_heat_data://0x0202    实时功率热量数据    50HZ
                        memcpy(&Referee.PowerHeatData,ReadFromUsart+DATA,Referee_LEN_power_heat_data);

                        break;

                    case Referee_ID_game_robot_pos://0x0203     机器人位置数据     10HZ
                        memcpy(&Referee.GameRobotPos,ReadFromUsart+DATA,Referee_LEN_game_robot_pos);
                        break;

                    case Referee_ID_buff_musk://0x0204  机器人增益数据     1HZ
                        memcpy(&Referee.Buff,ReadFromUsart+DATA,Referee_LEN_buff_musk);
                        break;

                    case Referee_ID_aerial_robot_energy://0x0205    空中机器人能量状态数据 10HZ
                        memcpy(&Referee.AerialRobotEnergy,ReadFromUsart+DATA,Referee_LEN_aerial_robot_energy);
                        break;

                    case Referee_ID_robot_hurt://0x0206     伤害状态数据  伤害发生后发送
                        memcpy(&Referee.RobotHurt,ReadFromUsart+DATA,Referee_LEN_robot_hurt);
                        Referee.RobotHurt.being_hurt = true;//受击判断
                        break;

                    case Referee_ID_shoot_data://0x0207     实时射击数据  射击后发送
                        memcpy(&Referee.ShootData,ReadFromUsart+DATA,Referee_LEN_shoot_data);
                        break;

                    case Referee_ID_bullet_remaining://0x0208   剩余发射数   10HZ周期发送
                        memcpy(&Referee.BulletRemaining,ReadFromUsart+DATA,Referee_LEN_bullet_remaining);
                        break;

                    case Referee_ID_rfid_status://0x0209    机器人RFID状态，1Hz
                        memcpy(&Referee.RfidStatus,ReadFromUsart+DATA,Referee_LEN_rfid_status);
                        break;

                    case Referee_ID_dart_client_directive://0x020A  飞镖机器人客户端指令书, 10Hz
                        memcpy(&Referee.DartClient,ReadFromUsart+DATA,Referee_LEN_dart_client_directive);
                        break;

                    case Referee_ID_dart_all_robot_position://0x020B
                        memcpy(&Referee.RobotPosition,ReadFromUsart+DATA,Referee_LEN_dart_all_robot_position);
                        break;

                    case Referee_ID_radar_mark://0x020C
                        memcpy(&Referee.RadarMark,ReadFromUsart+DATA,Referee_LEN_radar_mark);
                        break;

                    case Referee_ID_entry_info://0x020D
                        memcpy(&Referee.SentryInfo,ReadFromUsart+DATA,Referee_LEN_entry_info);
                        break;

                    case Referee_ID_radar_info://0x020E
                        memcpy(&Referee.RadarInfo,ReadFromUsart+DATA,Referee_LEN_radar_info);
                        break;

                    case Referee_ID_robot_interactive_header_data://0x0301
                        memcpy(&Referee.StudentInteractive,ReadFromUsart+DATA,Referee_LEN_robot_interactive_header_data);
                        break;

                    case Referee_ID_map_command://0x0303
                        memcpy(&Referee.MapCommand,ReadFromUsart+DATA,Referee_LEN_map_command);
                        break;

                    case Referee_ID_keyboard_information://0x0304
                        memcpy(&Referee.keyboard,ReadFromUsart+DATA,Referee_LEN_keyboard_information);
                        break;

                    case Referee_ID_robot_map_robot_data://0x0305
                        memcpy(&Referee.EnemyPosition,ReadFromUsart+DATA,Referee_LEN_robot_map_robot_data);
                        break;

                    case Referee_ID_robot_custom_client://0x0306
                        memcpy(&Referee.Custom,ReadFromUsart+DATA,Referee_LEN_robot_custom_client);
                        break;

                    case Referee_ID_robot_entry_info_receive://0x0307
                        memcpy(&Referee.SentryMapData,ReadFromUsart+DATA,Referee_LEN_robot_entry_info_receive);
                        break;

                    case Referee_ID_robot_custom_info_receive://0x0308
                        memcpy(&Referee.SendData,ReadFromUsart+DATA,Referee_LEN_robot_custom_info_receive);
                        break;

                    default:
                        break;
                }
                detect_handle(DETECT_REFEREE);
            }
        }
        if(*(ReadFromUsart + sizeof(frame_header_struct_t) + Referee_LEN_CMD_ID + Referee.FrameHeader.data_length +Referee_LEN_FRAME_TAIL) == 0xA5)
        {
            //如果一个数据包出现了多帧数据,则再次读取
            Referee_read_data(ReadFromUsart + sizeof(frame_header_struct_t) + Referee_LEN_CMD_ID + Referee.FrameHeader.data_length+ Referee_LEN_FRAME_TAIL);
        }
    }
}


//绘制变量
uint8_t state_first_graphic;//0~7循环 更新的图层数
uint8_t ClientTxBuffer[200];//发送给客户端的数据缓冲区
uint8_t ClientTxCapBuffer[200];//电容数据发送的缓存区
uint8_t ClientTxBufferChar[200];//字符型静态元素缓存区
uint8_t ClientTxBufferRect[200];//自瞄方框提示
uint8_t ClientTXBufferCir[200];//陀螺及弹舱提示
//绘制数据
//第0层画的字符串 字符串最长只能 30 Byte
/**************************************/
/**
 * 绘制字符串
 * @param graphic
 * @param name
 * @param op_type
 * @param layer
 * @param color
 * @param size
 * @param length
 * @param width
 * @param start_x
 * @param start_y
 * @param character
 */
void String_Graphic(ui_string_t*clientData,
                    const char* name,
                    uint32_t op_type,
                    uint32_t layer,
                    uint32_t color,
                    uint32_t size,
                    uint32_t length,
                    uint32_t width,
                    uint32_t start_x,
                    uint32_t start_y,
                    const char *character)// 数组
{
    ui_graphic_data_struct_t*data_struct=&clientData->graphic_data_struct;
    data_struct->graphic_tpye=UI_CHAR;

    for(char i=0;i<3;i++)
        data_struct->graphic_name[i] = name[i];	//字符索引
    data_struct->operate_tpye=op_type;// 图层操作  1为增加
    data_struct->layer=layer;//在第几图层
    data_struct->color=color;//颜色
    data_struct->start_angle=size;
    data_struct->end_angle=length;
    data_struct->width=width;
    data_struct->start_x=start_x;
    data_struct->start_y=start_y;
    data_struct->radius = 0;
    data_struct->end_x = 0;
    data_struct->end_y = 0;

    memcpy(clientData->data,character,30);
}

/**
 * 绘制腹图像
 * @param graphic
 * @param name
 * @param operate_tpye
 * @param graphic_tpye
 * @param layer
 * @param color
 * @param start_angle
 * @param end_angle
 * @param width
 * @param start_x
 * @param start_y
 * @param radius
 * @param end_x
 * @param end_y
 */
void Figure_Graphic(ui_graphic_data_struct_t* graphic,//最终要发出去的数组的数据段内容
                    const char* name,
                    uint32_t operate_tpye,
                    uint32_t graphic_tpye,//绘制什么图像
                    uint32_t layer,
                    uint32_t color,
                    uint32_t start_angle,
                    uint32_t end_angle,
                    uint32_t width,
                    uint32_t start_x,
                    uint32_t start_y,
                    uint32_t radius,
                    uint32_t end_x,
                    uint32_t end_y)
{
    for(char i=0;i<3;i++)
        graphic->graphic_name[i] = name[i];	//字符索引
    graphic->operate_tpye = operate_tpye; //图层操作
    graphic->graphic_tpye = graphic_tpye;         //Char型
    graphic->layer        = layer;//都在第一层
    graphic->color        = color;//变色
    graphic->start_angle  = start_angle;
    graphic->end_angle    = end_angle;
    graphic->width        = width;
    graphic->start_x      = start_x;
    graphic->start_y      = start_y;
    graphic->radius = radius;
    graphic->end_x  = end_x;
    graphic->end_y  = end_y;
}

/**
 * 绘制浮点数
 * @param graphic
 * @param name
 * @param operate_tpye
 * @param graphic_tpye
 * @param layer
 * @param color
 * @param size
 * @param decimal
 * @param width
 * @param start_x
 * @param start_y
 * @param number
 */
void Float_Graphic(ui_graphic_data_struct_t* graphic,//最终要发出去的数组的数据段内容
                   const char* name,
                   uint32_t operate_tpye,
                   uint32_t graphic_tpye,//绘制什么图像
                   uint32_t layer,
                   uint32_t color,
                   uint32_t size,
                   uint32_t decimal,
                   uint32_t width,
                   uint32_t start_x,
                   uint32_t start_y,
                   float number)
{
    for(char i=0;i<3;i++)
        graphic->graphic_name[i] = name[i];	//字符索引
    graphic->operate_tpye = operate_tpye; //图层操作
    graphic->graphic_tpye = graphic_tpye;
    graphic->layer        = layer;//
    graphic->color        = color;//变色
    graphic->start_angle  = size;
    graphic->end_angle    = decimal;//小数有效位
    graphic->width        = width;
    graphic->start_x      = start_x;
    graphic->start_y      = start_y;
    graphic->number       = number*1000;//浮点类型的要成1000后转换为一个int32类型的
}

/**
 * 绘制整形
 * @param graphic
 * @param name
 * @param operate_tpye
 * @param graphic_tpye
 * @param layer
 * @param color
 * @param size
 * @param zero
 * @param width
 * @param start_x
 * @param start_y
 * @param number
 */
void Int_Graphic(ui_graphic_data_struct_t* graphic,//最终要发出去的数组的数据段内容
                 const char* name,
                 uint32_t operate_tpye,
                 uint32_t graphic_tpye,//绘制什么图像
                 uint32_t layer,
                 uint32_t color,
                 uint32_t size,
                 uint32_t zero,
                 uint32_t width,
                 uint32_t start_x,
                 uint32_t start_y,
                 int32_t number)
{
    for(char i=0;i<3;i++)
        graphic->graphic_name[i] = name[i];	//字符索引
    graphic->operate_tpye = operate_tpye; //图层操作
    graphic->graphic_tpye = graphic_tpye;
    graphic->layer        = layer;//都在第一层
    graphic->color        = color;//变色
    graphic->start_angle  = size;
    graphic->end_angle    = zero;
    graphic->width        = width;
    graphic->start_x      = start_x;
    graphic->start_y      = start_y;
    graphic->number       = number;
}

void one_layer_draw()
{
    ext_graphic_seven_data_t ui_aim_line;//发送变量,七个数据段

    //裁判通信帧头处理
    ui_aim_line.txFrameHeader.SOF=REFREE_HEADER_SOF;
    ui_aim_line.txFrameHeader.data_length=sizeof (ext_student_interactive_header_data_t)+
                                          sizeof(ui_graphic_data_struct_t)*7;
    ui_aim_line.txFrameHeader.seq=0;
    memcpy(ClientTxBuffer,&ui_aim_line.txFrameHeader,sizeof(frame_header_struct_t));
    //CRC8
    append_CRC8_check_sum(ClientTxBuffer,sizeof(frame_header_struct_t));
    ui_aim_line.CmdID= Referee_ID_robot_interactive_header_data;

    //数据帧头
    ui_aim_line.dataFrameHeader.send_ID=Referee.GameRobotStat.robot_id;
    ui_aim_line.dataFrameHeader.receiver_ID=Referee.SelfClient;
    ui_aim_line.dataFrameHeader.data_cmd_id=UI_INTERACT_ID_draw_seven_graphic;

    //数据内容
    //将画图的内容赋值到的xxx_data_struct中
    Figure_Graphic(&ui_aim_line.clientData[0],"LI1",static_update_flag,UI_LINE,UI_ONE_LAYER,UI_YELLOW,
                   0,0,2,1920/2-80,1080/2-80,
                   0,1920/2+80,1080/2-80);
    Figure_Graphic(&ui_aim_line.clientData[1],"LI2",static_update_flag,UI_LINE,
                   UI_ONE_LAYER,UI_YELLOW,0,0,2,1920/2-60,1080/2-120,
                   0,1920/2+60,1080/2-120);
    Figure_Graphic(&ui_aim_line.clientData[2],"LI3",static_update_flag,UI_LINE,
                   UI_ONE_LAYER,UI_YELLOW,0,0,2,1920/2-40,1080/2-160,
                   0,1920/2+40,1080/2-160);
    Figure_Graphic(&ui_aim_line.clientData[3],"LI4",static_update_flag,UI_LINE,
                   UI_ONE_LAYER,UI_YELLOW,0,0,2,1920/2-20,1080/2-200,
                   0,1920/2+20,1080/2-200);
    Figure_Graphic(&ui_aim_line.clientData[4],"LI5",static_update_flag,UI_LINE,
                   UI_ONE_LAYER,UI_YELLOW,0,0,2,1920/2-10,1080/2-240,
                   0,1920/2+10,1080/2-240);
    Figure_Graphic(&ui_aim_line.clientData[5],"LI6",static_update_flag,UI_LINE,
                   UI_ONE_LAYER,UI_YELLOW,0,0,2,1920/2,1080/2-40,
                   0,1920/2,1080/2-240);
    Figure_Graphic(&ui_aim_line.clientData[6],"LI7",static_update_flag,UI_LINE,
                   UI_ONE_LAYER,UI_YELLOW,0,0,2,1920/2-100,1080/2-40,
                   0,1920/2+100,1080/2-40);

    //将除帧头部分放入缓冲区
    memcpy(ClientTxBuffer+Referee_LEN_FRAME_HEAD,(uint8_t *)&ui_aim_line.CmdID,sizeof(ui_aim_line));
    //帧尾CRC16处理
    append_CRC16_check_sum(ClientTxBuffer,sizeof(ui_aim_line));
    //串口发送
    usart6_tx_dma_enable(ClientTxBuffer,sizeof(ui_aim_line));
}



void cap_percentage_draw_init()
{
    ext_graphic_two_data_t ui_cap_percentage;//两个数据段
    //帧头处理
    ui_cap_percentage.txFrameHeader.SOF=REFREE_HEADER_SOF;
    ui_cap_percentage.txFrameHeader.data_length=sizeof (ext_student_interactive_header_data_t)+
                                                sizeof(ui_graphic_data_struct_t)*2;
    ui_cap_percentage.txFrameHeader.seq=0;

    memcpy(ClientTxCapBuffer,&ui_cap_percentage.txFrameHeader,sizeof(frame_header_struct_t));
    //CRC8校验
    append_CRC8_check_sum(ClientTxCapBuffer,sizeof(frame_header_struct_t));
    //数据帧头处理
    ui_cap_percentage.CmdID= Referee_ID_robot_interactive_header_data;
    ui_cap_percentage.dataFrameHeader.send_ID=Referee.GameRobotStat.robot_id;
    ui_cap_percentage.dataFrameHeader.receiver_ID=Referee.SelfClient;
    ui_cap_percentage.dataFrameHeader.data_cmd_id=UI_INTERACT_ID_draw_two_graphic;
    //数据填充
    //能量边框
    Figure_Graphic(&ui_cap_percentage.clientData[0],"CFL",UI_ADD,UI_RECTANGLE,
                   UI_FOUR_LAYER,UI_WHITE,0,0,10,834,109,0, 1059, 143);//放在第四图层
    //能量条,初始状态是满的
    Figure_Graphic(&ui_cap_percentage.clientData[1],"CAC",UI_ADD,UI_LINE,
                   UI_FOUR_LAYER,UI_GREEN,0,0, 20,839,125,0,1055,125);
    //CRC18校验
    memcpy(ClientTxCapBuffer+Referee_LEN_FRAME_HEAD,(uint8_t *)&ui_cap_percentage.CmdID,sizeof(ui_cap_percentage));
    append_CRC16_check_sum(ClientTxCapBuffer,sizeof(ui_cap_percentage));
    usart6_tx_dma_enable(ClientTxCapBuffer,sizeof(ui_cap_percentage));
    osDelay(100);
}
float asc=0.0;
float ant = 0;
void dynamic_cap_percentage_draw()
{
    ext_graphic_two_data_t ui_cap_percentage;//两个数据段
    //帧头处理
    ui_cap_percentage.txFrameHeader.SOF=REFREE_HEADER_SOF;
    ui_cap_percentage.txFrameHeader.data_length=sizeof (ext_student_interactive_header_data_t)+
                                                sizeof(ui_graphic_data_struct_t)*2;
    ui_cap_percentage.txFrameHeader.seq=0;

    memcpy(ClientTxCapBuffer,&ui_cap_percentage.txFrameHeader,sizeof(frame_header_struct_t));
    //CRC8校验
    append_CRC8_check_sum(ClientTxCapBuffer,sizeof(frame_header_struct_t));
    //数据帧头处理
    ui_cap_percentage.CmdID= Referee_ID_robot_interactive_header_data;
    ui_cap_percentage.dataFrameHeader.send_ID=Referee.GameRobotStat.robot_id;
    ui_cap_percentage.dataFrameHeader.receiver_ID=Referee.SelfClient;
    ui_cap_percentage.dataFrameHeader.data_cmd_id=UI_INTERACT_ID_draw_two_graphic;
    //数据填充
//    int32_t real_cap = cap_percentage * 216;

    asc+=0.1;
    if(asc>=1){
        asc=0;
    }
    float real_cap=asc*216;
//    int32_t real_cap=((HAL_GetTick()%100)/100.0)*216;

    //等待修改
    ant += 0.1;
    if(ant == 216)
        ant = 0;
//    float real_cap = cap_percentage * 216;
    //等待修改
    //能量边框,黄色边框代表动态模式启动了
    Figure_Graphic(&ui_cap_percentage.clientData[0],"CFL",UI_MODIFY,UI_RECTANGLE,
                   UI_FOUR_LAYER,UI_YELLOW,0,0,10,834,109,0, 1059, 143);//放在第四图层
    //能量条,初始状态是满的
    Figure_Graphic(&ui_cap_percentage.clientData[1],"CAC",UI_MODIFY,UI_LINE,
                   UI_FOUR_LAYER,UI_GREEN,0,0, 20,839,125,0,839 + ant, 125);
    //CRC18校验
    memcpy(ClientTxCapBuffer+Referee_LEN_FRAME_HEAD,(uint8_t *)&ui_cap_percentage.CmdID,sizeof(ui_cap_percentage));
    append_CRC16_check_sum(ClientTxCapBuffer,sizeof(ui_cap_percentage));
    usart6_tx_dma_enable(ClientTxCapBuffer,sizeof(ui_cap_percentage));
    osDelay(100);
}
extern bool magazine_cover_is_closed;
//动态元素，提示颜色转换


void ui_aim_draw()
{
    ext_graphic_seven_data_t ui_aim;//三根横线，一根竖线，加一个圆，一共五个，后续添加：十字准心
    //裁判通信帧头
    ui_aim.txFrameHeader.SOF = REFREE_HEADER_SOF;
    ui_aim.txFrameHeader.data_length = sizeof (ext_student_interactive_header_data_t) +
                                       sizeof (ui_graphic_data_struct_t) * 7;
    ui_aim.txFrameHeader.seq = 0;//包序号设置为0
    memcpy(ClientTxBuffer, &ui_aim.txFrameHeader, sizeof (frame_header_struct_t));//把帧头放进去
    //CRC8校验帧头
    append_CRC8_check_sum(ClientTxBuffer, sizeof (frame_header_struct_t));
    ui_aim.CmdID = Referee_ID_robot_interactive_header_data;
    //数据帧头
    ui_aim.dataFrameHeader.send_ID = Referee.GameRobotStat.robot_id;
    ui_aim.dataFrameHeader.receiver_ID = Referee.SelfClient;
    ui_aim.dataFrameHeader.data_cmd_id = UI_INTERACT_ID_draw_seven_graphic;
    //数据内容填充
    //竖线
    Figure_Graphic(&ui_aim.clientData[0], "LI1", UI_ADD, UI_LINE, UI_ZERO_LAYER, UI_YELLOW,
                   0, 0, 2, 959, 347, 0, 961, 428);
    //横线1
    Figure_Graphic(&ui_aim.clientData[1], "LI2", UI_ADD, UI_LINE, UI_ZERO_LAYER, UI_YELLOW,
                   0, 0, 2, 911, 396, 0, 1013, 398);
    //横线2
    Figure_Graphic(&ui_aim.clientData[2], "LI3", UI_ADD, UI_LINE, UI_ZERO_LAYER, UI_YELLOW,
                   0, 0, 2, 932, 376, 0, 992, 376);
    //横线3
    Figure_Graphic(&ui_aim.clientData[3], "LI4", UI_ADD, UI_LINE, UI_ZERO_LAYER, UI_ORANGE,
                   0, 0, 3, 945, 353, 0, 976, 353);
    //圆型
    Figure_Graphic(&ui_aim.clientData[4], "LI5", UI_ADD,  UI_CIRCLE, UI_ZERO_LAYER, UI_GREEN,
                   0, 0, 5, 959, 455, 50, 0, 0);
    //十字准心
    Figure_Graphic(&ui_aim.clientData[5], "LI5", UI_ADD, UI_LINE, UI_ZERO_LAYER, UI_GREEN,
                   0, 0, 2, 949, 458, 0, 973, 458);//横线
    Figure_Graphic(&ui_aim.clientData[6], "LI6", UI_ADD, UI_LINE, UI_ZERO_LAYER, UI_GREEN,
                   0, 0, 2, 961, 445, 0, 961, 471);//竖线
    //把除去帧头的其他部分放进缓存区
    memcpy(ClientTxBuffer + Referee_LEN_FRAME_HEAD, (uint8_t*)&ui_aim.CmdID, sizeof (ui_aim));
    //帧尾使用CRC16处理
    append_CRC16_check_sum(ClientTxBuffer, sizeof (ui_aim));
    //串口6发送
    usart6_tx_dma_enable(ClientTxBuffer, sizeof (ui_aim));
    osDelay(100);
}

static void draw_static_string(ext_string_data_t* ui_string)
{

    switch (state_first_graphic)
    {
        case 0:
        {
            char first_line[30]  = {"CHASSIS:"};//
            String_Graphic(&ui_string->clientData, "CL1", static_update_flag, UI_ZERO_LAYER, UI_PINK, 15, strlen(first_line), 2, 320,
                           620, first_line);
        }
            break;
        case 1:
        {
            char second_line[30] = {" GIMBAL:"};//云台模式
            String_Graphic(&ui_string->clientData, "CL2", static_update_flag, UI_ZERO_LAYER, UI_PINK, 15, strlen(second_line), 2, 320,
                           680, second_line);
            break;
        }
        case 2:
        {
            char third_line[30]={"   FIRE:"};
            String_Graphic(&ui_string->clientData, "CL3", static_update_flag, UI_ZERO_LAYER, UI_PINK, 15, strlen(third_line), 2, 320,
                           740, third_line);
            break;
        }
        case 3:
        {
            char fourth_line[30] = {"    LIP:"};
            String_Graphic(&ui_string->clientData, "CL4", static_update_flag, UI_ZERO_LAYER, UI_PINK, 15, strlen(fourth_line), 2, 320,
                           800, fourth_line);
            break;
        }
        case 4:
        {
            char cap_line[30]={"CAP:"};
            String_Graphic(&ui_string->clientData,"CAP",static_update_flag,UI_ZERO_LAYER,UI_ORANGE,15, strlen(cap_line),2,1920-550,
                           400,cap_line);
            break;
        }
        default:
            break;
    }
}

void ui_string_draw()
{
    ext_string_data_t ui_string;//选对结构体
    int draw_time = 8;
    //裁判系统通信帧头
    ui_string.txFrameHeader.SOF = REFREE_HEADER_SOF;
    ui_string.txFrameHeader.data_length = UI_LEN_INTERACT_draw_char_graphic;//帧头长度
    ui_string.txFrameHeader.seq = 0;//包序号
    memcpy(ClientTxBufferChar, &ui_string.txFrameHeader, sizeof (frame_header_struct_t));//帧头放入
    //CRC8校验
    append_CRC8_check_sum(ClientTxBufferChar, sizeof (frame_header_struct_t));
    ui_string.CmdID = Referee_ID_robot_interactive_header_data;
    //数据帧头
    ui_string.dataFrameHeader.send_ID = Referee.GameRobotStat.robot_id;
    ui_string.dataFrameHeader.receiver_ID = Referee.SelfClient;
    ui_string.dataFrameHeader.data_cmd_id = UI_INTERACT_ID_draw_char_graphic;
    //数据内容填充,循环填充并发送
    while(draw_time-- >= 0)
    {
        switch (draw_time)
        {
            case 0:
            {

                char cover[30] = {"SPIN"};//陀螺字符提示
                String_Graphic(&ui_string.clientData, "CO1", UI_ADD, UI_ONE_LAYER, UI_CYAN_BLUE, 15, strlen(cover),
                               2, 382, 775, cover);
            }
                break;

            case 1:
            {
                char spin[30] = {"COVER"};//弹舱字符提示
                String_Graphic(&ui_string.clientData, "SP1", UI_ADD, UI_ONE_LAYER, UI_CYAN_BLUE, 15, strlen(spin),
                               2, 366, 648, spin);
            }
                break;

            case 3:
            {
                char Sauto[30] = {"AUTO"};//自瞄模式,字符呈现紫红色
                String_Graphic(&ui_string.clientData, "BU1", UI_ADD, UI_ONE_LAYER, UI_FUCHSIA, 13, strlen(Sauto),
                               3, 707, 747, Sauto);//字符提示
            }
                break;

            case 4:
            {
                char sbuff[30] = {"SBUFF"};//打符，大符，红蓝配色
                String_Graphic(&ui_string.clientData, "BU2", UI_ADD, UI_ONE_LAYER, UI_RED_BLUE, 13, strlen(sbuff),
                               3, 844, 747, sbuff);//字符提示
            }
                break;

            case 5:
            {
                char buff[30] = {"BUFF"};//小符，兰青配色
                String_Graphic(&ui_string.clientData, "BU3", UI_ADD, UI_ONE_LAYER, UI_CYAN_BLUE, 13, strlen(buff),
                               3, 966, 747, buff);//字符提示
            }
                break;

            case 6:
            {
                char fire[30] = {"FIRE"};//摩擦轮提示
                String_Graphic(&ui_string.clientData, "FIR", UI_ADD, UI_ONE_LAYER, UI_CYAN_BLUE, 15, strlen(fire),
                               3, 500, 848, fire);
            }

            default:
                break;
        }
        //除去帧头部分放入缓存区
        memcpy(ClientTxBufferChar + Referee_LEN_FRAME_HEAD, (uint8_t*)&ui_string.CmdID, sizeof (ui_string));
        //帧尾CRC16处理
        append_CRC16_check_sum(ClientTxBufferChar, sizeof (ui_string));
        //发送
        usart6_tx_dma_enable(ClientTxBufferChar, sizeof (ui_string));
        osDelay(100);
    }
}

void ui_auto_aim_fire_init()
{
    ext_graphic_two_data_t ui_auto;//一次发两个

    //裁判系统帧头
    ui_auto.txFrameHeader.SOF = REFREE_HEADER_SOF;
    ui_auto.txFrameHeader.data_length = sizeof(ext_student_interactive_header_data_t) +
                                        sizeof (ui_graphic_data_struct_t) * 2;
    ui_auto.txFrameHeader.seq = 0;//包序号
    memcpy(ClientTxBufferRect, &ui_auto.txFrameHeader, sizeof (frame_header_struct_t));//将帧头放入
    //CRC8校验
    append_CRC8_check_sum(ClientTxBufferRect, sizeof (frame_header_struct_t));
    ui_auto.CmdID = Referee_ID_robot_interactive_header_data;
    //数据帧头
    ui_auto.dataFrameHeader.send_ID = Referee.GameRobotStat.robot_id;
    ui_auto.dataFrameHeader.receiver_ID = Referee.SelfClient;
    ui_auto.dataFrameHeader.data_cmd_id = UI_INTERACT_ID_draw_two_graphic;//绘制两个图形
    //数据填充
    //第一是提示方框
    Figure_Graphic(&ui_auto.clientData[0], "RC1", UI_ADD, UI_RECTANGLE, UI_TWO_LAYER, UI_GREEN,
                   0, 0, 2, 687, 264, 0, 1230, 709);//方框提示
    //第二是摩擦轮圆圈
    Figure_Graphic(&ui_auto.clientData[1], "FI1", UI_ADD, UI_CIRCLE, UI_TWO_LAYER, UI_CYAN_BLUE,
                   0, 0, 6, 632, 848, 42, 0, 0);//摩擦轮圆圈提示

    //去除帧头部分，其他放入缓存区
    memcpy(ClientTxBufferRect + Referee_LEN_FRAME_HEAD, (uint8_t*)&ui_auto.CmdID, sizeof (ui_auto));
    //CRC16校验
    append_CRC16_check_sum(ClientTxBufferRect, sizeof(ui_auto));
    //串口发送
    usart6_tx_dma_enable(ClientTxBufferRect, sizeof(ui_auto));
    osDelay(100);
}

void dynamic_auto_fire_draw()
{
    ext_graphic_two_data_t ui_auto;
    //裁判系统帧头
    ui_auto.txFrameHeader.SOF = REFREE_HEADER_SOF;
    ui_auto.txFrameHeader.data_length = sizeof(ext_student_interactive_header_data_t) +
                                        sizeof (ui_graphic_data_struct_t) * 2;
    ui_auto.txFrameHeader.seq = 0;//包序号
    memcpy(ClientTxBufferRect, &ui_auto.txFrameHeader, sizeof (frame_header_struct_t));//将帧头放入
    //CRC8校验
    append_CRC8_check_sum(ClientTxBufferRect, sizeof (frame_header_struct_t));
    ui_auto.CmdID = Referee_ID_robot_interactive_header_data;
    //数据帧头
    ui_auto.dataFrameHeader.send_ID = Referee.GameRobotStat.robot_id;
    ui_auto.dataFrameHeader.receiver_ID = Referee.SelfClient;
    ui_auto.dataFrameHeader.data_cmd_id = UI_INTERACT_ID_draw_two_graphic;
    //数据填充
    Figure_Graphic(&ui_auto.clientData[0], "RC1", UI_MODIFY, UI_RECTANGLE, UI_TWO_LAYER, uiColor.auto_aim_color,
                   0, 0, 2, 687, 264, 0, 1230, 709);//图形修改
    Figure_Graphic(&ui_auto.clientData[1], "FI1", UI_MODIFY, UI_CIRCLE, UI_TWO_LAYER, uiColor.fire_color,
                   0, 0, 6, 632, 848, 42, 0, 0);//颜色变化
    //去除帧头部分，其他放入缓存区
    memcpy(ClientTxBufferRect + Referee_LEN_FRAME_HEAD, (uint8_t*)&ui_auto.CmdID, sizeof (ui_auto));
    //CRC16校验
    append_CRC16_check_sum(ClientTxBufferRect, sizeof(ui_auto));
    //串口发送
    usart6_tx_dma_enable(ClientTxBufferRect, sizeof(ui_auto));
    osDelay(100);
}

void ui_cover_draw_init()//旋转跟弹舱一起
{
    ext_graphic_two_data_t ui_cover_spin;
    //裁判系统帧头
    ui_cover_spin.txFrameHeader.SOF = REFREE_HEADER_SOF;
    ui_cover_spin.txFrameHeader.data_length = sizeof (ext_student_interactive_header_data_t) +
                                              sizeof(ui_graphic_data_struct_t ) * 2;
    ui_cover_spin.txFrameHeader.seq = 0;//包序号
    memcpy(ClientTXBufferCir, &ui_cover_spin.txFrameHeader, sizeof (frame_header_struct_t));
    //CRC8校验
    append_CRC8_check_sum(ClientTXBufferCir, sizeof(frame_header_struct_t));
    ui_cover_spin.CmdID = Referee_ID_robot_interactive_header_data;
    //数据帧头
    ui_cover_spin.dataFrameHeader.send_ID = Referee.GameRobotStat.robot_id;
    ui_cover_spin.dataFrameHeader.receiver_ID = Referee.SelfClient;
    ui_cover_spin.dataFrameHeader.data_cmd_id = UI_INTERACT_ID_draw_two_graphic;
    //数据填充
    Figure_Graphic(&ui_cover_spin.clientData[0], "CO2", UI_ADD, UI_CIRCLE, UI_THREE_LAYER, UI_CYAN_BLUE,
                   0, 0, 6, 523, 643, 42, 0, 0);
    Figure_Graphic(&ui_cover_spin.clientData[1], "SP2", UI_ADD, UI_CIRCLE, UI_THREE_LAYER, UI_CYAN_BLUE,
                   0, 0, 6, 543, 760, 42, 0, 0);
    //去除帧头部分，其他放入缓存区
    memcpy(ClientTXBufferCir + Referee_LEN_FRAME_HEAD, (uint8_t*)&ui_cover_spin.CmdID, sizeof (ui_cover_spin));
    //CRC16校验
    append_CRC16_check_sum(ClientTXBufferCir, sizeof(ui_cover_spin));
    //串口发送
    usart6_tx_dma_enable(ClientTXBufferCir, sizeof(ui_cover_spin));
    osDelay(100);
}

void dynamic_spin_cover_draw()
{
    ext_graphic_two_data_t ui_cover_spin;
    //裁判系统帧头
    ui_cover_spin.txFrameHeader.SOF = REFREE_HEADER_SOF;
    ui_cover_spin.txFrameHeader.data_length = sizeof (ext_student_interactive_header_data_t) +
                                              sizeof(ui_graphic_data_struct_t ) * 2;
    ui_cover_spin.txFrameHeader.seq = 0;//包序号
    memcpy(ClientTXBufferCir, &ui_cover_spin.txFrameHeader, sizeof (frame_header_struct_t));
    //CRC8校验
    append_CRC8_check_sum(ClientTXBufferCir, sizeof(frame_header_struct_t));
    ui_cover_spin.CmdID = Referee_ID_robot_interactive_header_data;
    //数据帧头
    ui_cover_spin.dataFrameHeader.send_ID = Referee.GameRobotStat.robot_id;
    ui_cover_spin.dataFrameHeader.receiver_ID = Referee.SelfClient;
    ui_cover_spin.dataFrameHeader.data_cmd_id = UI_INTERACT_ID_draw_two_graphic;
    //数据填充
    Figure_Graphic(&ui_cover_spin.clientData[0], "CO2", UI_MODIFY, UI_CIRCLE, UI_THREE_LAYER, uiColor.cover_color,
                   0, 0, 6, 523, 643, 42, 0, 0);
    Figure_Graphic(&ui_cover_spin.clientData[1], "SP2", UI_MODIFY, UI_CIRCLE, UI_THREE_LAYER, uiColor.spin_color,
                   0, 0, 6, 543, 760, 42, 0, 0);
    //去除帧头部分，其他放入缓存区
    memcpy(ClientTXBufferCir + Referee_LEN_FRAME_HEAD, (uint8_t*)&ui_cover_spin.CmdID, sizeof (ui_cover_spin));
    //CRC16校验
    append_CRC16_check_sum(ClientTXBufferCir, sizeof(ui_cover_spin));
    //串口发送
    usart6_tx_dma_enable(ClientTXBufferCir, sizeof(ui_cover_spin));
    osDelay(100);//10Hz发送频率
}

_Noreturn void UI_paint_task(void const*argument)
{
    vTaskDelay(20);
    //osDelay(100);
    while(1)
    {
//        if(KeyBoard.V.status == KEY_CLICK && update_flag == UI_ADD)
//        {//静态元素及其动态元素的初始化,手动跳到动态模式,要等待界面加载完毕再按下
//            update_flag = UI_MODIFY;
//        }
//
//        if(KeyBoard.V.status == KEY_CLICK && update_flag == UI_MODIFY)
//        {//再按一下可以回到加载模式
//            update_flag = UI_ADD;
//        }

        if(KeyBoard.V.click_flag == 0)
        {
            ui_aim_draw();//辅助瞄准线

            ui_auto_aim_fire_init();//自瞄提示和摩擦轮开启提示初始化

            ui_cover_draw_init();//旋转与弹舱提示初始化

            ui_string_draw();//画字符

            cap_percentage_draw_init();//电容百分比显示
        }
        else
        {


            dynamic_auto_fire_draw();//自瞄提示和摩擦轮开启提示

            dynamic_spin_cover_draw();//旋转与弹舱提示

            dynamic_cap_percentage_draw();//电容百分比修改
        }
    }
}