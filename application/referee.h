//
// Created by xhuanc on 2021/10/23.
// Update by zxk on 2024/3/12
//

#ifndef _REFEREE_H_
#define _REFEREE_H_
#include "struct_typedef.h"
#include "stdbool.h"

#define REFREE_HEADER_SOF 0xA5
#define REF_PROTOCOL_FRAME_MAX_SIZE         128

#define REF_PROTOCOL_HEADER_SIZE            sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE               2
#define REF_PROTOCOL_CRC16_SIZE             2
#define REF_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))
#define REFEREE_BUFFER_SIZE     200

#define LEN_HEADER 5

//ͨ��Э���ʽ λ����
typedef enum
{
    FRAME_HEADER= 0,
    CMD_ID               = 5,
    DATA                 = 7,
}RefereeFrameOffset;

// frame_header ��ʽ
typedef enum
{
    SOF          = 0,//��ʼλ
    DATA_LENGTH  = 1,//֡�����ݳ���,�����������ȡ���ݳ���
    SEQ          = 3,//�����
    CRC8         = 4 //CRC8
}FrameHeaderOffset;

//����ϵͳ����ID
/***************������ID********************

	ID: 0x0001  Byte:  3    ����״̬����       			����Ƶ�� 1Hz
	ID: 0x0002  Byte:  1    �����������         		������������
	ID: 0x0003  Byte:  32   ����������Ѫ������   		1Hz����       **
	ID: 0x0004  Byte:  3   	���ڷ���״̬  		?		���ڷ���ʱ����**
	ID: 0x0005  Byte:  3   	�˹�������ս���ӳ���ͷ�����   **

	ID: 0x0101  Byte:  4    �����¼�����   				�¼��ı����
	ID: 0x0102  Byte:  3    ���ز���վ������ʶ����    	�����ı����
	ID: 0X0104  Byte:  2    ���о�������
	ID: 0x0105  Byte:  1    ���ڷ���ڵ���ʱ

	ID: 0X0201  Byte: 15    ������״̬����        		10Hz
	ID: 0X0202  Byte: 14    ʵʱ������������   			50Hz
	ID: 0x0203  Byte: 16    ������λ������           	10Hz
	ID: 0x0204  Byte:  1    ��������������           	����״̬�ı����
	ID: 0x0205  Byte:  3    ���л���������״̬����      10Hz
	ID: 0x0206  Byte:  1    �˺�״̬����           		�˺���������
	ID: 0x0207  Byte:  6    ʵʱ�������           		�ӵ��������
	ID: 0x0208  Byte:  2    ����ʣ������  �����л����� �ڱ�
	ID: 0x0209  Byte:  4    ������RFID״̬

	ID: 0x0301  Byte:  n    �����˼佻������           	���ͷ���������,10Hz

*/
////�����룬�޸�ǰ
//typedef enum
//{
//    Referee_ID_game_state                   = 0x0001,
//    Referee_ID_game_result                  = 0x0002,
//    Referee_ID_game_robot_survivors       	= 0x0003,//���������˴������
//    Referee_ID_game_dart_state              = 0x0004, //���ڷ���״̬
//    Referee_ID_game_buff                    = 0x0005,//buff
//    Referee_ID_event_data  					= 0x0101,//�����¼�����
//    Referee_ID_supply_projectile_action   	= 0x0102,//���ز���վ������ʶ����
//    Referee_ID_supply_warm 	                = 0x0104,//����ϵͳ��������
//    Referee_ID_dart_shoot_time              = 0x0105, //���ڷ���ڵ���ʱ
//    Referee_ID_game_robot_state    			= 0x0201,//������״̬����
//    Referee_ID_power_heat_data    			= 0x0202,//ʵʱ������������
//    Referee_ID_game_robot_pos        		= 0x0203,//������λ������
//    Referee_ID_buff_musk					= 0x0204,//��������������
//    Referee_ID_aerial_robot_energy			= 0x0205,//���л���������״̬����
//    Referee_ID_robot_hurt					= 0x0206,//�˺�״̬����
//    Referee_ID_shoot_data					= 0x0207,//ʵʱ�������
//    Referee_ID_bullet_remaining             = 0x0208,//ʣ�෢����
//    Referee_ID_rfid_status					= 0x0209,//������RFID״̬��1Hz
//    Referee_ID_dart_client_directive        = 0x020A,//���ڻ����˿ͻ���ָ����, 10Hz
//    Referee_ID_robot_interactive_header_data	  = 0x0301,//�����˽������ݣ��������ͷ������������� 10Hz
//    Referee_ID_controller_interactive_header_data = 0x0302,//�Զ���������������ݽӿڣ�ͨ�������ͻ��˴����������� 30Hz
//    Referee_ID_map_interactive_header_data        = 0x0303,//�ͻ���С��ͼ�������ݣ������������͡���
//    Referee_ID_keyboard_information               = 0x0304,//���̡������Ϣ��ͨ������ͼ�����ڡ�������
////    IDCustomData,
//}referee_cmd_id_t;


//�޸ĺ�
typedef enum
{
    Referee_ID_game_state                   = 0x0001,//����״̬����
    Referee_ID_game_result                  = 0x0002,//�����������
//    Referee_ID_game_robot_survivors       	= 0x0003,//���������˴������->������Ѫ������
    Referee_ID_game_robot_HP       	        = 0x0003,//���������˴������->������Ѫ������
//    Referee_ID_game_dart_state              = 0x0004,//���ڷ���״̬  ɾ��
//    Referee_ID_game_buff                    = 0x0005,//buff        ɾ��

    Referee_ID_event_data  					= 0x0101,//�����¼�����
    Referee_ID_supply_projectile_action   	= 0x0102,//���ز���վ������ʶ����
    Referee_ID_supply_warm 	                = 0x0104,//����ϵͳ��������
//    Referee_ID_dart_shoot_time              = 0x0105, //���ڷ���ڵ���ʱ->���ڷ����������
    Referee_ID_dart_info                    = 0x0105, //���ڷ���ڵ���ʱ->���ڷ����������

    Referee_ID_game_robot_state    			= 0x0201,//������״̬����
    Referee_ID_power_heat_data    			= 0x0202,//ʵʱ������������
    Referee_ID_game_robot_pos        		= 0x0203,//��������λ������
    Referee_ID_buff_musk					= 0x0204,//��������������
    Referee_ID_aerial_robot_energy			= 0x0205,//���л���������״̬����
    Referee_ID_robot_hurt					= 0x0206,//�˺�״̬����
    Referee_ID_shoot_data					= 0x0207,//ʵʱ�������
    Referee_ID_bullet_remaining             = 0x0208,//ʣ�෢����
    Referee_ID_rfid_status					= 0x0209,//������RFID״̬��3Hz
    Referee_ID_dart_client_directive        = 0x020A,//���ڻ����˿ͻ���ָ����, 3Hz
    Referee_ID_dart_all_robot_position      = 0x020B,//�������л�����λ������
    Referee_ID_radar_mark                   = 0x020C,//�״��ǽ�������
    Referee_ID_entry_info                   = 0x020D,//�ڱ�����������Ϣͬ�� 1Hz
    Referee_ID_radar_info                   = 0x020E,//�״�����������Ϣͬ�� 1Hz

    Referee_ID_robot_interactive_header_data	    = 0x0301,//�����˽������ݣ��������ͷ������������� 10Hz
    Referee_ID_controller_interactive_header_data   = 0x0302,//�Զ��������������˽������ݣ����ͷ��������ͣ�Ƶ������30Hz
    Referee_ID_map_command                          = 0x0303,//ѡ�ֶ�С��ͼ�������ݣ�ѡ�ֶ˴�������
    Referee_ID_keyboard_information                 = 0x0304,//����ң�����ݣ��̶�30Hz,ͼ����·
    Referee_ID_robot_map_robot_data                 = 0x0305,//ѡ�ֶ�С��ͼ�����״�����,����10Hz
    Referee_ID_robot_custom_client                  = 0x0306,//�Զ����������ѡ�ֶ˽������ݣ����ͷ��������ͣ�Ƶ������30Hz
    Referee_ID_robot_entry_info_receive             = 0x0307,//ѡ�ֶ�С��ͼ�����ڱ�����,����1Hz
    Referee_ID_robot_custom_info_receive            = 0x0308,//ͨ��������·���ջ����˵�����,���ض�λ����ʾ������3Hz
//    IDCustomData,?
}referee_cmd_id_t;



//����ϵͳ����������ݳ���
typedef enum
{
    /* Std */
    Referee_LEN_FRAME_HEAD 	                    = 5,	// ֡ͷ����
    Referee_LEN_CMD_ID 		                    = 2,	// �����볤��
    Referee_LEN_FRAME_TAIL 	                    = 2,	// ֡βCRC16
    // ֡βCRC16
    /* Ext */

    Referee_LEN_game_state       				=  11,	//0x0001
    Referee_LEN_game_result       				=  1,	//0x0002
    Referee_LEN_game_robot_HP     		        =  32,	//0x0003  ����������Ѫ������
//    Referee_LEN_game_robot_survivors       		=  32,	//0x0003  ����������Ѫ������
//    Referee_LED_game_missile_state              = 3  , //0X0004���ڷ���״̬  ɾ��
//    Referee_LED_game_buff                       =11 , //0X0005

    Referee_LEN_event_data  					=  4,	//0x0101  �����¼�����
    Referee_LEN_supply_projectile_action        =  4,	//0x0102���ز���վ������ʶ����
    Referee_LEN_supply_warm                     = 3,    //0x0104 ����ϵͳ����
//    Referee_LEN_missile_shoot_time              = 3,    //0x0105 ���ڷ���ڵ���ʱ
    Referee_LEN_dart_info                       = 3,    //0x0105 ���ڷ���ڵ���ʱ

    Referee_LEN_game_robot_state    			= 13,	//0x0201 ������״̬����
    Referee_LEN_power_heat_data   				= 16,	//0x0202 ʵʱ������������
    Referee_LEN_game_robot_pos        			= 16,	//0x0203 ������λ������
    Referee_LEN_buff_musk        				=  6,	//0x0204 ��������������
    Referee_LEN_aerial_robot_energy        		=  2,	//0x0205 ���л���������״̬����
    Referee_LEN_robot_hurt        				=  1,	//0x0206 �˺�״̬����
    Referee_LEN_shoot_data       				=  7,	//0x0207 ʵʱ�������
    Referee_LEN_bullet_remaining                = 6,    //0x0208ʣ�෢����
    Referee_LEN_rfid_status					    = 4,    //0x0209
    Referee_LEN_dart_client_directive           = 6,    //0x020A
    Referee_LEN_dart_all_robot_position         = 40,   //0x020B
    Referee_LEN_radar_mark                      = 6,    //0x020C
    Referee_LEN_entry_info                      =4,     //0x020D
    Referee_LEN_radar_info                      =1,     //0x020E

    Referee_LEN_robot_interactive_header_data   =128,   //0x0301
    Referee_LEN_controller_interactive_header_data=30,  //0x0302
    Referee_LEN_map_command                     =15,    //0x0303
    Referee_LEN_keyboard_information            =12,    //0x0304
    Referee_LEN_robot_map_robot_data            =10,    //0x0305
    Referee_LEN_robot_custom_client             =8,     //0x0306
    Referee_LEN_robot_entry_info_receive        =103,   //0x0307
    Referee_LEN_robot_custom_info_receive       =34,    //0x0308

}RefereeDataLength;


typedef enum{
    Referee_hero_red       = 1,
    Referee_engineer_red   = 2,
    Referee_infantry3_red  = 3,
    Referee_infantry4_red  = 4,
    Referee_infantry5_red  = 5,
    Referee_plane_red      = 6,

    Referee_hero_blue      = 101,
    Referee_engineer_blue  = 102,
    Referee_infantry3_blue = 103,
    Referee_infantry4_blue = 104,
    Referee_infantry5_blue = 105,
    Referee_plane_blue     = 106,
}Referee_robot_ID;


typedef struct {
    bool static_update;//��̬Ԫ���Ƿ�Ҫˢ��
    uint8_t gimbal_mode;//��̨ģʽ ���˿� �����Կأ��Կ��Ǵ����������)
    uint8_t chassis_mode;//
    uint8_t block_warning;//�µ�����
    uint8_t shoot_heat_limit;//��ǰ��������
    fp32 super_cap_value;//��������ֵ
    uint8_t fire_mode;
    float pitch_value;
    float relative_yaw_value;
}ui_robot_status_t;//������״̬�ṹ�� ��������Ƿ��,�����Ƿ�򿪣������Ƿ�򿪵���Ϣ

//����ϵͳ֡ͷ�ṹ��
typedef  struct
{
    uint8_t SOF;
    uint16_t data_length;
    uint8_t seq;
    uint8_t CRC8;
}__packed frame_header_struct_t;


/* ID: 0x0001  Byte:  11    ����״̬���� */
typedef  struct
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} __packed ext_game_state_t;


/* ID: 0x0002  Byte:  1    ����������� */
typedef  struct
{
    uint8_t winner;
    bool game_over;
}__packed  ext_game_result_t;


/* ID: 0x0003  Byte:  32    ����������Ѫ������ */
typedef  struct
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_6_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;

    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_6_robot_HP;
    uint16_t blue_7_robot_HP;

    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} __packed ext_game_robot_HP_t;



/* ID: 0x0101  Byte:  4    �����¼����� */
typedef  struct
{
    uint32_t event_type;
} __packed ext_event_data_t;


/* ID: 0x0102  Byte:  4    ���ز���վ������ʶ���� */
typedef  struct
{
//    uint8_t supply_projectile_id;  0
    uint8_t reserved;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} __packed ext_supply_projectile_action_t;

//V1.6.1�޸�
/* ID: 0x0104  Byte: 2->3   ����ϵͳ������Ϣ */
typedef  struct
{
//    uint8_t level;
//    uint8_t foul_robot_id;
    uint8_t level;
    uint8_t offending_robot_id;
    uint8_t count;
} __packed  ext_referee_warning_t;

//V1.6.1�޸�
/* ID: 0x0105  Byte:1->3  ���ڷ���ڵ���ʱ */
typedef  struct
{
    uint8_t dart_remaining_time;
    uint16_t dart_info;  //����
} __packed ext_dart_remaining_time_t;

//V1.6.1�޸�
///* ID: 0X0201  Byte: 27    ������״̬���� */
//typedef  struct
//{
//    uint8_t robot_id;   //������ID��������У�鷢��
//    uint8_t robot_level;  //1һ����2������3����
//    uint16_t remain_HP;  //������ʣ��Ѫ��
//    uint16_t max_HP; //������Ѫ������
//
//    uint16_t shooter1_17mm_cooling_rate;  //������ 17mm �ӵ�������ȴ�ٶ� ��λ /s
//    uint16_t shooter1_17mm_cooling_limit;   // ������ 17mm �ӵ���������
//    uint16_t shooter1_17mm_speed_limit;
//
//
//    uint16_t shooter2_17mm_cooling_rate;
//    uint16_t shooter2_17mm_cooling_limit;
//    uint16_t shooter2_17mm_speed_limit;
//
//
//    uint16_t shooter_42mm_cooling_rate;
//    uint16_t shooter_42mm_cooling_limit;
//    uint16_t shooter_42mm_speed_limit;
//
//
//    uint16_t max_chassis_power;
//    uint8_t mains_power_gimbal_output : 1;
//    uint8_t mains_power_chassis_output : 1;
//    uint8_t mains_power_shooter_output : 1;
//} __packed ext_game_robot_state_t;

/* ID: 0X0201  Byte:     ������״̬���� */

typedef  struct
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;

    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t  chassis_power_limit;

    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
} __packed ext_robot_status_t;

/* ID: 0X0202  Byte: 16    ʵʱ������������ */
typedef  struct
{
    uint16_t chassis_volt;
    uint16_t chassis_current;
    float chassis_power;   //˲ʱ����
    uint16_t chassis_power_buffer;//60������������
    uint16_t shooter_heat0;//��1�� 17mm ���������ǹ������
    uint16_t shooter_heat1;// ��2�� 17mm ���������ǹ������
    uint16_t mobile_shooter_heat2;//42mm ���������ǹ������
} __packed ext_power_heat_data_t;

//V1.6.1�޸�
///* ID: 0x0203  Byte: 16    ������λ������ */
//typedef  struct
//{
//    float x;
//    float y;
//    float z;
//    float yaw;
//} __packed ext_game_robot_pos_t;

/* ID: 0x0203  Byte: 16    ������λ������ */
typedef  struct
{
    float x;//��λm
    float y;
    float angle;//�������˲���ģ��ĳ��򣬵�λ���ȡ�����Ϊ 0 ��
}__packed ext_robot_pos_t;

//V1.6.1�޸�
///* ID: 0x0204  Byte:  1    �������������� */
//typedef  struct
//{
//    uint8_t power_rune_buff;
//} __packed ext_buff_musk_t;

/* ID: 0x0204  Byte:  1    �������������� */
typedef struct
{
    uint8_t recovery_buff;          //��Ѫ,ֵ��ʾ�ٷֱ�
    uint8_t cooling_buff;           //ǹ����ȴ����,5��ʾ5����ȴ
    uint8_t defence_buff;           //����,�����������ǰٷֱ�
    uint8_t vulnerability_buff;     //����
    uint16_t attack_buff;           //����
}__packed  ext_buff_t;

/* ID: 0x0205  Byte:  1->2    ���л���������״̬���� */
typedef  struct
{
    uint8_t airforce_status; //���л�����״̬��0 ������ȴ��1 ��ȴ��ϣ�2 ���ڿ���֧Ԯ�� V1.6.1����
    uint8_t attack_time;
} __packed aerial_robot_energy_t;


/* ID: 0x0206  Byte:  1    �˺�״̬���� */
typedef  struct
{
    uint8_t armor_id : 4;
    uint8_t hurt_type : 4;
    bool being_hurt;
} __packed ext_robot_hurt_t;


/* ID: 0x0207  Byte:  7    ʵʱ������� */
typedef  struct
{
    uint8_t bullet_type;    // 1 17mm, 2 42mm
    uint8_t shooter_id;     // 1 17mm1, 2 17mm2, 3 42mm
    uint8_t bullet_freq;    //���٣���λ��Hz��
    float bullet_speed;     //����ٶȣ���λ��m/s)
}__packed  ext_shoot_data_t;


/* ID: 0x0208  Byte:  6    �ӵ�ʣ������ */
typedef  struct
{
    uint16_t bullet_remaining_num_17mm;
    uint16_t bullet_remaining_num_42mm;
    uint16_t coin_remaining_num;//���ʣ��
} __packed ext_bullet_remaining_t;

/* ID: 0x0209  Byte:  4 	������RFID״̬ */
typedef  struct
{
    uint32_t rfid_status;
} __packed ext_rfid_status_t;

////V1.6.1�޸�
///* ID:  0x020A  Byte:   	 */
//typedef  struct{
//    uint8_t dart_launch_opening_status;//��ǰ���ڷ���ڵ�״̬
//    uint8_t dart_attack_target;        //���ڵĴ��Ŀ�꣬Ĭ��Ϊǰ��վ��1��ǰ��վ��2�����أ�
//    uint16_t target_change_time;       //�л����Ŀ��ʱ�ı���ʣ��ʱ��
//    uint8_t first_dart_speed;          //��⵽�ĵ�һö�����ٶȣ���λ 0.1m/s/LSB
//    uint8_t second_dart_speed;         //��⵽�ĵڶ�ö�����ٶȣ���λ 0.1m/s/LSB
//    uint8_t third_dart_speed;          //��⵽�ĵ���ö�����ٶȣ���λ 0.1m/s/LSB
//    uint8_t fourth_dart_speed;         //��⵽�ĵ���ö�����ٶȣ���λ 0.1m/s/LSB
//    uint16_t last_dart_launch_time;    //���һ�εķ�����ڵı���ʣ��ʱ�䣬��λ��
//    uint16_t operate_launch_cmd_time;  //���һ�β�����ȷ������ָ��ʱ�ı���ʣ��ʱ�䣬��λ��
//}__packed ext_dart_client_cmd_t; //LEN_DART_CLIENT_DIRECTIVE  ��3-19

/* ID:  0x020A  Byte:6   	 */
typedef  struct
{
    uint8_t dart_launch_opening_status;     //���ڷ���״̬ 0 �Ѿ�����, 1 �ر�, 2 ���ڿ������߹ر���,
    uint8_t reserved;                       //����
    uint16_t target_change_time;            //�л�����Ŀ��ʱ�ı���ʣ��ʱ�䣬��λs,Ĭ��Ϊ0
    uint16_t latest_launch_cmd_time;        //���һ�β�����ȷ������ָ��ʱ�ı���ʣ��ʱ��,��λs,��ʼֵΪ0
}__packed ext_dart_client_cmd_t;

//V1.6.1���� 24�����Զ�����
/* ID:   0x020B  Byte:40   	 */
typedef  struct
{
    float hero_x;
    float hero_y;

    float engineer_x;
    float engineer_y;

    float standard_3_x;
    float standard_3_y;

    float standard_4_x;
    float standard_4_y;

    float standard_5_x;
    float standard_5_y;
}__packed ext_ground_robot_position_t;

//V1.6.1���� 24�����״��޸�
/* ID:   0x020C  Byte:6  �����˱��״��ǽ��� 0-120	 */
typedef  struct
{
    uint8_t mark_hero_progress;
    uint8_t mark_engineer_progress;
    uint8_t mark_standard_3_progress;
    uint8_t mark_standard_4_progress;
    uint8_t mark_standard_5_progress;
    uint8_t mark_sentry_progress;
}__packed ext_radar_mark_data_t;

//V1.6.1���� 24�����ڱ��޸�
/* ID:   0x020D  Byte:4  �ڱ��һ���������Ѫ���� */
typedef  struct
{
    uint32_t sentry_info;
} __packed ext_sentry_info_t;

//V1.6.1���� 24�����״��޸�
/* ID:   0x020E  Byte:1  �״ﴥ��˫����Ϣ	 */
typedef struct
{
    uint8_t radar_info;
} __packed  ext_radar_info_t;

/*
	�������ݣ�����һ��ͳһ�����ݶ�ͷ�ṹ��
	���������� ID���������Լ������ߵ� ID ���������ݶΣ�
	�����������ݵİ��ܹ������Ϊ 128 ���ֽڣ�
	��ȥ frame_header,cmd_id,frame_tail �Լ����ݶ�ͷ�ṹ�� 6 ���ֽڣ�
	�ʶ����͵��������ݶ����Ϊ 113��
	������������ 0x0301 �İ�����Ƶ��Ϊ 10Hz��

	������ ID��
	1��Ӣ��(��)��
	2������(��)��
	3/4/5������(��)��
	6������(��)��
	7���ڱ�(��)��
	11��Ӣ��(��)��
	12������(��)��
	13/14/15������(��)��
	16������(��)��
	17���ڱ�(��)��
	�ͻ��� ID��
	0x0101 ΪӢ�۲����ֿͻ���( ��) ��
	0x0102 �����̲����ֿͻ��� ((�� )��
	0x0103/0x0104/0x0105�����������ֿͻ���(��)��
	0x0106�����в����ֿͻ���((��)��
	0x0111��Ӣ�۲����ֿͻ���(��)��
	0x0112�����̲����ֿͻ���(��)��
	0x0113/0x0114/0x0115�������ֿͻ��˲���(��)��
	0x0116�����в����ֿͻ���(��)��
*/


/* �������ݽ�����Ϣ��0x0301  */
typedef  struct
{
    uint16_t data_cmd_id;
    uint16_t send_ID;
    uint16_t receiver_ID;
}__packed ext_student_interactive_header_data_t;

typedef struct{
    uint16_t teammate_hero;
    uint16_t teammate_engineer;
    uint16_t teammate_infantry3;
    uint16_t teammate_infantry4;
    uint16_t teammate_infantry5;
    uint16_t teammate_plane;
    uint16_t teammate_sentry;

    uint16_t client_hero;
    uint16_t client_engineer;
    uint16_t client_infantry3;
    uint16_t client_infantry4;
    uint16_t client_infantry5;
    uint16_t client_plane;
}ext_interact_id_t;

/* ѡ�ֶ�С��ͼ�������ݣ�0x0303  */
typedef  struct
{
    float target_position_x;
    float target_position_y;
    uint8_t cmd_keyboard;
    uint8_t target_robot_id;
    uint8_t cmd_source;
}__packed ext_map_command_t;

/* ����ң�����ݣ�0x0304  */
typedef struct
{
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    int8_t left_button_down;
    int8_t right_button_down;
    uint16_t keyboard_value;
    uint16_t reserved;
}__packed ext_remote_control_t;

/* ѡ�ֶ�С��ͼ�����״����ݣ�0x0305  */
typedef struct
{
    uint16_t target_robot_id;
    float target_position_x;
    float target_position_y;
} ext_map_robot_data_t;

/* �Զ����������ѡ�ֶ˽������ݣ�0x0306  */
typedef struct
{
    uint16_t key_value;
    uint16_t x_position:12;
    uint16_t mouse_left:4;
    uint16_t y_position:12;
    uint16_t mouse_right:4;
    uint16_t reserved;
}__packed ext_custom_client_data_t;

/* ѡ�ֶ�С��ͼ�����ڱ����ݣ�0x0307  */
typedef  struct
{
    uint8_t intention;
    uint16_t start_position_x;
    uint16_t start_position_y;
    int8_t delta_x[49];
    int8_t delta_y[49];
    uint16_t sender_id;
}__packed ext_map_data_t;

/* ѡ�ֶ�С��ͼ���ջ��������ݣ�0x0308  */
typedef  struct
{
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[30];
} __packed ext_custom_info_t;

/*
	ѧ�������˼�ͨ�� cmd_id 0x0301������ ID:0x0200~0x02FF
	�������� �����˼�ͨ�ţ�0x0301��
	����Ƶ�ʣ����� 10Hz

	�ֽ�ƫ���� 	��С 	˵�� 			��ע
	0 			2 		���ݵ����� ID 	0x0200~0x02FF
										���������� ID ��ѡȡ������ ID �����ɲ������Զ���

	2 			2 		�����ߵ� ID 	��ҪУ�鷢���ߵ� ID ��ȷ�ԣ�

	4 			2 		�����ߵ� ID 	��ҪУ������ߵ� ID ��ȷ�ԣ�
										���粻�ܷ��͵��жԻ����˵�ID

	6 			n 		���ݶ� 			n ��ҪС�� 113

*/
typedef  struct
{
    uint8_t data[113]; //���ݶ�,n��ҪС��113
} __packed robot_interactive_data_t;

typedef struct judge_info_struct {
    frame_header_struct_t 							FrameHeader;				// ֡ͷ��Ϣ

    ext_game_state_t 							    GameState;				    // 0x0001           ����״̬����
    ext_game_result_t 							    GameResult;				    // 0x0002         �����������
    ext_game_robot_HP_t 						    GameRobotHP;			    // 0x0003         ������Ѫ������
//    ext_dart_status_t								GameDartStatus;				// 0x0004         ���ڷ���״̬
//    ext_ICRA_buff_debuff_zone_status_t	            GameICRABuff;           // �˹�������ս���ӳ���ͷ���״̬  ɾ��

    ext_event_data_t								EventData;					// 0x0101         �����¼�����
    ext_supply_projectile_action_t	                SupplyProjectileAction;		// 0x0102 ����վ������ʶ
    ext_referee_warning_t						    RefereeWarning;		        // 0x0104         ���о�����Ϣ
    ext_dart_remaining_time_t				        DartRemainingTime;          // 0x0105         ���ڷ���ڵ���ʱ


    ext_robot_status_t					            GameRobotStat;	            // 0x0201         ����������״̬
    ext_power_heat_data_t						    PowerHeatData;		        // 0x0202         ʵʱ������������
    ext_robot_pos_t						            GameRobotPos;			    // 0x0203         ������λ��
    ext_buff_t									    Buff;						// 0x0204     ����������
    aerial_robot_energy_t				            AerialRobotEnergy;// 0x0205             ���л���������״̬
    ext_robot_hurt_t								RobotHurt;					//0x0206         �˺�״̬
    ext_shoot_data_t								ShootData;					//0x0207         ʵʱ�����Ϣ(��Ƶ  ����  �ӵ���Ϣ)
    ext_bullet_remaining_t					        BulletRemaining;		    //0x0208	        �ӵ�ʣ�෢����
    ext_rfid_status_t								RfidStatus;				    //0x0209	        RFID��Ϣ
    ext_dart_client_cmd_t                           DartClient;                 //0x020A         ���ڿͻ���
    ext_ground_robot_position_t                     RobotPosition;              //0x020B
    ext_radar_mark_data_t                           RadarMark;                  //0x020C
    ext_sentry_info_t                               SentryInfo;                 //0x020D
    ext_radar_info_t                                RadarInfo;                  //0x020E

    ext_student_interactive_header_data_t           StudentInteractive;//0x0301
    ext_map_command_t                               MapCommand;//0x0303
    ext_remote_control_t                            keyboard;      //0x0304 ����
    ext_map_robot_data_t                            EnemyPosition;      //0x0305 �з�������λ��
    ext_custom_client_data_t                        Custom;             //0x0306 �Զ��������
    ext_map_data_t                                  SentryMapData;      //0x0307�ڱ���������
    ext_custom_info_t                               SendData;           //0x0308 �������Զ��巢����Ϣ

    ext_interact_id_t								ids;								//�뱾�������Ļ�����id
    uint16_t                                        SelfClient;       //�����ͻ���

} Referee_info_t;


/*
	ѧ�������˼�ͨ�� cmd_id 0x0301������ data_ID:0x0200~0x02FF
	�������� �����˼�ͨ�ţ�0x0301��
	����Ƶ�ʣ����������кϼƴ������� 5000 Byte�� �����з���Ƶ�ʷֱ𲻳���30Hz��
 * +------+------+-------------+------------------------------------+
 * | byte | size |    breif    |            note                    |
 * |offset|      |             |                                    |
 * +------+------+-------------+------------------------------------+
 * |  0   |  2   | 	 data_ID   | 0x0200~0x02FF,��������Щ ID ��ѡȡ |
 * |      |      |             | ����ID�����ɲ������Զ���           |
 * +------|------|-------------|------------------------------------|
 * |  2   |  2   | 	sender_ID  | ��ҪУ�鷢���ߵ� ID ��ȷ��					|
 * +------|------|-------------|------------------------------------|
 * |  4   |  2   | receiver_ID | ��ҪУ������ߵ� ID ��ȷ��					|
 * |      |      |             | ���粻�ܷ��͵��жԻ����˵�ID				|
 * +------|------|-------------|------------------------------------|
 * |  6   |  n   | 		Data     | n ��ҪС�� 113 										|
 * +------+------+-------------+------------------------------------+
*/


//����ͼ��ID
typedef enum
{
    //0x200-0x02ff 	�����Զ������� ��ʽ  INTERACT_ID_XXXX
    UI_INTERACT_ID_delete_graphic 			= 0x0100,	/*�ͻ���ɾ��ͼ��*/
    UI_INTERACT_ID_draw_one_graphic 		= 0x0101,	/*�ͻ��˻���һ��ͼ��*/
    UI_INTERACT_ID_draw_two_graphic 		= 0x0102,	/*�ͻ��˻���2��ͼ��*/
    UI_INTERACT_ID_draw_five_graphic 	= 0x0103,	/*�ͻ��˻���5��ͼ��*/
    UI_INTERACT_ID_draw_seven_graphic 	= 0x0104,	/*�ͻ��˻���7��ͼ��*/
    UI_INTERACT_ID_draw_char_graphic 	= 0x0110,	/*�ͻ��˻����ַ�ͼ��*/
    UI_INTERACT_ID_bigbome_num					= 0x02ff
}Interact_ID;

typedef enum
{
    UI_LEN_INTERACT_delete_graphic     = 8,  //ɾ��ͼ�� 2(��������ID)+2(������ID)+2��������ID��+2���������ݣ�
    UI_LEN_INTERACT_draw_one_graphic   = 21, // ����2+2+2+15
    UI_LEN_INTERACT_draw_two_graphic   = 36, //6+15*2
    UI_LEN_INTERACT_draw_five_graphic  = 81, //6+15*5
    UI_LEN_INTERACT_draw_seven_graphic = 111,//6+15*7
    UI_LEN_INTERACT_draw_char_graphic  = 51, //6+15+30���ַ������ݣ�
}Interact_ID_len;

//ͼ������
typedef  struct
{
    uint8_t graphic_name[3];
    uint32_t operate_tpye:3;
    uint32_t graphic_tpye:3;          //ֱ��  ����  ��Բ  ��Բ  Բ��  ����  ����  �ַ�
    uint32_t layer:4;
    uint32_t color:4;
    uint32_t start_angle:9;           //��    ��    ��    ��    �Ƕ�  ��С  ��С  ��С
    uint32_t end_angle:9;             //��    ��    ��    ��          λ��  ��    ����
    uint32_t width:10;
    uint32_t start_x:11;              //���  ���  Բ��  Բ��  Բ��  ���  ���  ���
    uint32_t start_y:11;              //
    union {
        struct {
            uint32_t radius:10;      //��    ��    �뾶  ��    ��    ��    ��    ��
            uint32_t end_x:11;       //�յ�  �Զ�  ��    ����  ����  ��    ��    ��
            uint32_t end_y:11;       //                              ��    ��    ��                  ��    ��    ��
        };
        int32_t number;
    };

} __packed ui_graphic_data_struct_t;//ui��ͷ �����κ͸�����Ҳ��Ϊͼ�� ������graphic����

typedef  struct
{
    frame_header_struct_t txFrameHeader;			//֡ͷ
    uint16_t  CmdID;										//������
    ext_student_interactive_header_data_t   dataFrameHeader;//���ݶ�ͷ�ṹ
    ui_graphic_data_struct_t clientData;		//���ݶ�
    uint16_t	FrameTail;								//֡β
}__packed ext_graphic_one_data_t;

typedef  struct
{
    frame_header_struct_t txFrameHeader;			//֡ͷ
    uint16_t  CmdID;										//������
    ext_student_interactive_header_data_t   dataFrameHeader;//���ݶ�ͷ�ṹ
    ui_graphic_data_struct_t clientData[2];		//���ݶ�
    uint16_t	FrameTail;								//֡β
}__packed ext_graphic_two_data_t;

typedef  struct
{
    frame_header_struct_t txFrameHeader;			//֡ͷ
    uint16_t  CmdID;										//������
    ext_student_interactive_header_data_t   dataFrameHeader;//���ݶ�ͷ�ṹ
    ui_graphic_data_struct_t clientData[5];		//���ݶ�
    uint16_t	FrameTail;								//֡β
}__packed ext_graphic_five_data_t;

typedef  struct
{
    frame_header_struct_t txFrameHeader;			//֡ͷ
    uint16_t  CmdID;										//������
    ext_student_interactive_header_data_t   dataFrameHeader;//���ݶ�ͷ�ṹ
    ui_graphic_data_struct_t clientData[7];		//���ݶ�
    uint16_t	FrameTail;								//֡β
}__packed ext_graphic_seven_data_t;

//���ַ���
//�ַ�������ui_graphic_data_struct_t����ṹ�������
// ����30���ֽڵ��ַ����ݴ洢�ַ���
typedef  struct
{
    ui_graphic_data_struct_t graphic_data_struct;
    uint8_t data[30];
}__packed ui_string_t;

//�̶����ݶγ������ݰ�
typedef  struct
{
    frame_header_struct_t txFrameHeader;			//֡ͷ
    uint16_t  CmdID;										//������
    ext_student_interactive_header_data_t   dataFrameHeader;//���ݶ�ͷ�ṹ
    ui_string_t clientData;//���ݶ�
    uint16_t	FrameTail;								//֡β
}__packed ext_string_data_t;

//****************************��ͼ�����ݶ�����****************************/


//typedef  struct//ͼ��
//{
//    uint8_t graphic_name[3];
//    uint32_t operate_tpye:3;
//    uint32_t graphic_tpye:3; //ֱ��  ����  ��Բ  ��Բ  Բ��  ����  ����  �ַ�
//    uint32_t layer:4;
//    uint32_t color:4;
//    uint32_t start_angle:9;  //��    ��    ��    ��    �Ƕ�  ��С  ��С  ��С
//    uint32_t end_angle:9;    //��    ��    ��    ��          λ��  ��    ����
//    uint32_t width:10;
//    uint32_t start_x:11;     //���  ���  Բ��  Բ��  Բ��  ���  ���  ���
//    uint32_t start_y:11;     //
//} __packed graphic_data_struct_t;
//
//typedef  struct//������
//{
//    uint8_t graphic_name[3];
//    uint32_t operate_tpye:3;
//    uint32_t graphic_tpye:3;
//    uint32_t layer:4;
//    uint32_t color:4;
//    uint32_t start_angle:9;
//    uint32_t end_angle:9;
//    uint32_t width:10;
//    uint32_t start_x:11;
//    uint32_t start_y:11;
//    uint32_t number;
//}__packed  float_data_struct_t;
//
//typedef  struct//������
//{
//    uint8_t graphic_name[3];
//    uint32_t operate_tpye:3;
//    uint32_t graphic_tpye:3;
//    uint32_t layer:4;
//    uint32_t color:4;
//    uint32_t start_angle:9;
//    uint32_t end_angle:9;
//    uint32_t width:10;
//    uint32_t start_x:11;
//    uint32_t start_y:11;
//    uint32_t number;
//} __packed int_data_struct_t;

/* data_ID: 0X0100  Byte:  2	    �ͻ���ɾ��ͼ��*/
typedef  struct
{
    uint8_t operate_type;
    uint8_t layer;//ͼ������0~9
}__packed ext_client_custom_graphic_delete_t;

/* ͼ��ɾ��������ö�� */
typedef enum
{
    UI_NONE_delete    = 0,
    UI_GRAPHIC_delete = 1,
    UI_ALL_delete     = 2
}Delete_Graphic_Operate;//ext_client_custom_graphic_delete_t��uint8_t operate_type

//bit 0-2
typedef enum
{
    UI_NONE   = 0,/*�ղ���*/
    UI_ADD    = 1,/*����ͼ��*/
    UI_MODIFY = 2,/*�޸�ͼ��*/
    UI_DELETE = 3,/*ɾ��ͼ��*/
}Graphic_Operate;//graphic_data_struct_t��uint32_t operate_tpye
/*ͼ�����*/

//bit3-5
/*ͼ������*/
typedef enum
{
    UI_LINE      = 0,//ֱ��
    UI_RECTANGLE = 1,//����
    UI_CIRCLE    = 2,//��Բ
    UI_OVAL      = 3,//��Բ
    UI_ARC       = 4,//Բ��
    UI_FLOAT     = 5,//������
    UI_INT       = 6,//������
    UI_CHAR      = 7 //�ַ�
}Graphic_Type;
/*ͼ������*/

//bit 6-9ͼ���� ���Ϊ9����С0

//bit 10-13��ɫ
typedef enum
{
    UI_RED_BLUE  = 0,//������ɫ
    UI_YELLOW    = 1,
    UI_GREEN     = 2,
    UI_ORANGE    = 3,
    UI_FUCHSIA   = 4,	/*�Ϻ�ɫ*/
    UI_PINK      = 5,
    UI_CYAN_BLUE = 6,	/*��ɫ*/
    UI_BLACK     = 7,
    UI_WHITE     = 8
}Graphic_Color;

typedef enum {

    UI_ZERO_LAYER=0,
    UI_ONE_LAYER,
    UI_TWO_LAYER,
    UI_THREE_LAYER,
    UI_FOUR_LAYER,
    UI_FIVE_LAYER,
    UI_SIX_LAYER,
    UI_SEVEN_LAYER,
    UI_EIGHT_LAYER,

}Graphic_layer;
/*ͼ����ɫ����*/
//bit 14-31 �Ƕ� [0,360]

/*
 * ���ݽṹ��
 */

//ɾ��ͼ��
typedef  struct
{
    frame_header_struct_t txFrameHeader;
    uint16_t  CmdID;
    ext_student_interactive_header_data_t   dataFrameHeader;
    ext_client_custom_graphic_delete_t clientData;
    uint16_t	FrameTail;
} __packed deleteLayer_data_t;

/* ID:   0x0120  Byte:4  �ڱ���������ָ�� */
typedef  struct
{
    uint32_t sentry_cmd;
} __packed ext_sentry_cmd_t;
//��������ui����ɫ
typedef struct
{
    uint32_t cover_color;//���տ���
    uint32_t auto_aim_color;//���鿪��
    uint32_t spin_color;//��ת����
    uint32_t fire_color;//Ħ���ֿ�����
} __packed ext_ui_color;


/* ID:   0x0121  Byte:1  �״���������ָ�� */
typedef  struct
{
    uint8_t radar_cmd;
} __packed ext_radar_cmd_t;

////������ͼ
//typedef  struct
//{
//    frame_header_struct_t txFrameHeader;			//֡ͷ
//    uint16_t  CmdID;										//������
//    ext_student_interactive_header_data_t   dataFrameHeader;//���ݶ�ͷ�ṹ
//    graphic_data_struct_t clientData;		//���ݶ�
//    uint16_t	FrameTail;								//֡β
//}__packed ext_graphic_one_data_t;
//
//typedef  struct
//{
//    frame_header_struct_t txFrameHeader;
//    uint16_t  CmdID;
//    ext_student_interactive_header_data_t   dataFrameHeader;
//    graphic_data_struct_t clientData[2];
//    uint16_t	FrameTail;
//}__packed ext_graphic_two_data_t;
//
//
//typedef  struct
//{
//    frame_header_struct_t txFrameHeader;
//    uint16_t  CmdID;
//    ext_student_interactive_header_data_t   dataFrameHeader;
//    graphic_data_struct_t clientData[5];
//    uint16_t	FrameTail;
//}__packed ext_graphic_five_data_t;
//
//
//typedef  struct
//{
//    frame_header_struct_t txFrameHeader;
//    uint16_t  CmdID;
//    ext_student_interactive_header_data_t   dataFrameHeader;
//    graphic_data_struct_t clientData[7];
//    uint16_t	FrameTail;
//}__packed ext_graphic_seven_data_t;
//
//
////���Ƹ�����
//typedef  struct
//{
//    frame_header_struct_t txFrameHeader;
//    uint16_t  CmdID;
//    ext_student_interactive_header_data_t   dataFrameHeader;
//    float_data_struct_t clientData[2];
//    uint16_t	FrameTail;
//}__packed ext_float_two_data_t;
//
//typedef  struct
//{
//    frame_header_struct_t txFrameHeader;
//    uint16_t  CmdID;
//    ext_student_interactive_header_data_t   dataFrameHeader;
//    float_data_struct_t clientData;
//    uint16_t	FrameTail;
//}__packed ext_float_one_data_t;
//
//
//typedef  struct
//{
//    frame_header_struct_t txFrameHeader;
//    uint16_t  CmdID;
//    ext_student_interactive_header_data_t   dataFrameHeader;
//    float_data_struct_t clientData[7];
//    uint16_t	FrameTail;
//}__packed ext_float_seven_data_t;
//
////��������
//typedef  struct
//{
//    frame_header_struct_t txFrameHeader;
//    uint16_t  CmdID;
//    ext_student_interactive_header_data_t   dataFrameHeader;
//    int_data_struct_t clientData[2];
//    uint16_t	FrameTail;
//}__packed ext_int_two_data_t;
//
//typedef  struct
//{
//    frame_header_struct_t txFrameHeader;
//    uint16_t  CmdID;
//    ext_student_interactive_header_data_t   dataFrameHeader;
//    int_data_struct_t clientData;
//    uint16_t	FrameTail;
//}__packed ext_int_one_data_t;
//
//typedef  struct
//{
//    frame_header_struct_t txFrameHeader;
//    uint16_t  CmdID;
//    ext_student_interactive_header_data_t   dataFrameHeader;
//    int_data_struct_t clientData[7];
//    uint16_t	FrameTail;
//}__packed ext_int_seven_data_t;

//enum {
//    not_start,
//    prepare,
//    self_detect,
//    five_countdown,
//
//};
extern ui_robot_status_t ui_robot_status;
extern Referee_info_t Referee;
extern void referee_task(void const*argument);

_Noreturn extern void UI_paint_task(void const*argument);
extern uint8_t usart6_buf[REFEREE_BUFFER_SIZE];
extern uint8_t usart1_buf[REFEREE_BUFFER_SIZE];


#endif //DEMO1_REFEREE_H
