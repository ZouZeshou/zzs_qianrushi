#ifndef JUDGE_SYSTEM_H
#define JUDGE_SYSTEM_H

#include "stm32f4xx_hal.h"

#define VERSION19

typedef union{
	uint8_t c[2];
	int16_t d;
	uint16_t ud;
}wl2data;

typedef union{
	uint8_t c[4];
	float f;
	uint32_t d;
}wl4data;

//0x0001	����״̬����
typedef struct{ 
	float data1;
	float data2;
	float data3;
	struct{
		uint8_t bit0 : 1;
		uint8_t bit1 : 1;
		uint8_t bit2 : 1;
		uint8_t bit3 : 1;
		uint8_t bit4 : 1;
		uint8_t bit5 : 1;
	}code;
	uint8_t mask;
}data_to_judge_t;

extern int JudgeSendFresh;

#ifdef VERSION19
//0x0001	����״̬����
typedef struct{ 
	uint8_t game_type : 4; /*1 ���״�ʦ��   
												  *2  ������   
												  *3  ICRA	*/
	uint8_t game_progress : 4;/*0  δ��ʼ���� 
														 *1  ׼���׶�  
														 *2  �Լ�׶�
														 *3  5s����ʱ 
														 *4  ��ս��
														 *5  ����������  */
	uint16_t stage_remain_time; /* ��ǰ�׶�ʣ��ʱ�� s*/
}ext_game_state_t;

//0x0002	�����������
typedef struct{ 
	uint8_t winner; /* 0 ƽ��
									 * 1 �췽ʤ��
									 * 2 ����ʤ�� */
}ext_game_result_t;

//0x0003	�����˴������
typedef struct{
	uint16_t robot_legion; /* �췽 λ0��ʼ Ӣ�ۡ����̡�����1������2������3�����С��ڱ������� 
												  * ���� λ8��ʼ Ӣ�ۡ����̡�����1������2������3�����С��ڱ������� */
}ext_game_robot_survivors_t;

//0x0101	�����¼�����
typedef struct{
	uint32_t event_type; /* ���� ����Ҫ�����ֲ� */
}ext_event_data_t;

//0x0102	����վ������ʶ
typedef struct { 
	uint8_t supply_projectile_id;/* ����վID    
															  * 1��1�Ų����� 
															  * 2��2�Ų����� */
	uint8_t supply_robot_id;/* ����������ID   
													 * 0 �� �޻����˲��� 
													 * 1 �� ��Ӣ��
													 * 2 �� �칤��
													 * 3/4/5 �� �첽��
													 * 11 �� ��Ӣ��
													 * 12 �� ������
													 * 13/14/15 �� ������		*/
	uint8_t supply_projectile_step;/* �����ڿ���״̬ 
																  * 0���ر�
																  * 1���ӵ�׼����
																  * 2���ӵ����� */
	uint8_t supply_projectile_num;  /* ��������  50/100/150/200 */
} ext_supply_projectile_action_t;

//0x0103	���󲹸�վ���� �Կ���δ����
typedef struct {
	uint8_t supply_projectile_id;
	uint8_t supply_robot_id;
	uint8_t supply_num; 
} ext_supply_projectile_booking_t;

//0x0201  ����������״̬
typedef struct {
	uint8_t robot_id;/* ������ID 
									  * 1�� ��Ӣ��
									  * 2�� �칤��
									  * 3/4/5���첽��
									  * 6�� �����
									  * 7�� ���ڱ�
										  * 11����Ӣ��
										  * 12��������
										  * 13/14/15��������		
										  * 16  ������
										  * 17  ���ڱ�*/
	uint8_t robot_level; /* �����˵ȼ� 1/2/3*/
	uint16_t remain_HP; /* ������ʣ��Ѫ�� */
	uint16_t max_HP; /* ����������Ѫ�� */
	uint16_t shooter_heat0_cooling_rate; /* ������17mmǹ��ÿ����ȴֵ */
	uint16_t shooter_heat0_cooling_limit; /* ������17mmǹ���������� */
	uint16_t shooter_heat1_cooling_rate;	/* ������42mmǹ��ÿ����ȴֵ */
	uint16_t shooter_heat1_cooling_limit; /* ������42mmǹ���������� */
	uint8_t mains_power_gimbal_output : 1; /* ��̨��Դ������ */
	uint8_t mains_power_chassis_output : 1; /* ���̵�Դ������ */
	uint8_t mains_power_shooter_output : 1; /* ����Ħ���ֵ�Դ������ */
} ext_game_robot_state_t;

//0x0202 ʵʱ������������
typedef struct {
	uint16_t chassis_volt; /* ���������ѹ mV */
	uint16_t chassis_current; /* ����������� mA*/
	float chassis_power; /* ����������� W */
	uint16_t chassis_power_buffer; /* ���̹��ʻ��� J */
	uint16_t shooter_heat0; /* 17mmǹ������ */
	uint16_t shooter_heat1; /* 42mmǹ������ */
} ext_power_heat_data_t;

//0x0203 ������λ���Լ�ǹ�ڳ���Ƕ�
typedef struct {
	float x; 
	float y; 
	float z; 
	float yaw; 
} ext_game_robot_pos_t;

//0x0204 ����������
typedef struct { 
	union
  {
    uint8_t can_data;
    struct 
    {
			uint8_t blood : 1; 				/* bit0  ��Ѫ״̬ */
			uint8_t heat : 1; 				/* bit1  ������ȴ�ӱ� */
			uint8_t defence : 1; 				/* bit2  �����ӳ� */
			uint8_t attack : 1; 				/* bit3  �����ӳ� */
																/* ���ౣ�� */
    } type;
  }buff;
}ext_buff_musk_t;

//0x0205 ���л���������״̬ ֻ�п��л������ܹ�����
typedef struct {
	uint8_t energy_point; /* ���۵����� */
	uint8_t attack_time; /* ʣ��ɹ���ʱ�� */
}aerial_robot_energy_t;

//0x0206 �˺�״̬
typedef struct {
	uint8_t armor_id : 4; /* װ���˺�ʱΪװ��ID ����Ϊ0 */
	uint8_t hurt_type : 4; /* 0 װ���˺� 
												  * 1 ģ�����߿�Ѫ 
												  * 2 ǹ�ڳ�������Ѫ 
												  * 3 ���̳����ʿ�Ѫ */
} ext_robot_hurt_t;

//0x0207 ʵʱ�����Ϣ
typedef struct {
	uint8_t bullet_type; /* �ӵ����� 17mmΪ1 42mmΪ2*/
	uint8_t bullet_freq; /* ��Ƶ */
	float bullet_speed; /* ���� */
} ext_shoot_data_t;

//0x0301 �����˼佻������
typedef struct { 
	uint16_t data_cmd_id; /* ���ݶε�����ID */
	uint16_t send_ID; /* �����ߵ�ID ����1�� ��IDΪ 1*/
	uint16_t receiver_ID; /* �����ߵ�ID */
}ext_student_interactive_header_data_t;

//�ͻ�����Ϣ ����ID 0xD180
/*���Ϳͻ�����Ϣʱ������IDΪ0xD180 
										������ID Ϊ���͵Ļ�����ID 
										�����ߵ�ID Ϊ�����˶�Ӧ�ͻ��˵�ID*/
typedef struct{
	float data1; /* �Զ��帡������ 1 */ 
	float data2; /* �Զ��帡������ 2 */
	float data3; /* �Զ��帡������ 3 */
	uint8_t masks; /* �Զ����λ���� λ0-5 �ֱ����������������ָʾ�� */
} client_custom_data_t;


//������֮��ͨ������ 
/* ���ͻ�����֮��ͨ����Ϣʱ�� ����ID Ϊ0x0200-0x02FF 
															������ID Ϊ���ͻ����˵�ID 
															������ID Ϊ���ܻ����˵�ID 
															���ݶε��ֽ���ҪС��113 */
typedef struct
{
	uint8_t data[112];
} robot_interactive_data_t;

extern data_to_judge_t															ToJudge_Data;

extern ext_game_state_t 														Judge_GameState;
extern ext_game_result_t     												Judge_GameResult;
extern ext_game_robot_survivors_t 									Judge_GameRobotSurvivors;

extern ext_event_data_t 														Judge_EventData;
extern ext_supply_projectile_action_t     					Judge_SupplyProjectileAction;
extern ext_supply_projectile_booking_t 							Judge_SupplyProjectileBooking;

extern ext_game_robot_state_t												Judge_GameRobotState;
extern ext_power_heat_data_t												Judge_PowerHeatData;
extern ext_game_robot_pos_t													Judge_GameRobotPos;
extern ext_buff_musk_t															Judge_BuffMusk;
extern aerial_robot_energy_t												Judge_AerialRobotEnergy;
extern ext_robot_hurt_t															Judge_RobotHurt;
extern ext_shoot_data_t  														Judge_ShootData;
extern ext_student_interactive_header_data_t				Judge_StudentInfoHeader;
extern client_custom_data_t													Judge_ClientData;
extern robot_interactive_data_t  										Judge_RobotInterfaceData;

#endif
void judgesystem_param_init(void);
void RobotSendMsgToClient(float data1,float data2,float data3,uint8_t mask);
#endif
