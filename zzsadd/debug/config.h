#ifndef CONFIG_H
#define CONFIG_H
/***************** for robot *****************/
#define ROBOT_ID                        1//������id
#define NO_JUDGE_SYSTEM                 1//�޲���ϵͳ
/******************* chassis *****************/
#define NO_POWER_LIMIT                  1//�Ƿ�����������
/****************** onboard imu *************/
#define USE_MAHONY_METHOD               0//�Ƿ�ʹ��mahony�㷨����
#define USE_LPF_ACCELARATION            1//���ٶ������Ƿ񾭹���ͨ�˲�
#define USE_IST8310_DATA                1//�Ƿ�ʹ�õشż�����
#define GET_OFFSET_TIME                 300//��ʼ��ʱ�ɼ�ƫ�õ�ʱ��
#define DEFAULT_IMU_TEMP                50//���ȵ����Ŀ���¶�
/***************** adjust pid ************/
/*˵������������Ӧ�������£������ڵ���ʱʹ�á�PID����˫��ʱ�⻷��PID��,�ڻ���pid����
��Q��= p+1  ��W��= i+0.1 ��E��= d+1    ��U��= P+1  ��I��= I+0.1 ��O��= D+1
��A��= p-1  ��S��= i-0.1 ��D��= d-1    ��J��= P-1  ��K��= I-0.1 ��L��= D-1
��R��= p=i=d=0                         ��P�� = P=I=D=0
*/
#define CHASSIS_WHEEL_PID_DEBUG         0//���ڵ��̵���ٶȻ�pid
#define CHASSIS_FOLLOW_PID_DEBUG        0//���ڵ��̸���pid
#define CHASSIS_SWING_PID_DEBUG         0//���ڵ���Ť��pid
#define YAW_PID_DEBUG                   0//����yaw pid
#define PITCH_PID_DEBUG                 0//����pitch pid
#define TRANS_PID_DEBUG                 0//���ڲ��̵�� pid
#define FRIC_PID_DEBUG                  0//����Ħ���� pid
#define YAW_ROBOT_PID_DEBUG             0//���ڻ��������yaw �⻷pid
#define PITCH_ROBOT_PID_DEBUG           0//���ڻ��������pitch �⻷pid
#define YAW_FAN_PID_DEBUG               0//���ڻ�����������yaw �⻷pid
#define PITCH_FAN_PID_DEBUG             0//���ڻ�����������pitch �⻷pid
#define YAW_VISION_SPD_DEBUG            0//�����Ӿ�����yaw �ڻ�pid
#define PITCH_VISION_SPD_DEBUG          0//�����Ӿ�����pitch �⻷pid
/******************** vision **************/
#define NO_YAW_COMP         						1//ȡ��yaw����
#define NO_PIT_COMP         						1//ȡ��pitch����
#define NO_FAN_PIT_COMP     						1//ȡ�����pitch����
#define H1        											2169//�������ؾ����߶�  		
#define H2						      						850 //���������߶�			
#define H3          										350	//����ǹ�ھ��ŵص�߶�
#define FAN_DIS             						7300.0f//�������ؾ�����ˮƽ����
/******************** show message *************/
#define USE_PLOT                        0//�Ƿ�ʹ�������ƴ���λ����ͼ
#define SHOW_MONITOR                    0//��ʾģ���쳣���
#define SHOW_DBUS                       0//��ʾң����ͨ�������ֵ
#define SHOW_CHASSIS                    0//��ʾ���������Ϣ
#define SHOW_SHOOT                      0//��ʾ������������Ϣ
#define SHOW_GIMBAL                     0//��ʾ��̨�����Ϣ
#define SHOW_JUDGE                      0//��ʾ����ϵͳ�����Ϣ
#define SHOW_VISION                     0//��ʾ�Ӿ������Ϣ
#endif
