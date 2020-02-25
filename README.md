# 概述
* an doc readme
## 软件环境
* IDE：MDK-ARM V5
* package version: STM32Cube FW_F4 V1.24.1
* FreeRTOS version: 10.0.1
* CMSIS-RTOS version: 1.02
## 编程规范
* gimbal_task，chassis_task,shoot_task,imu_update_task是强实时性任务，采用os_Delay_until()调度，代码区块使用临界区进行保护， 优先级由高到低排列，禁止被其他任务抢占阻塞
* 其余任务采用os_Delay()调度，实时性受任务时间影响，可以被其他任务抢占
## 陀螺仪校准方法
* 遥控器左摇杆上，右拨杆中，两个摇杆打成内八，维持2秒，进入陀螺仪校准模式，采集陀螺仪数据并写入falsh
## 模块离线说明
* 按照优先级顺序显示LED，当有离线发生，10个LED熄灭， 按照离线顺序显示 若离线，对应的LED亮起 对应关系如下
| LED亮起 | 对应设备 |
| :--: | :--: |
| LED_RED    | 				dbus |
|LED_GREEN  |  			gimbal-imu|
|LED_H			|				  board_imu|
|LED_G			|					chas-imu|
|LED_F			|					yaw|
|LED_E      |          pitch|
|LED_D		  |				trigger|
|LED_C			|				chassis-motor|
|LED_B			|				nuc|
|LED_A			|				judge|
| 作者   | 负责部分       | 微信号               |
| ------ | -------------- | -------------------- |
| 唐欣阳 | 自瞄装甲板识别 | xinyang_tang         |
| 卫志坤 | 自瞄装甲板识别 |                      |
| 孙加桐 | 能量机关识别   | SJTxixiliadorabarryU |
| 罗嘉鸣 | 能量机关识别   |                      |
