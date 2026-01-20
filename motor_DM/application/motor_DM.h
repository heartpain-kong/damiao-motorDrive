#ifndef __MOTOR_DM_H__
#define __MOTOR_DM_H__

#include "struct_typedef.h"
#include "bsp_can.h"

#define motor_DM_N 2
#define DM_CAN hcan1

enum motor_DM_model{
	MOTOR_DM_J10010L = 0,
	MOTOR_DM_J4310	,
	
};
typedef struct __pack
{
    uint8_t id    		 :5;      // 电机ID
    uint8_t model		 :3;
	
} Motor_DM_Init;



typedef struct{
	uint8_t MError;
	fp32 Angle;
	fp32 W;
	fp32 T;
	fp32 mos_Temp;
	fp32 motor_Temp;
}user_DM_recv;

//公式: T= t_ff+kp*(angle-last_angle)+kd*(w-last_w);
typedef struct{
	fp32 angle;
	fp32 w;
	fp32 t_ff;
	fp32 Kp;
	fp32 Kd;
}user_DM_send;

typedef struct{
	uint8_t tx_Data[8];
	user_DM_recv motor_recv;
	user_DM_send *motor_send;
	uint8_t id;
	uint8_t enable;
	uint8_t model;
}motor_DM;

/**
  * @brief          电机初始化
  * @param[in]      none
  * @retval         none
  */
void motor_DM_init();


/**
  * @brief          电机发送数据的指针传输
  * @param[in]      id      电机id值
  * @param[in]      *motor  电机指针
  * @retval         none
  */
void motor_DM_send_init(uint8_t id,user_DM_send *motor);

/**
  * @brief          电机接收数据的指针传输
  * @param[in]      id      电机id值
  * @retval         const user_DM_recv 类型指针
  */
const user_DM_recv *motor_DM_recv_return(uint8_t id);
/**
  * @brief          电机使能
  * @param[in]      id      电机id值
  * @retval         none
  */
void motor_DM_enable(uint8_t id);

/**
  * @brief          电机失能
  * @param[in]      id      电机id值
  * @retval         none
  */
void motor_DM_lose(uint8_t id);
/**
  * @brief          电机发送数据传递
  * @param[in]      id      电机id值
  * @retval         none
  */
void motor_DM_send(uint8_t id);

#endif
