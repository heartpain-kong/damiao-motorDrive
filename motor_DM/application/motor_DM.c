#include "motor_DM.h"
#include "main.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

static Motor_DM_Init motor_DM_id[motor_DM_N]={{1,MOTOR_DM_J10010L},{2,MOTOR_DM_J4310}};
 uint8_t motor_DM_id_seat[8];
motor_DM motor_DM_send_recv[motor_DM_N];

 CAN_TxHeaderTypeDef  motor_DM_tx;

/**
  * @brief          int 转 fp32 
  * @param[in]      x_int 要转换的数据
  * @param[in]      x_min 要转换的数据的最小值
  * @param[in]      x_max 要转换的数据的最大值
  * @param[in]      bits  要转换的数据位数
  * @retval         none
  */
static fp32 uint_to_float(uint16_t x_int, fp32 x_min, fp32 x_max, uint8_t bits){
	fp32 span=x_max-x_min;
	fp32 offset = x_min;
	return ((fp32)x_int)*span/((fp32)((1<<bits)-1)) + offset;
}

/**
  * @brief          fp32 转 int 
  * @param[in]      x     要转换的数据
  * @param[in]      x_min 要转换的数据的最小值
  * @param[in]      x_max 要转换的数据的最大值
  * @param[in]      bits  要转换的数据位数
  * @retval         none
  */
static uint16_t float_to_uint(fp32 x, fp32 x_min, fp32 x_max, uint8_t bits){
	fp32 span=x_max-x_min;
	fp32 offset = x_min;
	return (uint16_t) ((x-offset)*((fp32)((1<<bits)-1))/span) ;
}

/**
  * @brief          电机初始化
  * @param[in]      none
  * @retval         none
  */
void motor_DM_init(){
	can_filter_init();
	motor_DM_tx.IDE = CAN_ID_STD;
	motor_DM_tx.RTR = CAN_RTR_DATA;
	motor_DM_tx.DLC = 8;
	for(uint8_t i=0;i<motor_DM_N;++i){
		motor_DM_send_recv[i].id=motor_DM_id[i].id;
		motor_DM_id_seat[motor_DM_id[i].id] = i;
		motor_DM_send_recv[i].model = motor_DM_id[i].model;
		motor_DM_send_recv[i].enable = 0;
	}
	for(uint8_t i=0;i<motor_DM_N;++i){
		while(motor_DM_send_recv[i].motor_recv.mos_Temp==0){
			motor_DM_enable(motor_DM_id[i].id);
			HAL_Delay(1);
		}
		motor_DM_send_recv[i].enable = 1;
		motor_DM_send_recv[i].motor_send->angle = motor_DM_send_recv[i].motor_recv.Angle;
	}
}


/**
  * @brief          电机发送数据的指针传输
  * @param[in]      id      电机id值
  * @param[in]      *motor  电机指针
  * @retval         none
  */
void motor_DM_send_init(uint8_t id,user_DM_send *motor){
	motor->angle = motor_DM_send_recv[motor_DM_id_seat[id]].motor_recv.Angle;
	motor_DM_send_recv[motor_DM_id_seat[id]].motor_send = motor;
}

/**
  * @brief          电机接收数据的指针传输
  * @param[in]      id      电机id值
  * @retval         const user_DM_recv 类型指针
  */
const user_DM_recv *motor_DM_recv_return(uint8_t id){
	return &motor_DM_send_recv[motor_DM_id_seat[id]].motor_recv;
}

/**
  * @brief          电机发送数据
  * @param[in]      id      电机id值
  * @retval         none
  */
static void motor_DM_Handle_send(uint8_t id){
	uint32_t motor_LZ_send_box;
	motor_DM_tx.StdId = motor_DM_send_recv[motor_DM_id_seat[id]].id;
	HAL_CAN_AddTxMessage(&DM_CAN,&motor_DM_tx,motor_DM_send_recv[motor_DM_id_seat[id]].tx_Data,&motor_LZ_send_box);
}

/**
  * @brief          电机使能
  * @param[in]      id      电机id值
  * @retval         none
  */
void motor_DM_enable(uint8_t id){
	for(uint8_t i=0;i<7;++i){
		motor_DM_send_recv[motor_DM_id_seat[id]].tx_Data[i]=0xFF;
	}
	motor_DM_send_recv[motor_DM_id_seat[id]].tx_Data[7]=0xFC;
	motor_DM_Handle_send(id);
}

/**
  * @brief          电机失能
  * @param[in]      id      电机id值
  * @retval         none
  */
void motor_DM_lose(uint8_t id){
	for(uint8_t i=0;i<7;++i){
		motor_DM_send_recv[motor_DM_id_seat[id]].tx_Data[i]=0xFF;
	}
	motor_DM_send_recv[motor_DM_id_seat[id]].tx_Data[7]=0xFD;
	motor_DM_Handle_send(id);
}

/**
  * @brief          电机零点设置 会失能 再使能
  * @param[in]      id      电机id值
  * @retval         none
  */
void motor_DM_zero(uint8_t id){
	motor_DM_lose(id);
	motor_DM_send_recv[motor_DM_id_seat[id]].tx_Data[7]=0xFE;
	motor_DM_Handle_send(id);
	motor_DM_enable(id);
}

/**
  * @brief          电机发送数据处理
  * @param[in]      id      电机id值
  * @param[in]      model   电机类型
  * @retval         none
  */
static void motor_DM_send_handle(uint8_t id,uint16_t *motor,uint8_t model){
	motor_DM *motor_st;
	motor_st = motor_DM_send_recv + motor_DM_id_seat[id];
	motor[3]=float_to_uint(motor_st->motor_send->Kp,0.0f,500.0f,12);
	motor[4]=float_to_uint(motor_st->motor_send->Kd,0.0f,5.0f,12);
	switch(model){
		case MOTOR_DM_J10010L:
			motor[0]=float_to_uint(motor_st->motor_send->angle,-12.57f,12.57f,16);
			motor[1]=float_to_uint(motor_st->motor_send->w,-50.0f,50.0f,12);
			motor[2]=float_to_uint(motor_st->motor_send->t_ff,-100.0f,100.0f,12);
		break;
		case MOTOR_DM_J4310 :
			motor[0]=float_to_uint(motor_st->motor_send->angle,-12.57f,12.57f,16);
			motor[1]=float_to_uint(motor_st->motor_send->w,-30.0f,30.0f,12);
			motor[2]=float_to_uint(motor_st->motor_send->t_ff,-30.0f,30.0f,12);
			
		break;
		default:
		{
			break;
		}
	}
	
}

/**
  * @brief          电机发送数据传递
  * @param[in]      id      电机id值
  * @retval         none
  */
void motor_DM_send(uint8_t id){
	motor_DM *motor_st;
	motor_st = motor_DM_send_recv + motor_DM_id_seat[id];
	uint16_t motor_DM_send1[5]; //angle w t_ff kp kd
	motor_DM_send_handle(id,motor_DM_send1,motor_st->model);
	motor_st->tx_Data[0]=motor_DM_send1[0]>>8;
	motor_st->tx_Data[1]=motor_DM_send1[0];
	motor_st->tx_Data[2]=motor_DM_send1[1]>>4;
	motor_st->tx_Data[3]=(motor_DM_send1[1]&0xF)<<4|(motor_DM_send1[3]>>8);
	motor_st->tx_Data[4]=motor_DM_send1[3];
	motor_st->tx_Data[5]=motor_DM_send1[4]>>4;
	motor_st->tx_Data[6]=(motor_DM_send1[4]&0xF)<<4|(motor_DM_send1[2]>>8);
	motor_st->tx_Data[7]=motor_DM_send1[2];
	motor_DM_Handle_send(id);
}

/**
  * @brief          电机接收
  * @param[in]      id      电机id值
  * @param[in]      model   电机类型
  * @param[in]      *data   接受数组
  * @retval         none
  */
static void motor_DM_recv_data(uint8_t id,uint8_t model,uint8_t *data){
	motor_DM *motor_st;
	motor_st = motor_DM_send_recv + motor_DM_id_seat[id];
	motor_st->motor_recv.mos_Temp 	= (fp32)data[6];
	motor_st->motor_recv.motor_Temp  = (fp32)data[7];
	motor_st->motor_recv.MError      = data[0]>>4;
	switch(model){
		case MOTOR_DM_J10010L:
			motor_st->motor_recv.Angle  = uint_to_float((data[1]<<8|data[2]),-12.57f,12.57f,16);
			motor_st->motor_recv.W  = uint_to_float((data[3]<<4|data[4]>>4),-50.0f,50.0f,12);
			motor_st->motor_recv.T  =uint_to_float((data[4]&0x0F)<<8|data[5],-100.0,100.0f,12);
		break;
		case MOTOR_DM_J4310:
			motor_st->motor_recv.Angle  = uint_to_float((data[1]<<8|data[2]),-12.57f,12.57f,16);
			motor_st->motor_recv.W  = uint_to_float((data[3]<<4|data[4]>>4),-30.0f,30.0f,12);
			motor_st->motor_recv.T  = uint_to_float((data[4]&0x0F)<<8|data[5],-30.0,30.0f,12);
		
		break;
		default:
		break;
	}
	

}

/**
  * @brief          can接收回调函数
  * @param[in]      hcan hcan指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{	
	if(hcan == &DM_CAN){
		uint8_t rt_motor_id;
		CAN_RxHeaderTypeDef rx_header;
		uint8_t rx_data[8];
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
		switch(rx_header.StdId){
			case 0:
				rt_motor_id=rx_data[0]&0x0F;
				motor_DM_recv_data(rt_motor_id,motor_DM_send_recv[motor_DM_id_seat[rt_motor_id]].model,rx_data);
			break;
			default:
			{
				break;
			}
		}	
		
		
	}
	
}

