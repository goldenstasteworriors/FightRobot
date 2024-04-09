/******************************************************************************
/// @brief
/// @copyright Copyright (c) 2017 <dji-innovations, Corp. RM Dept.>
/// @license MIT License
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction,including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense,and/or sell
/// copies of the Software, and to permit persons to whom the Software is furnished
/// to do so,subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in
/// all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
/// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
/// THE SOFTWARE.
*******************************************************************************/

#include "can.h"
#include "bsp_can.h"

extern CAN_HandleTypeDef hcan;
//moto_measure_t moto_pit;
//moto_measure_t moto_yaw;
//moto_measure_t moto_poke;	//拨单电机
moto_measure_t moto_chassis[4] = {0};//4 chassis moto
moto_measure_t moto_info;


void get_total_angle(moto_measure_t *p);
void get_moto_offset(moto_measure_t *ptr, CAN_HandleTypeDef* hcan);
 




float ZGyroModuleAngle;

CAN_TxHeaderTypeDef g_canx_txheader;    /* 发送参数句柄 */
CAN_RxHeaderTypeDef g_canx_rxheader;    /* 接收参数句柄 */
uint8_t moto_status[8]={0};
uint8_t rec_len=0;
uint16_t rec_id=0;
uint8_t rec_buf[8];
extern uint8_t can_get_flag;
//过滤器设置
void CAN_Filter_Config()
{
	CAN_FilterTypeDef CAN_FilterInitStructure;

	CAN_FilterInitStructure.FilterBank=0;
	CAN_FilterInitStructure.FilterMode=CAN_FILTERMODE_IDMASK;
	CAN_FilterInitStructure.FilterScale=CAN_FILTERSCALE_32BIT;
	CAN_FilterInitStructure.FilterIdHigh=0x0000;
	CAN_FilterInitStructure.FilterIdLow=0x0000;
	CAN_FilterInitStructure.FilterMaskIdHigh=0x0000;
	CAN_FilterInitStructure.FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.FilterFIFOAssignment=CAN_RX_FIFO0;
	CAN_FilterInitStructure.FilterActivation=CAN_FILTER_ENABLE;
	HAL_CAN_ConfigFilter(&hcan,&CAN_FilterInitStructure);

}



//发送函数
uint8_t can_write(uint32_t id, uint8_t *msg)
{

  uint32_t TxMailbox = CAN_TX_MAILBOX0;
    
  g_canx_txheader.StdId = id;         /* 标准标识符 */
//  g_canx_txheader.ExtId = id;         /* 扩展标识符(29位) 标准标识符情况下，该成员无效*/
  g_canx_txheader.IDE = CAN_ID_STD;   /* 使用标准标识符 */
  g_canx_txheader.RTR = CAN_RTR_DATA; /* 数据帧 */
  g_canx_txheader.DLC = 8;
	
  if (HAL_CAN_AddTxMessage(&hcan, &g_canx_txheader, msg, &TxMailbox) != HAL_OK) /* 发送消息 */
  {
    return 1;
  }

  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) != 3); /* 等待发送完成,所有邮箱(有三个邮箱)为空 */
	printf("can send: ");
		for(int i=0;i<8;i++)
	{
	printf("%d  ",msg[i]);
	if(i==7){printf("\r\n");}
	}
  return 0;

}

//接收函数
uint8_t can_receive( uint8_t *buf)
{
  if (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) == 0)     /* 没有接收到数据 */
  {
    return 0;
  }

	HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &g_canx_rxheader, buf);
//	printf("id:");
//	printf("%d\r\n",g_canx_rxheader.StdId);
	rec_id=g_canx_rxheader.StdId;
  return g_canx_rxheader.DLC;
}


void set_moto_current(CAN_HandleTypeDef* hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4){
	uint32_t TxMailbox = CAN_TX_MAILBOX0;
	
	g_canx_txheader.DLC=8;
	g_canx_txheader.IDE=CAN_ID_STD;
	g_canx_txheader.RTR=CAN_RTR_DATA;
	g_canx_txheader.StdId=0x200;
	

	moto_status[0] = iq1 >> 8;
	moto_status[1] = iq1;
	moto_status[2] = iq2 >> 8;
	moto_status[3] = iq2;
	moto_status[4] = iq3 >> 8;
	moto_status[5] = iq3;
	moto_status[6] = iq4 >> 8;
	moto_status[7] = iq4;
	
	HAL_CAN_AddTxMessage(hcan,&g_canx_txheader,moto_status,&TxMailbox);
}



/*******************************************************************************************
  * @Func			void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
  * @Brief    这是一个回调函数,都不用声明
  * @Param		
  * @Retval		None 
  * @Date     2015/11/24
 *******************************************************************************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *_hcan)
{
		rec_len=can_receive(rec_buf);

				static u8 i;
				i = rec_id - CAN_3510Moto1_ID;
				
				moto_chassis[i].msg_cnt++ <= 50	?	get_moto_offset(&moto_chassis[i], _hcan) : get_moto_measure(&moto_chassis[i], _hcan);
				get_moto_measure(&moto_info, _hcan);
				can_get_flag=i;
				//get_moto_measure(&moto_chassis[i], _hcan);
	
//		switch(rec_id){
//		case CAN_3510Moto1_ID:
//		case CAN_3510Moto2_ID:
//		case CAN_3510Moto3_ID:
//		case CAN_3510Moto4_ID:
//			{
//				
//			}
//			break;
//		
//		
//	}
		

	//hcan1.Instance->IER|=0x00008F02;
	/*#### add enable can it again to solve can receive only one ID problem!!!####**/
HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	/*#### add enable can it again to solve can receive only one ID problem!!!####**/
}

//can接收中断
\


/*******************************************************************************************
  * @Func			void get_moto_measure(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
  * @Brief    接收云台电机,3510电机通过CAN发过来的信息
  * @Param		
  * @Retval		None
  * @Date     2015/11/24
 *******************************************************************************************/
void get_moto_measure(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
{
//	u32  sum=0;
//	u8	 i = FILTER_BUF_LEN;
	
	/*BUG!!! dont use this para code*/
//	ptr->angle_buf[ptr->buf_idx] = (uint16_t)(hcan->pRxMsg->Data[0]<<8 | hcan->pRxMsg->Data[1]) ;
//	ptr->buf_idx = ptr->buf_idx++ > FILTER_BUF_LEN ? 0 : ptr->buf_idx;
//	while(i){
//		sum += ptr->angle_buf[--i];
//	}
//	ptr->fited_angle = sum / FILTER_BUF_LEN;
	ptr->last_angle = ptr->angle;
	ptr->angle = (uint16_t)(rec_buf[0]<<8 | rec_buf[1]) ;
	ptr->real_current  = (int16_t)(rec_buf[2]<<8 | rec_buf[3]);
	ptr->speed_rpm = ptr->real_current;	
	ptr->given_current = (int16_t)(rec_buf[4]<<8 | rec_buf[5])/-5;
	ptr->hall = rec_buf[6];
	if(ptr->angle - ptr->last_angle > 4096)
		ptr->round_cnt --;
	else if (ptr->angle - ptr->last_angle < -4096)
		ptr->round_cnt ++;
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
}

/*this function should be called after system+can init */
void get_moto_offset(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
{
	ptr->angle = (uint16_t)(rec_buf[0]<<8 | rec_buf[1]) ;
	ptr->offset_angle = ptr->angle;
}

#define ABS(x)	( (x>0) ? (x) : (-x) )
/**
*@bref 电机上电角度=0， 之后用这个函数更新3510电机的相对开机后（为0）的相对角度。
	*/
void get_total_angle(moto_measure_t *p){
	
	int res1, res2, delta;
	if(p->angle < p->last_angle){			//可能的情况
		res1 = p->angle + 8192 - p->last_angle;	//正转，delta=+
		res2 = p->angle - p->last_angle;				//反转	delta=-
	}else{	//angle > last
		res1 = p->angle - 8192 - p->last_angle ;//反转	delta -
		res2 = p->angle - p->last_angle;				//正转	delta +
	}
	//不管正反转，肯定是转的角度小的那个是真的
	if(ABS(res1)<ABS(res2))
		delta = res1;
	else
		delta = res2;

	p->total_angle += delta;
	p->last_angle = p->angle;
}

	
