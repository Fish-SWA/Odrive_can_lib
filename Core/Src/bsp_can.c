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
#include "com.h"
#include "usart.h"

#define printf(...) cprintf(&huart1,__VA_ARGS__)


Odrive_motor_measure motor[3] = {0};


/*******************************************************************************************
  * @Func		my_can_filter_init
  * @Brief    CAN1和CAN2滤波器配置
  * @Param		CAN_HandleTypeDef* hcan
  * @Retval		None
  * @Date     2015/11/30
 *******************************************************************************************/
void my_can_filter_init_recv_all(CAN_HandleTypeDef* _hcan)
{
	//can1 &can2 use same filter config
	CAN_FilterTypeDef		CAN_FilterConfigStructure;

//	CAN_FilterConfigStructure.FilterNumber = 0;
	CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
	CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN_FilterConfigStructure.FilterIdHigh = 0x0000;
	CAN_FilterConfigStructure.FilterIdLow = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;
	CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
	CAN_FilterConfigStructure.FilterBank = 14;//can1(0-13)和can2(14-27)分别得到一半的filter
	CAN_FilterConfigStructure.FilterActivation = ENABLE;

	if(HAL_CAN_ConfigFilter(_hcan, &CAN_FilterConfigStructure) != HAL_OK)
	{
//		while(1); //show error!
	}
}

uint32_t FlashTimer;
/*******************************************************************************************
  * @Func			void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
  * @Brief    HAL库中标准的CAN接收完成回调函数，需要在此处理通过CAN总线接收到的数据
  * @Param		
  * @Retval		None 
  * @Date     2015/11/24
 *******************************************************************************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
	CAN_RxHeaderTypeDef		Rx1Message;
	uint8_t Data[8];
	uint8_t MotorID;
	
	Rx1Message.StdId = 0x201;  
	Rx1Message.ExtId = 0;  
	Rx1Message.IDE = CAN_ID_STD;
	Rx1Message.RTR = CAN_RTR_DATA;
	Rx1Message.DLC = 0x08;

	if(HAL_GetTick() - FlashTimer>500)
	{
		HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		FlashTimer = HAL_GetTick();
	}

	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Rx1Message, Data);

	//输出CAN接收到的所有信息
	// printf("ID:%x, \t\tData:%x, %x, %x, %x, %x, %x, %x, %x\n", Rx1Message.StdId, Data[0], 
	// 				Data[1], Data[2], Data[3], Data[4], Data[5], Data[6], Data[7]);

	//处理收到的编码器数据
	if(Rx1Message.StdId == (0x00<<5 | 0x0A)) motor[0].encoder = Data[5]<<8 | Data[4];
	if(Rx1Message.StdId == (0x01<<5 | 0x0A)) motor[1].encoder = Data[5]<<8 | Data[4];
	if(Rx1Message.StdId == (0x02<<5 | 0x0A)) motor[2].encoder = Data[5]<<8 | Data[4];

	/*#### add enable can it again to solve can receive only one ID problem!!!####**/
	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}


/*向Odrive发送CAN信息*/
uint8_t Odrv_CAN_Send_Msg(CAN_HandleTypeDef *hcan,uint16_t StdID,uint8_t *msg,uint8_t len,uint8_t Frame_type)
{
	CAN_TxHeaderTypeDef TxHeader;
    uint8_t index=0;
    uint32_t TxMailbox;   //邮箱
    uint8_t send_buf[8] = {0};
    TxHeader.StdId=StdID;        //标准标识符
    TxHeader.ExtId=0;        //扩展标识符(29位)
    TxHeader.IDE=CAN_ID_STD;    //使用标准帧
    TxHeader.RTR = Frame_type != CAN_RTR_REMOTE ? CAN_RTR_DATA : CAN_RTR_REMOTE;  //数据帧为发送数据，远程帧为请求返回数据
    //TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC=len;
    /*****填充消息******/
    for ( index = 0; index < len; index++) {
            send_buf[index] = msg[index];
    }
    /*****发送消息*****/
    if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, send_buf, &TxMailbox) != HAL_OK)//发送
    {
        return 1;
    }
    return 0;
}

/*设置电机扭矩*/
void Odrv_set_motor_torque(CAN_HandleTypeDef* hcan, int axis_id, float torque_set)
{
	uint8_t can_msg[8] = {0};
	unsigned int* float_bit_data = (unsigned int*)(&torque_set);
	can_msg[3] = (*float_bit_data>>24)&0xff;
	can_msg[2] = (*float_bit_data>>16)&0xff;
	can_msg[1] = (*float_bit_data>>8)&0xff;
	can_msg[0] = (*float_bit_data)&0xff;
	//printf("%d, %d, %d, %d\n", can_msg[0], can_msg[1], can_msg[2], can_msg[3]);
	Odrv_CAN_Send_Msg(&hcan, axis_id<<5 | 0x00E, can_msg, 8, CAN_RTR_DATA);
}
