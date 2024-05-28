/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "pid.h"
#include "mytype.h"
#include "oled.h"
#include "bmp.h"
#include "stdio.h"
#include "math.h"
#include "usart.h"
//#include "tim.h"
#include "main.h"
#include "sys.h"
#include "usart.h"
#include "lcd.h"
#include "adc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define touch_distance 100//待定的推到物块时openmv检测到的距离
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern CAN_HandleTypeDef hcan;
extern moto_measure_t moto_chassis[4];
extern uint8_t aRxBuffer;
extern uint8_t RxData;//串口3收到的数据
extern bool find;

extern int tag_id;
extern int x_translation;
extern int x_rotation;
extern keys key[4];

s16 torque=500;
uint8_t data[8]={1,2,3,4,5,6,7,8};
uint8_t rxcnt=0;
bool run=0;

uint8_t can_get_flag=4;//要打印需要该值在0-3之间
int set_v,set_spd[4];

char text[20];

bool go_straight=0;
double front_infrafot_distanceL=0; //左边的
double front_infrafot_distanceR=0; //右边的
double left_infrafot_distance=0; 
double right_infrafot_distance=0;
double front_dis_err=0;
uint8_t outofstage=0;
uint16_t v_push_box=200;
uint8_t x = 0;

//传感器量
uint16_t ADValue1;
uint16_t ADValue2;
uint16_t ADValue3;
uint16_t ADValue4;
float Voltage1;
float Voltage2;
float Voltage3;
float Voltage4;
float L1;
float L2;
float L3;
float L4;
char text[20];
uint16_t GD1;
uint16_t GD2;
uint16_t GD3;
uint16_t GD4;
uint16_t GD5;

uint16_t HD1;
uint16_t HD2;

uint32_t ticktime=0;
uint32_t spintime=1000;
uint32_t chaoshitime=100000;
uint16_t tuikuaitime=50000;
uint16_t long_time=0;
uint16_t shangtaitime1=500;//下台后向前撞以对正的时间
uint16_t shangtaitime2=2000;//下台对正后冲上台的时间
uint16_t	zhengduihuiducha=700;//正对灰度差，待测
uint16_t zhongxinhuidu1=2600;//灰度1的中心灰度，待测
uint16_t zhongxinhuidu2=3500;//灰度2的中心灰度，待测
uint16_t xiataihuidu1=1800;//灰度1的下台灰度，待测
uint16_t xiataihuidu2=2000;//灰度2的下台灰度，待测


//实际判断量
bool enermy_find=0;
bool enermy_front=0;
bool turn_left=0;
bool turn_right=0;
bool xuntaitest=0;
bool xuntai=0;
bool zhengduitest=0;
bool tuichetest=0;
bool keep_spin=0;
bool xiataiflag=0;
bool huizhong=0;
bool diaotai=0;
bool zhengduikuai=0;
bool find_then_zhengduikuai=1;
bool openmvxuntai=1;
bool x_rotation_flag=1;

uint8_t kuai=2;

//debug使用
	int vel=1800;
	uint8_t state=0;//0:前进   1:后退   2:前进
	bool state_change=0;
	uint32_t time=0;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId moveHandle;
osThreadId pushBoxHandle;
osThreadId scanTaskHandle;
osThreadId openmvConnectHandle;
osThreadId tsetHandle;
osThreadId tickHandle;
osThreadId keyHandle;
osThreadId xiataiHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
double max(double L)	
{
	if(L<80)
		return L;
	else 
		return 81.000;
}

//转圈(右)
void spin(int vel)
{
		set_spd[1]=vel;
		set_spd[3]=vel;
		set_spd[2]=vel;
		set_spd[0]=vel;
}

//向前
void forward(int vel)
{
		set_spd[1]=-vel;
		set_spd[3]=vel;
		set_spd[2]=vel;
		set_spd[0]=-vel;
}

void forward_spin(int vel)
{
		set_spd[1]=-(vel);
		set_spd[3]=vel-x_translation/10;
		set_spd[2]=vel-x_translation/10;
		set_spd[0]=-(vel);
}

//上台，前轮转的比后轮快
void shangtai(int vel)
{
		set_spd[0]=vel;
		set_spd[3]=-vel;
	
//		set_spd[1]=vel/2;
//		set_spd[2]=-vel/2;

}
//右转，不在原地转而是向前转
void zuozhuan(int vel)
{
		set_spd[1]=-vel;
		set_spd[3]=vel/4;
		set_spd[2]=vel/4;
		set_spd[0]=-vel;	
}

//左转，不在原地转而是向前转
void youzhuan(int vel)
{
		set_spd[1]=-vel/4;
		set_spd[3]=vel;
		set_spd[2]=vel;
		set_spd[0]=-vel/4;	
}

//检测/执行函数
bool hongwai_find(double L)
{
	if(L<80&&L>25)
		return 1;
	else 
		return 0;
}

void find_enermy()
{

	if(GD5==0&&hongwai_find(L1)&&hongwai_find(L2))
	{
		enermy_find=1;
		enermy_front=1;
		turn_left=0;
		turn_right=0;
	}
	else if(hongwai_find(L1))
	{
		turn_right=1;
		turn_left=0;
		enermy_find=1;
		enermy_front=0;
		keep_spin=0;
	}
	else if(hongwai_find(L2))
	{
		turn_left=1;
		turn_right=0;
		enermy_find=1;
		enermy_front=0;
		keep_spin=0;
	}
	else if(hongwai_find(L3))
	{
		turn_left=1;
		turn_right=0;
		enermy_find=1;
		enermy_front=0;
		keep_spin=1;
	}
	else if(hongwai_find(L4))
	{
		turn_left=1;
		turn_right=0;
		enermy_find=1;
		enermy_front=0;
		keep_spin=1;
	}
	else
	{
		turn_right=0;
		turn_left=0;
		enermy_find=0;
		enermy_front=0;
		keep_spin=0;
	}
}


void scan(void)
{
		
		ADValue1=get_adc(&hadc1);
		ADValue2=get_adc(&hadc1);
		ADValue3=get_adc(&hadc1);
		ADValue4=get_adc(&hadc1);

		HAL_ADC_Stop(&hadc1);

	
		Voltage1 = (float)ADValue1 / 4095 * 3.3;
		Voltage2 = (float)ADValue2 / 4095 * 3.3;
		Voltage3 = (float)ADValue3 / 4095 * 3.3;
		Voltage4 = (float)ADValue4 / 4095 * 3.3;
	
	
    L1= max(61.119*pow(Voltage1,-1.092));
    L2= max(61.119*pow(Voltage2,-1.092));
    L3= max(61.119*pow(Voltage3,-1.092));
    L4= max(61.119*pow(Voltage4,-1.092));

		find_enermy();
//		if(HD1>xiataihuidu&&HD1>xiataihuidu)//这里会不会出现出去一半就识别出这个的情况？
//		{
//			osThreadDef(xiatai, XiataiTask, osPriorityNormal, 0, 128);
//			xiataiHandle = osThreadCreate(osThread(xiatai), NULL);
//			vTaskDelete(NULL);
//		}


		

		
}


/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void MoveTask(void const * argument);
void PushBoxTask(void const * argument);
void ScanTask(void const * argument);
void OpenmvConnect(void const * argument);
void Test(void const * argument);
void TickTask(void const * argument);
void KeyTask(void const * argument);
void XiataiTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityHigh, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of move */
  osThreadDef(move, MoveTask, osPriorityAboveNormal, 0, 128);
  moveHandle = osThreadCreate(osThread(move), NULL);

  /* definition and creation of pushBox */
  osThreadDef(pushBox, PushBoxTask, osPriorityNormal, 0, 128);
  pushBoxHandle = osThreadCreate(osThread(pushBox), NULL);

  /* definition and creation of scanTask */
  osThreadDef(scanTask, ScanTask, osPriorityNormal, 0, 128);
  scanTaskHandle = osThreadCreate(osThread(scanTask), NULL);

  /* definition and creation of openmvConnect */
  osThreadDef(openmvConnect, OpenmvConnect, osPriorityNormal, 0, 128);
  openmvConnectHandle = osThreadCreate(osThread(openmvConnect), NULL);

  /* definition and creation of tset */
//  osThreadDef(tset, Test, osPriorityNormal, 0, 128);
//  tsetHandle = osThreadCreate(osThread(tset), NULL);

  /* definition and creation of tick */
  osThreadDef(tick, TickTask, osPriorityRealtime, 0, 128);
  tickHandle = osThreadCreate(osThread(tick), NULL);

  /* definition and creation of key */
  osThreadDef(key, KeyTask, osPriorityRealtime, 0, 128);
  keyHandle = osThreadCreate(osThread(key), NULL);

  /* definition and creation of xiatai */
  osThreadDef(xiatai, XiataiTask, osPriorityNormal, 0, 128);
  xiataiHandle = osThreadCreate(osThread(xiatai), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern uint8_t aRxBuffer;			//接收中断缓冲
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	HAL_UART_Receive_IT(&huart1, (uint8_t *)&aRxBuffer, 1);
	HAL_UART_Receive_IT(&huart3, (uint8_t *)&RxData, 1);

	lcd_init();
	CAN_Filter_Config();
	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING);
//	HAL_TIM_Base_Start_IT(&htim6);

	printf("OK\r\n");
	PID_struct_init(&pid_apriltag_x, POSITION_PID, 500, 500,
								1.5f,	0.1f,	0.0f	);  //待调
	pid_apriltag_x.deadband=1000;//待调
	PID_struct_init(&pid_apriltag_d, POSITION_PID, 500, 500,
								1.5f,	0.1f,	0.0f	);//待调
	pid_apriltag_d.deadband=1000;//待调
	
  lcd_clear(BLACK);
	vTaskDelete(xiataiHandle);
//	forward(vel);
//	spin(vel);




  /* Infinite loop */
  for(;;)
  {
		if(key[1].flag==1)
		{
			vel+=100;
			key[1].flag=0;
		}
//		else if(key[1].flag==2)
//		{
//			vel-=100;
//			key[1].flag=0;
//		}
		sprintf(text,"vel=%d",vel);
		lcd_show_string(10, 140, 60, 32, 16, text, RED);			
			
//		if(x_translation>pid_apriltag_d.deadband)//待调
//		{

//			OLED_ShowString(0,12*0,(uint8_t*)"turn right",12 );
//		}
//		else if(x_translation<-pid_apriltag_d.deadband)//待调
//		{
//			OLED_ShowString(0,12*0,(uint8_t*)"turn left",12 );
//		}
//		else 
//		{
//			OLED_ShowString(0,12*0,(uint8_t*)"go!",12 );
//		}
	
//		OLED_Refresh();
//		if(abs(x_translation)>pid_apriltag_d.deadband)
//		{
//			pid_calc(&pid_apriltag_x, x_translation, 0);	
//			set_spd[0]=	set_spd[1]=	pid_apriltag_x.pos_out;
//			set_spd[2]=	set_spd[3]=	-pid_apriltag_x.pos_out;
//		}		
//		else if(distance<touch_distance)
//		{	
//			set_spd[0]=	set_spd[1]=set_spd[2]=	set_spd[3]=	v_push_box;
//			
//		}
//		HAL_Delay(300);
//		vTaskDelete(NULL);

    osDelay(10);
  }

  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_MoveTask */
/**
* @brief Function implementing the move thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MoveTask */
void MoveTask(void const * argument)
{
  /* USER CODE BEGIN MoveTask */
//	while(key[1].flag!=1){}


//		set_spd[1]=	-3000;
//		set_spd[3]=3000;
//		set_spd[2]=3000;
//		set_spd[0]=-3000;
	
	for(int i=0; i<4; i++)
	{
		PID_struct_init(&pid_spd[i], POSITION_PID, 20000, 20000,
									1.5f,	0.1f,	0.0f	);  //4 motos angular rate closeloop.
		pid_spd[i].deadband=50;
	}
	while(key[0].flag!=1)
	{
	}
	ticktime=0;
	vTaskDelete(defaultTaskHandle);
  /* Infinite loop */
  for(;;)
  {
			for(int i=0; i<4; i++)
			{
				pid_calc(&pid_spd[i], moto_chassis[i].speed_rpm, set_spd[i]);
			}
			set_moto_current(&hcan, pid_spd[0].pos_out, 
									pid_spd[1].pos_out,
									pid_spd[2].pos_out,
									pid_spd[3].pos_out);
		if(can_get_flag==0||can_get_flag==1||can_get_flag==2||can_get_flag==3)
		{
			printf("电机 %d : angle=%d ; speed: %d; current: %d; temp:%d\r\n",
										can_get_flag,moto_chassis[can_get_flag].total_angle,moto_chassis[can_get_flag].speed_rpm,
										moto_chassis[can_get_flag].given_current,moto_chassis[can_get_flag].hall);
				
			can_get_flag=4;
		}
		
		
//打印lcd信息
		if(turn_left)
		{
			sprintf(text,"turn_left");
		}
		else if(turn_right)
		{
			sprintf(text,"turnright");
		}
		else 
		{
			sprintf(text,"no___turn");
		}
		strcat(text,"-----");			
		if(enermy_find&&enermy_front)
		{
			strcat(text,"go!!");
		}
	
		else if(enermy_find)
		{
			strcat(text,"find");
		}
		else 
		{
			strcat(text,"nooo");
		}
		lcd_show_string(10, 140, 150, 32, 16, text, RED);	
		

		
		sprintf(text,"L1=%f",L1);
		lcd_show_string(10, 60, 120, 32, 16, text, RED);
	

		sprintf(text,"L2=%f",L2);
		lcd_show_string(10, 80, 120, 32, 16, text, RED);

		sprintf(text,"L3=%f",L3);
		lcd_show_string(10, 100, 120, 32, 16, text, RED);

		sprintf(text,"L4=%f",L4);
 		lcd_show_string(10, 120, 120, 32, 16, text, RED);	

		
		
		GD1=HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0);
		sprintf(text,"GD1=%u",GD1);
//		if(GD1==1){		set_spd[1]=	0;
//		set_spd[3]=0;
//		set_spd[2]=0;
//		set_spd[0]=0;}
//		else{		set_spd[1]=	1800;
//		set_spd[3]=-1800;
//		set_spd[2]=-1800;
//		set_spd[0]=1800;}
//		lcd_show_string(10, 160, 240, 32, 16, text, RED);
	
		GD2=HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1);
		sprintf(text,"GD2=%u",GD2);
//		lcd_show_string(10, 180, 240, 32, 16, text, RED);

		GD3=HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2);
		sprintf(text,"GD3=%u",GD3);
//		lcd_show_string(10, 200, 240, 32, 16, text, RED);
	
		GD4=HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_3);
		sprintf(text,"GD4=%u",GD4);
//		lcd_show_string(10, 220, 240, 32, 16, text, RED);
		
		GD5=HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_10);
		sprintf(text,"GD5=%u",GD5);
		lcd_show_string(10, 180, 240, 32, 16, text, RED);		
		
		HD1=get_adc(&hadc3);
		HD2=get_adc(&hadc3);

		HAL_ADC_Stop(&hadc3);

		sprintf(text,"HD1=%u",HD1);
		lcd_show_string(10, 200, 240, 32, 16, text, RED);
		
		sprintf(text,"HD2=%u",HD2);
		lcd_show_string(10, 220, 240, 32, 16, text, RED);
		
		
		sprintf(text,"state=%u",state);
		lcd_show_string(10, 240, 240, 32, 16, text, RED);
		
		sprintf(text,"ticktime=%u",ticktime);
		lcd_show_string(10, 20, 240, 32, 16, text, RED);
		sprintf(text,"kuai=%u",kuai);
		lcd_show_string(10, 40, 240, 32, 16, text, RED);
		
	
		osDelay(10);
  }
  /* USER CODE END MoveTask */
}

/* USER CODE BEGIN Header_PushBoxTask */
/**
* @brief Function implementing the pushBox thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PushBoxTask */
void PushBoxTask(void const * argument)
{
  /* USER CODE BEGIN PushBoxTask */
//	forward(-3500);
//	osDelay(2000);
	
//	spin(-1200);
//	osDelay(1500);
  /* Infinite loop */
  for(;;)
  {
		if(HD1<xiataihuidu1&&HD2<xiataihuidu2)
			{
				osThreadDef(xiatai, XiataiTask, osPriorityNormal, 0, 128);
				xiataiHandle = osThreadCreate(osThread(xiatai), NULL);
				vTaskDelete(pushBoxHandle);
				vTaskDelete(NULL);
				openmvxuntai=0;
			}

			if(GD1||GD2)
			{
				forward(-vel);
				osDelay(2000);
				forward(0);
				if(zhengduikuai==1)
				{
					zhengduikuai=0;
					find_then_zhengduikuai=1;
					x_translation=0;
					kuai--;
					openmvxuntai=1;
				}
			}
			else if(GD3||GD4)
			{
				forward(vel);
				osDelay(2000);
			}
		if(zhengduikuai)
		{
			forward_spin(vel);
		}
			
		//pushbox代码本来的地方

		if(openmvxuntai&&x_translation==0)
		{
			if(GD1||GD2)
			{
				forward(-vel);
				osDelay(2000);
				forward(0);
				if(zhengduikuai==1)
				{
					zhengduikuai=0;
					find_then_zhengduikuai=1;
					x_translation=0;
					kuai--;
				}
			}
			if(GD3||GD4)
			{
				forward(vel);
				osDelay(2000);			
			}
//			else 
//			{
//				spin(600);
//			}
			
			else if(HD1<xiataihuidu1&&HD2<xiataihuidu2)
			{
				osThreadDef(xiatai, XiataiTask, osPriorityNormal, 0, 128);
				xiataiHandle = osThreadCreate(osThread(xiatai), NULL);
				vTaskDelete(pushBoxHandle);
				vTaskDelete(NULL);
				openmvxuntai=0;
			}
			else if(HD1<zhongxinhuidu1)
			{
				spin(-1200);
				osDelay(1200);
				forward(vel);
				osDelay(500);

			}
			else
			{
				forward(vel);
			}

		}	

//			printf("id:%d\r\n",tag_id);
//			printf("x_translation:%d\r\n",x_translation);		
//			printf("x_rotation:%d\r\n",x_rotation);
//			printf("%d\r\n",x_rotation-1800000);
//			printf("find=%d",find);
	
    osDelay(5);
  }
  /* USER CODE END PushBoxTask */
}

/* USER CODE BEGIN Header_ScanTask */
/**
* @brief Function implementing the scanTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ScanTask */
void ScanTask(void const * argument)
{
  /* USER CODE BEGIN ScanTask */

  /* Infinite loop */
  for(;;)
  {
		scan();//需要赋值的：红外：FL,FR,l,r；光电：outofstage(是否和哪边出台)

//		if(left_infrafot_distance!=0)
//		{
//			turn_left=1;
////			OLED_ShowString(0,12*4,(uint8_t*)"turn left",12);					
//		}
//		else if(right_infrafot_distance!=0)
//		{
//			turn_right=1;
////			OLED_ShowString(0,12*4,(uint8_t*)"turn right",12);					
//		}
//		else if(front_infrafot_distanceL!=0&&front_infrafot_distanceR==0)
//		{
//			turn_left=1;
////			OLED_ShowString(0,12*4,(uint8_t*)"turn left",12);	
//		}
//		else if(front_infrafot_distanceL==0&&front_infrafot_distanceR!=0)
//		{
//			turn_right=1;
////			OLED_ShowString(0,12*4,(uint8_t*)"turn right",12);	
//		}
//		else if(fabs(front_infrafot_distanceL-front_infrafot_distanceR)>front_dis_err)
//		{
//			if(front_infrafot_distanceL>front_infrafot_distanceR)
//			{
//				turn_right=1;
////				OLED_ShowString(0,12*4,(uint8_t*)"turn right",12);			
//			}
//			else if(front_infrafot_distanceL<front_infrafot_distanceR)
//			{
//				turn_left=1;
////				OLED_ShowString(0,12*4,(uint8_t*)"turn left",12);			
//			}
//			
//		}
//		else if(fabs(front_infrafot_distanceL-front_infrafot_distanceR)<front_dis_err)
//		{
//			go_straight=1;
////			OLED_ShowString(0,12*4,(uint8_t*)"go",12);			
//		}
//		else //在车尾
//		{
//			
//		}
//		if(outofstage!=0)
//		{
//			switch(outofstage)
//			{
//				case 1://车头出台
////					OLED_ShowString(0,12*2,(uint8_t*)"headout",12);			
//					break;
//				case 2://车尾出台
////					OLED_ShowString(0,12*2,(uint8_t*)"backout",12);						
//					break;
//				case 3://左侧出台
////					OLED_ShowString(0,12*2,(uint8_t*)"leftout",12);						
//					break;
//				case 4://右侧出台
////					OLED_ShowString(0,12*2,(uint8_t*)"rightout",12);						
//					break;
//			}
//			outofstage=0;
//		}
//		sprintf(text,"FL:%.2f  FR:%.2f",front_infrafot_distanceL,front_infrafot_distanceR);
////		OLED_ShowString(0,12*0,(uint8_t*)text,12);	
//		sprintf(text,"l:%.2f  r:%.2f",left_infrafot_distance,right_infrafot_distance);
////		OLED_ShowString(0,12*1,(uint8_t*)text,12);	

//			sprintf(text,"电机 %d :speed: %d;",can_get_flag,moto_chassis[can_get_flag].speed_rpm);
//			lcd_show_string(10, 40, 200, 32, 16, text, RED);	

		

    osDelay(10);
  }
  /* USER CODE END ScanTask */
}

/* USER CODE BEGIN Header_OpenmvConnect */
/**
* @brief Function implementing the openmvConnect thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OpenmvConnect */
void OpenmvConnect(void const * argument)
{
  /* USER CODE BEGIN OpenmvConnect */

  /* Infinite loop */
  for(;;)
  {
	if(find_then_zhengduikuai)
	{
//		if((x_rotation-1800000)>150000)
//		{			
////			youzhuan(800);
////			osDelay(800);
////			spin(-600);
////			osDelay(500);	
//			spin(600);
//			osDelay(500);
//			forward(1000);
//			osDelay(600);
//			spin(-600);
//			osDelay(600);				
//		}
//		else if((x_rotation-1800000)<-150000)
//		{	
////			zuozhuan(800);
////			osDelay(800);
////			spin(400);
////			osDelay(500);
//			spin(-600);
//			osDelay(500);
//			forward(1000);
//			osDelay(600);
//			spin(600);
//			osDelay(600);			
//		}
//		else
//			x_rotation_flag=1;
		if(x_rotation_flag)
		{
			if(x_translation!=0)
			{
				openmvxuntai=0;
				if(x_translation>1000)
				{
					spin(-300);
				}
				else if(x_translation<-1000)
				{
					spin(300);
				}
				else 
				{
		//			diaotai=1;
		//			zhengduikuaitest=0;
//					if(zhengduiflag==1)
//					{
//					forward(0);
//					osDelay(200);
//					}
					forward_spin(vel);
					find_then_zhengduikuai=0;
					zhengduikuai=1;
					openmvxuntai=0;
				}
			}
		}
	}

			


		if(kuai==0||ticktime>=tuikuaitime)
		{
			osThreadDef(tset, Test, osPriorityNormal, 0, 128);
			tsetHandle = osThreadCreate(osThread(tset), NULL);
			vTaskDelete(pushBoxHandle);
			vTaskDelete(NULL);
		}
    osDelay(5);
  }
  /* USER CODE END OpenmvConnect */
}

/* USER CODE BEGIN Header_Test */
/**
* @brief Function implementing the tset thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Test */
void Test(void const * argument)
{
  /* USER CODE BEGIN Test */
//		forward(vel);
//	spin(800);
//	shangtai(4500);

  /* Infinite loop */
	//state: 0前进 1后退 2旋转
  for(;;)
  {
		/*检测台边回中检测逻辑：
		-->起始0--检测到台边-->1--回到台中(ticktime>time)-->2
		|																									  |
		-------------------<--转一定角度(ticktime>spintime)--	
		
		*/
//		if(xiataiflag)
//		{
//			osThreadDef(xiatai, XiataiTask, osPriorityNormal, 0, 128);
//			xiataiHandle = osThreadCreate(osThread(xiatai), NULL);
//			vTaskDelete(NULL);
//		}
		
		
//		forward(0);
//		if(HD1<xiataihuidu1&&HD2<xiataihuidu2)
//		{
//			osThreadDef(xiatai, XiataiTask, osPriorityNormal, 0, 128);
//			xiataiHandle = osThreadCreate(osThread(xiatai), NULL);
//			vTaskDelete(NULL);
//		}
//		else if(huizhong)
//		{
//			if(abs(HD1-HD2)>zhengduihuiducha)
//			{
//				if(HD1>HD2)
//					forward(-vel);
//				else
//					forward(vel);
//			}
//		if(HD1<zhongxinhuidu1)
//		{
//			if(HD2-HD1>zhengduihuiducha)
//			{

//				forward(-vel);
//				osDelay(1000);
//				forward(0);
//			}
//			else spin(1000);
//		}

//			else if(HD1>zhongxinhuidu1&&HD2>zhongxinhuidu2)
//			{
//				huizhong=0;
//			}
//			else 
//				spin(500);
//		}
//		else if(xuntai)
//		{
//			if(GD1||GD2)
//			{
//				forward(-vel);
//				osDelay(1500);
//			}
//			else if(GD3||GD4)
//			{
//				forward(vel);
//				osDelay(1500);			
//			}
//			if(HD1<xiataihuidu1&&HD2<xiataihuidu2)
//			{
//				osThreadDef(xiatai, XiataiTask, osPriorityNormal, 0, 128);
//				xiataiHandle = osThreadCreate(osThread(xiatai), NULL);
//				vTaskDelete(NULL);
//			}
//			else if(HD1<zhongxinhuidu1)
//			{
//				huizhong=1;
//			}
//			else 
//			{
//				spin(600);
//			}

//		}
//		else if(xuntaitest)
//		{
//			if(state==0)
//			{
//				if(GD1||GD2||GD3||GD4)//记得把GD2加回来
//				{
//					state=1;
//					state_change=1;	
//				}
//			}
//			
//			if(state_change==1)
//			{		
//				switch(state)
//				{
//					case 0:
//						ticktime=0;
//						forward(vel);
//						state_change=0;
//						break;
//					case 1:
//						time=ticktime;
//						ticktime=0;
//						forward(-vel);
//						osDelay(time);
//						state=2;
//						state_change=1;
//						break;
//					case 2:
//						ticktime=0;
//						spin(600);
//						osDelay(spintime);
//						state=0;
//						state_change=1;

//						break;				
//				}		

//				lcd_clear(BLACK);
//			}
//		
//			
//		}
//		
//		if(zhengduitest)
//		{

//				if(enermy_front)
//				{
//						forward(0);
//				}					
//				else if(turn_left)
//				{
//					spin(-800);
//				}
//				else if(turn_right)
//				{
//					spin(800);
//				}
//				else 
//				{
//					if(keep_spin==0)
//					{
//						forward(0);
//					}
//				}					

//		}
//		

//		
//		


//		else if(tuichetest)//可由zhengduitest改过来
//		{
//				if(GD1==0||GD2==0||GD3==0||GD4==0||GD5==0)
//				{
//					forward(0);
//				}

//				else if(enermy_front)
//				{
//						forward(1000);
//				}					
//				else if(turn_left)
//				{
//					spin(-800);
//				}
//				else if(turn_right)
//				{
//					spin(800);
//				}
//				else 
//				{
//					if(keep_spin==0)
//					{
//						forward(0);
//					}
//				}					

//		}
//		else if(diaotai)		
//		{
//				if(GD1||GD2||GD3||GD4)
//				{
//					forward(0);
//				}
//		}

		

//		if(x_translation>200)
//		{
//			forward(200);
//		}
//		else if(x_translation<200)
//		{
//			forward(-200);
//		}
//		else
//			forward(0);


    osDelay(10);
  }
  /* USER CODE END Test */
}

/* USER CODE BEGIN Header_TickTask */
/**
* @brief Function implementing the tick thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TickTask */
void TickTask(void const * argument)
{
  /* USER CODE BEGIN TickTask */
  /* Infinite loop */
  for(;;)
  {
		ticktime+=10;
//		if(xuntaitest)
//		{
//			if(state==0)
//			{
//				
//			}
//			else if(state==1)
//			{
//				if(ticktime>time)
//				{
//					state=2;
//					state_change=1;
//				}
//			}
//			else
//			{
//				if(ticktime>spintime)
//				{
//					state=0;
//					state_change=1;
//				}
//			}
//		}
	if(ticktime>=chaoshitime)
	{
		vTaskDelete(scanTaskHandle);
		vTaskDelete(moveHandle);
		lcd_clear(BLACK);
		sprintf(text,"-----------timeout-----------");
		lcd_show_string(10, 160, 240, 32, 16, text, RED);
		
	}


		
    osDelay(10);
  }
  /* USER CODE END TickTask */
}

/* USER CODE BEGIN Header_KeyTask */
/**
* @brief Function implementing the key thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_KeyTask */
void KeyTask(void const * argument)
{
  /* USER CODE BEGIN KeyTask */
  /* Infinite loop */
  for(;;)
  {
		key[0].key_sta=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4);
		key[1].key_sta=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3);
		key[2].key_sta=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2);
		key[3].key_sta=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
		for(int i=0;i<4;i++)
		{
			switch(key[i].jud_sta)
			{
				case 0:
					if(key[i].key_sta==GPIO_PIN_RESET)
					{
						key[i].jud_sta=1;
					}
				break;
				case 1:
					if(key[i].key_sta==GPIO_PIN_RESET)
					{
						key[i].jud_sta=2;
					}
					else
					{
					key[i].jud_sta=0;
					}
				break;
				case 2:
					if(key[i].key_sta==GPIO_PIN_SET)
					{
						if(i==3&&long_time>200){

							key[i].flag=2;
							key[i].jud_sta=0;
							long_time=0;
						}
						else{
						key[i].flag=1;
						key[i].jud_sta=0;
						}
					}
					else if(key[i].key_sta==GPIO_PIN_RESET&&i==3)
					{
						long_time+=10;
					}
					
				break;

			}
		}
		
    osDelay(10);
  }
  /* USER CODE END KeyTask */
}

/* USER CODE BEGIN Header_XiataiTask */
/**
* @brief Function implementing the xiatai thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_XiataiTask */
void XiataiTask(void const * argument)
{
  /* USER CODE BEGIN XiataiTask */
  /* Infinite loop */
  for(;;)
  {
		forward(0);
//		ticktime=0;
//		forward(-2000);
//		while(ticktime>shangtaitime1)
//		{}
//		forward(2000);
//		while(ticktime>shangtaitime2)//定死延时
//		{}
////	  while(HD1<&&HD2<)//灰度判断{}		

//		osThreadDef(tset, Test, osPriorityNormal, 0, 128);
//		tsetHandle = osThreadCreate(osThread(tset), NULL);
//		vTaskDelete(NULL);
    osDelay(1);
  }
  /* USER CODE END XiataiTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

