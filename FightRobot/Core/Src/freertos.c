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
#include "math.h"

#include "my_uart.h"
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
int old_x_translation=0;
uint8_t xiatai_dir=0;//下台面对的方向 不用转：1 要转：2 左边掉：3 右边掉：4
uint8_t xiatai_state=0;//实际下台状态
bool xiatai_change=0;
bool xiatai=0;
bool shangtaiflag=1;
bool tuikuaiflag=1;
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
uint32_t ticktime_last=0;
uint32_t spintime=1000;
uint32_t chaoshitime=200000;
uint32_t tuikuaitime=100000;
uint16_t long_time=0;
uint16_t shangtaitime1=500;//下台后向前撞以对正的时间
uint16_t shangtaitime2=2000;//下台对正后冲上台的时间
uint16_t	zhengduihuiducha=700;//正对灰度差，待测
//uint16_t zhongxinhuidu1=2600;//灰度1的中心灰度，待测
uint16_t zhongxinhuidu1=2500;//灰度1的中心灰度，待测
uint16_t zhongxinhuidu2=3500;//灰度2的中心灰度，待测
uint16_t xiataihuidu1=2200;//灰度1的下台灰度，待测
uint16_t xiataihuidu2=2200;//灰度2的下台灰度，待测

uint16_t turn90=600;
uint16_t turn180=1200;
uint16_t turn360=2200;

bool turn90flag=0;
bool turn180flag=0;
bool turn360flag=0;

double JY901_ANGLE1=0;
double JY901_ANGLE2=0;
double JY901_ANGLE3=0;
double JY901_ANGLE4=0;

//实际判断量
bool enermy_find=0;
bool enermy_front=0;
bool enermy_go=0;//这个是只要之前曾经检测到就先撞
bool turn_left=0;
bool turn_right=0;
bool xuntaitest=0;
bool xuntai=0;
bool zhengduitest=1;
bool tuichetest=0;
bool keep_spin=0;
bool huizhong=0;
bool zhengduikuai=0;
bool findkuai=0;
bool find_then_zhengduikuai=1;
bool openmvxuntai=1;
bool x_rotation_flag=1;
bool x_not_change=0;
bool zhengduiche=0;
bool tui=0;
uint8_t kuai=2;

//debug使用
	int vel_che=5000;
	int vel_kuai=1500;
	uint8_t state=0;//0:前进   1:后退   2:前进
	bool state_change=0;
	uint32_t time=0;
	
	
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数    
struct __FILE 
{ 
	int handle; 
}; 
 
FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 

	
float pitch,roll,yaw; 		//欧拉角:俯仰角，滚转角，偏航角
short aacx,aacy,aacz;		//加速度传感器原始数据  加速度
short gyrox,gyroy,gyroz;	//陀螺仪原始数据  角速度
short temp;					//MPU温度
uint8_t str_buff[64];
uint8_t str_buff1[64]="俯仰角:";		//pitch
uint8_t str_buff2[64]="偏航角:";		//yaw
uint8_t str_buff3[64]="翻滚角:";		//roll
uint8_t str_buff4[64]="温度值:";
 
struct MPU6050				//MPU6050结构体
{
	u8 flag;				//采集成功标志位
	u8 speed;				//上报速度
}mpu6050;					//唯一结构体变量

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

/**
  * @brief  MPU6050数据上报
  * @param  无
  * @retval 无
  */



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
	if(x_not_change==0){
		set_spd[1]=-(vel);
		set_spd[3]=vel-x_translation/10;
		set_spd[2]=vel-x_translation/10;
		set_spd[0]=-(vel);}
	else 
	{
		set_spd[1]=-vel;
		set_spd[3]=vel;
		set_spd[2]=vel;
		set_spd[0]=-vel;
	}
}

void forward_pian(int pianzhi)
{
		set_spd[1]=-(vel_kuai);
		set_spd[3]=vel_kuai+pianzhi;
		set_spd[2]=vel_kuai+pianzhi;
		set_spd[0]=-(vel_kuai);	
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

//	if((GD5==0&&hongwai_find(L1))||(GD5==0&&hongwai_find(L2)))
	if(GD5==0&&hongwai_find(L1)&&hongwai_find(L2))
	{
		enermy_find=1;
		enermy_front=1;
		turn_left=0;
		turn_right=0;
		zhengduiche=1;
		enermy_go=1;
		
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

			if(GD1&&GD2)
			{
				xiatai_dir=1;
				xiatai_change=1;
			}
			else if(GD3&&GD4)
			{
				xiatai_dir=2;
				xiatai_change=1;			
			}
			else if(GD1&&GD3)
			{
				xiatai_dir=3;
				xiatai_change=1;			
			}
			else if(GD2&&GD4)
			{
				xiatai_dir=4;
				xiatai_change=1;			
			}
			
		find_enermy();



		

		
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
  osThreadDef(openmvConnect, OpenmvConnect, osPriorityAboveNormal, 0, 128);
  openmvConnectHandle = osThreadCreate(osThread(openmvConnect), NULL);

  /* definition and creation of tset */
  osThreadDef(tset, Test, osPriorityNormal, 0, 128);
  tsetHandle = osThreadCreate(osThread(tset), NULL);

  /* definition and creation of tick */
  osThreadDef(tick, TickTask, osPriorityHigh, 0, 128);
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
	User_USART_Init(&JY901_data);
//	while(mpu_dmp_init())                            //初始化mpu_dmp库
// 	{
//		printf("Initialization failed！\r\n");		//串口初始化失败上报
//	}
	mpu6050.flag = 0;          //采集成功标志位初始化
	mpu6050.speed = 0;				//上报速度初始化
//	HAL_TIM_Base_Start_IT(&htim6);

	printf("OK\r\n");
	PID_struct_init(&pid_apriltag_x, POSITION_PID, 500, 500,
								1.5f,	0.1f,	0.0f	);  //待调
	pid_apriltag_x.deadband=1000;//待调
	PID_struct_init(&pid_apriltag_d, POSITION_PID, 500, 500,
								1.5f,	0.1f,	0.0f	);//待调
	pid_apriltag_d.deadband=1000;//待调
	
  lcd_clear(BLACK);
	
  osThreadDef(tick, TickTask, osPriorityHigh, 0, 128);
  tickHandle = osThreadCreate(osThread(tick), NULL);
  osThreadDef(key, KeyTask, osPriorityRealtime, 0, 128);
  keyHandle = osThreadCreate(osThread(key), NULL);
	
	vTaskDelete(xiataiHandle);
//	forward(vel);
//	spin(vel);




  /* Infinite loop */
  for(;;)
  {
		if(key[1].flag==1)
		{
			vel_che+=100;
			key[1].flag=0;
		}

		sprintf(text,"vel_che=%d",vel_che);
		lcd_show_string(10, 140, 60, 32, 16, text, RED);			
			
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
		
		
		sprintf(text,"L1=%f",L1);
		lcd_show_string(10, 60, 120, 32, 16, text, RED);
	

		sprintf(text,"L2=%f",L2);
		lcd_show_string(10, 80, 120, 32, 16, text, RED);

		sprintf(text,"L3=%f",L3);
		lcd_show_string(10, 100, 120, 32, 16, text, RED);

		sprintf(text,"L4=%f",L4);
 		lcd_show_string(10, 120, 120, 32, 16, text, RED);	
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
//			printf("电机 %d : angle=%d ; speed: %d; current: %d; temp:%d\r\n",
//										can_get_flag,moto_chassis[can_get_flag].total_angle,moto_chassis[can_get_flag].speed_rpm,
//										moto_chassis[can_get_flag].given_current,moto_chassis[can_get_flag].hall);
				
			can_get_flag=4;
		}
//			printf("id:%d\r\n",tag_id);
//			printf("x_translation:%d\r\n",x_translation);		
//			printf("x_rotation:%d\r\n",x_rotation);
//			printf("%d\r\n",x_rotation-1800000);
			if(x_not_change==0)
				printf("find");
		
		
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
		if(xiatai)
		{
			sprintf(text,"XIATAI");
			lcd_show_string(10, 60, 120, 32, 16, text, RED);
		}

		
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
		
		
		sprintf(text,"xiataistate=%u",xiatai_state);
		lcd_show_string(10, 240, 240, 32, 16, text, RED);
		
//		sprintf(text,"ticktime=%u",ticktime);
//		lcd_show_string(10, 20, 240, 32, 16, text, RED);
		
		sprintf(text,"e:%u",enermy_front);
		lcd_show_string(10, 20, 240, 32, 16, text, RED);		
//		sprintf(text,"x_not_change=%u",x_not_change);
//		lcd_show_string(10, 40, 240, 32, 16, text, RED);
		

//		if(key[1].flag!=1)
//		{
//			forward(0);
//			vTaskDelete(NULL);
//		}




//		if(key[0].flag==1){forward(0);vTaskDelete(NULL);}
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

	
//	spin(-1200);
//	osDelay(1500);
//	if(shangtaiflag)
//	{
////		forward(-4000);
////		osDelay(2000);
////		shangtaiflag=0;
//		spin(2000);
//		osDelay(turn90);
//		forward(0);
//		osDelay(20000);
//	}
  /* Infinite loop */
  for(;;)
  {
		if(HD1<xiataihuidu1&&HD2<xiataihuidu2)
			{
				xiatai=1;
				osThreadDef(xiatai, XiataiTask, osPriorityNormal, 0, 128);
				xiataiHandle = osThreadCreate(osThread(xiatai), NULL);
				vTaskDelete(openmvConnectHandle);
				openmvConnectHandle=NULL;
				vTaskDelete(NULL);
				openmvxuntai=0;
			}


			if(GD1||GD2)
			{
				xiatai_dir=1;
				tui=1;
				forward(-2000);
				osDelay(1000);
				forward(0);
				if(zhengduikuai==1)
				{
					zhengduikuai=0;
					findkuai=0;
					find_then_zhengduikuai=1;
					x_translation=0;
					kuai--;
					openmvxuntai=1;
				}
				else if(findkuai)
				{
					findkuai=0;
				}
			}
			else if(GD3||GD4)
			{
				tui=1;
				findkuai=0;
				forward(2000);
				osDelay(1000);
			}
			else if(zhengduikuai)
			{
				forward_spin(vel_kuai);
			}
			else if(turn360flag)
			{
				turn360flag=0;
				spin(-2000);
				osDelay(turn360);
			}
		//openmvconnect代码本来的地方

		if(openmvxuntai&&x_translation==0)
		{
			if(GD1||GD2)
			{
				tui=1;
				forward(-2000);
				osDelay(1000);
				forward(0);
				if(zhengduikuai==1)
				{
					zhengduikuai=0;
					findkuai=0;
					find_then_zhengduikuai=1;
					x_translation=0;
					kuai--;
				}
				else if(findkuai)
				{
					findkuai=0;
				}
			}
			if(GD3||GD4)
			{
				tui=1;
				findkuai=0;
				forward(2000);
				osDelay(1000);			
			}
//			else 
//			{
//				spin(600);
//			}
			
			else if(HD1<xiataihuidu1&&HD2<xiataihuidu2)
			{
				xiatai=1;
				osThreadDef(xiatai, XiataiTask, osPriorityNormal, 0, 128);
				xiataiHandle = osThreadCreate(osThread(xiatai), NULL);
				vTaskDelete(openmvConnectHandle);
				openmvConnectHandle=NULL;
				vTaskDelete(NULL);
				openmvxuntai=0;
			}
			else if(HD1<zhongxinhuidu1)
			{
				spin(-1200);
				osDelay(300);
				forward(vel_kuai);
				osDelay(200);

			}
			else
			{
				forward(3000);
			}

		}	


	
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
		static bool deleteflag=0;//把pushbox删除标志位
//		if(tui&&(GD1||GD2||GD3||GD4))
//		{
//			spin(600);
//		}
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
				//直接干掉推块进程
				if(findkuai!=1)
				{
					vTaskDelete(pushBoxHandle);
					pushBoxHandle=NULL;
					deleteflag=1;
				}
				openmvxuntai=0;
				if(x_translation>1000)
				{
					spin(-300);
					findkuai=1;
				}
				else if(x_translation<-1000)
				{
					spin(300);
					findkuai=1;
				}
				else 
				{
		//			zhengduikuaitest=0;
//					if(zhengduiflag==1)
//					{
//					forward(0);
//					osDelay(200);
//					}
//					forward_spin(vel_kuai);
					forward(0);
					find_then_zhengduikuai=0;
					findkuai=1;
					zhengduikuai=1;
					openmvxuntai=0;
					if(deleteflag)
					{
						osThreadDef(pushBox, PushBoxTask, osPriorityNormal, 0, 128);
						pushBoxHandle = osThreadCreate(osThread(pushBox), NULL);
						deleteflag=0;
					}
				}

			}
		}
	}

			


		if(kuai==0||ticktime>=tuikuaitime)
		{
			osThreadDef(tset, Test, osPriorityNormal, 0, 128);
			tsetHandle = osThreadCreate(osThread(tset), NULL);
			tuikuaiflag=0;
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

		
//		spin(500);

//		forward(0);
		
		if(HD1<xiataihuidu1&&HD2<xiataihuidu2)
		{
			xiatai=1;
			osThreadDef(xiatai, XiataiTask, osPriorityNormal, 0, 128);
			xiataiHandle = osThreadCreate(osThread(xiatai), NULL);
			vTaskDelete(NULL);
		}
//		else if(huizhong)
//		{
//			if(abs(HD1-HD2)>zhengduihuiducha)
//			{
//				if(HD1>HD2)
//					forward(-vel);
//				else
//					forward(vel);
//			}
		if(GD1||GD2)
		{
			forward(-2000);
			osDelay(1000);
			enermy_front=0;
			enermy_go=0;
			zhengduiche=0;
		}
		else if(GD3||GD4)
		{
			forward(2000);
			osDelay(1000);	
			enermy_front=0;	
			enermy_go=0;			
			zhengduiche=0;
		}
		else if(HD1<zhongxinhuidu1&&enermy_front==0)
		{
			if(HD2-HD1>zhengduihuiducha)
			{

				forward(-3000);
				while(HD1>zhongxinhuidu1+100);
//				osDelay(1000);
			}
//			else spin(1000);
		}
				if(enermy_go)
				{
						keep_spin=0;
						forward(vel_che);
				}					
				else if(turn_left)
				{
					if(zhengduiche)
					{
						forward_pian(50);
					}
					else
						spin(-800);
				}
				else if(turn_right)
				{
					if(zhengduiche)
					{
						forward_pian(50);
					}
					else					
						spin(800);
				}
		else if(keep_spin==0)
		{


						spin(800);
					
									

		}





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
	if(tuikuaiflag&&((ticktime%20000)==0))
	{
		turn360flag=1;
	}
	
	if(xiatai_change&&(xiatai==0))
	{
		
		if(xiatai_state==0)
		{
			xiatai_state=xiatai_dir;
		}
		else 
		{
			if(ticktime-ticktime_last>1000)
			{
				xiatai_state=xiatai_dir;
			}
		}
		ticktime_last=ticktime;
		xiatai_change=0;
	}
	
	if(ticktime%3==0)
	{
		if(old_x_translation==x_translation)
			x_not_change=1;
		else 
			x_not_change=0;
		old_x_translation=x_translation;
	}

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
//		if((L3!=0)&&(L4!=0)&&(L4<40)&&(L3<40))
//		{
//			key[0].flag=1;
//		}
		
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
		osDelay(2000);
		if(xiatai_state==1)
		{
			forward(3000);
			osDelay(1000);
			forward(-4000);
			osDelay(3500);
		}
		else if(xiatai_state==2)
		{
			spin(2000);
			osDelay(1200);
			forward(3000);
			osDelay(1000);
			forward(-4000);	
			osDelay(3500);			
		}
		else if(xiatai_state==3)
		{
			zuozhuan(2000);
			osDelay(1200);
			forward(3000);
			osDelay(1500);
			forward(-4000);	
			osDelay(3500);
		}
		else if(xiatai_state==4)
		{
			youzhuan(2000);
			osDelay(1200);
			forward(3000);
			osDelay(1500);
			forward(-4000);	
			osDelay(3500);
		}
		
//		ticktime=0;
//		forward(-2000);
//		while(ticktime>shangtaitime1)
//		{}
//		forward(2000);
//		while(ticktime>shangtaitime2)//定死延时
//		{}
	  while(HD1>xiataihuidu1&&HD2>xiataihuidu1)//灰度判断{}
		{		

		osThreadDef(tset, Test, osPriorityNormal, 0, 128);
		tsetHandle = osThreadCreate(osThread(tset), NULL);
		vTaskDelete(NULL);
		}
    osDelay(1);
  }
  /* USER CODE END XiataiTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

