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

extern int tag_id;
extern int x_translation;
extern int distance;



s16 torque=500;
uint8_t data[8]={1,2,3,4,5,6,7,8};
uint8_t rxcnt=0;

uint8_t can_get_flag=4;//要打印需要该值在0-3之间
int set_v,set_spd[4];

char text[20];
bool turn_left=0;//这里记得要把行进策略写在一个变量里
bool turn_right=0;
bool go_straight=0;
double front_infrafot_distanceL=0; //左边的
double front_infrafot_distanceR=0; //右边的
double left_infrafot_distance=0; 
double right_infrafot_distance=0;
double front_dis_err=0;
uint8_t outofstage=0;
uint16_t v_push_box=200;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId moveHandle;
osThreadId pushBoxHandle;
osThreadId scanTaskHandle;
osThreadId openmvConnectHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void scan(void)
{
	
}

void spin(void)
{

}
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void MoveTask(void const * argument);
void PushBoxTask(void const * argument);
void ScanTask(void const * argument);
void OpenmvConnect(void const * argument);

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
  osThreadDef(move, MoveTask, osPriorityNormal, 0, 128);
  moveHandle = osThreadCreate(osThread(move), NULL);

  /* definition and creation of pushBox */
  osThreadDef(pushBox, PushBoxTask, osPriorityAboveNormal, 0, 128);
  pushBoxHandle = osThreadCreate(osThread(pushBox), NULL);

  /* definition and creation of scanTask */
  osThreadDef(scanTask, ScanTask, osPriorityAboveNormal, 0, 128);
  scanTaskHandle = osThreadCreate(osThread(scanTask), NULL);

  /* definition and creation of openmvConnect */
  osThreadDef(openmvConnect, OpenmvConnect, osPriorityAboveNormal, 0, 128);
  openmvConnectHandle = osThreadCreate(osThread(openmvConnect), NULL);

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
	OLED_Init();
	OLED_ColorTurn(0);//0正常显示，1 反色显示
  OLED_DisplayTurn(0);//0正常显示 1 屏幕翻转显示
	OLED_Refresh();
	

	OLED_ShowString(0,12*0,(uint8_t*)"11111111111111111111111",12 );	
	OLED_ShowString(0,12*1,(uint8_t*)"test",12);
	OLED_ShowString(0,12*2,(uint8_t*)"hello world",12 );	
	OLED_ShowString(0,12*3,(uint8_t*)"test",12);
	OLED_ShowString(0,12*4,(uint8_t*)"test",12);
	OLED_Refresh();	
	OLED_ShowString(0,12*4,(uint8_t*)"nihao",12);
	OLED_Refresh();	
	CAN_Filter_Config();
	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING);
	printf("OK\r\n");
	PID_struct_init(&pid_apriltag_x, POSITION_PID, 500, 500,
								1.5f,	0.1f,	0.0f	);  //待调
	pid_apriltag_x.deadband=1000;//待调
	PID_struct_init(&pid_apriltag_d, POSITION_PID, 500, 500,
								1.5f,	0.1f,	0.0f	);//待调
	pid_apriltag_d.deadband=1000;//待调
//	vTaskDelete(NULL);
  /* Infinite loop */
  for(;;)
  {
		printf("id:%d\r\n",tag_id);
		printf("x_translation:%d\r\n",x_translation);		
		printf("distance:%d\r\n",distance);
		OLED_Clear();
		if(x_translation>pid_apriltag_d.deadband)//待调
		{

			OLED_ShowString(0,12*0,(uint8_t*)"turn right",12 );
		}
		else if(x_translation<-pid_apriltag_d.deadband)//待调
		{
			OLED_ShowString(0,12*0,(uint8_t*)"turn left",12 );
		}
		else 
		{
			OLED_ShowString(0,12*0,(uint8_t*)"go!",12 );
		}

		OLED_Refresh();
		if(abs(x_translation)>pid_apriltag_d.deadband)
		{
			pid_calc(&pid_apriltag_x, x_translation, 0);	
			set_spd[0]=	set_spd[1]=	pid_apriltag_x.pos_out;
			set_spd[2]=	set_spd[3]=	-pid_apriltag_x.pos_out;
		}		
		else if(distance<touch_distance)
		{	
			set_spd[0]=	set_spd[1]=set_spd[2]=	set_spd[3]=	v_push_box;
			
		}
//		HAL_Delay(300);
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
	set_spd[0]=	set_spd[1]=	set_spd[2]=	set_spd[3]=400;
	for(int i=0; i<4; i++)
	{
		PID_struct_init(&pid_spd[i], POSITION_PID, 20000, 20000,
									1.5f,	0.1f,	0.0f	);  //4 motos angular rate closeloop.
		pid_spd[i].deadband=50;
	}
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
	
		osDelay(1);
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
  /* Infinite loop */
  for(;;)
  {
		if(Serial_GetRxFlag())
		{
			sprintf(text,"ID:%d  x:%d, dis:%d",tag_id,x_translation,distance);
			OLED_ShowString(0,12*0,(uint8_t*)text,12);	
		}
    osDelay(1);
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

		if(left_infrafot_distance!=0)
		{
			turn_left=1;
			OLED_ShowString(0,12*4,(uint8_t*)"turn left",12);					
		}
		else if(right_infrafot_distance!=0)
		{
			turn_right=1;
			OLED_ShowString(0,12*4,(uint8_t*)"turn right",12);					
		}
		else if(front_infrafot_distanceL!=0&&front_infrafot_distanceR==0)
		{
			turn_left=1;
			OLED_ShowString(0,12*4,(uint8_t*)"turn left",12);	
		}
		else if(front_infrafot_distanceL==0&&front_infrafot_distanceR!=0)
		{
			turn_right=1;
			OLED_ShowString(0,12*4,(uint8_t*)"turn right",12);	
		}
		else if(fabs(front_infrafot_distanceL-front_infrafot_distanceR)>front_dis_err)
		{
			if(front_infrafot_distanceL>front_infrafot_distanceR)
			{
				turn_right=1;
				OLED_ShowString(0,12*4,(uint8_t*)"turn right",12);			
			}
			else if(front_infrafot_distanceL<front_infrafot_distanceR)
			{
				turn_left=1;
				OLED_ShowString(0,12*4,(uint8_t*)"turn left",12);			
			}
			
		}
		else if(fabs(front_infrafot_distanceL-front_infrafot_distanceR)<front_dis_err)
		{
			go_straight=1;
			OLED_ShowString(0,12*4,(uint8_t*)"go",12);			
		}
		else //在车尾
		{
			
		}
		if(outofstage!=0)
		{
			switch(outofstage)
			{
				case 1://车头出台
					OLED_ShowString(0,12*2,(uint8_t*)"headout",12);			
					break;
				case 2://车尾出台
					OLED_ShowString(0,12*2,(uint8_t*)"backout",12);						
					break;
				case 3://左侧出台
					OLED_ShowString(0,12*2,(uint8_t*)"leftout",12);						
					break;
				case 4://右侧出台
					OLED_ShowString(0,12*2,(uint8_t*)"rightout",12);						
					break;
			}
			outofstage=0;
		}
		sprintf(text,"FL:%.2f  FR:%.2f",front_infrafot_distanceL,front_infrafot_distanceR);
		OLED_ShowString(0,12*0,(uint8_t*)text,12);	
		sprintf(text,"l:%.2f  r:%.2f",left_infrafot_distance,right_infrafot_distance);
		OLED_ShowString(0,12*1,(uint8_t*)text,12);	
		OLED_Refresh();
    osDelay(1);
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
    osDelay(1);
  }
  /* USER CODE END OpenmvConnect */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

