/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "driver.h"
#include "include.h"
#include "key_exti.h"
#include "rocker.h"
#include "rc.h"
#include "timer.h"
#include "oled.h"


/*
	NVIC 可以配置16种中断向量的优先级，即NVIC由一个4位的数字来决定，将这4位数字分配成
	抢占优先级和响应优先级部分，有5组分配方式：
			抢占    响应 
	0组：   4		0
	1组： 	1		3
	2组：   2		2
	3组：	3		1
	4组：   0		4 

	NVIC_PriorityGroup_0-4

*/
void Plane_Nvic_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	//2位抢占， 2位响应 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 	
	
	//usart1 
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&NVIC_InitStructure);
	
	//tim2 
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	// 0-4 line interrupt  这个地方一定要分开写，写在一起就不能独立工作了。
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;            
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;            
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;            
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;            
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;            
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	
	//5/9 line interrupt 	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;            
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	//10/15 line interrupt 
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;         
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	//Rocker DMA 
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn ; 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
    NVIC_Init(&NVIC_InitStructure);          
}


// EXTI 0/1  shell P/I
void EXTI0_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line0) != RESET) 
	{	
		switch(oled_show_item)
		{
			case 0: break;
			case 1: break;
			case 2: ctrl.roll.shell.kp += 1.0f; break; //roll
			case 3: ctrl.pitch.shell.kp += 1.0f; break; //pitch
			case 4: ctrl.yaw.shell.kp += 1.0f; break; //yaw
		}
		
		EXTI_ClearITPendingBit(EXTI_Line0);  
	}
}


void EXTI1_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line1) != RESET) 
	{	
		switch(oled_show_item)
		{
			case 0: break;
			case 1: break;
			case 2: ctrl.roll.shell.ki += 0.01f; break; //roll
			case 3: ctrl.pitch.shell.ki += 0.01f; break; //pitch
			case 4: ctrl.yaw.shell.ki += 0.01f; break; //yaw
		}
		
		EXTI_ClearITPendingBit(EXTI_Line1);    
	}
}

//EXTI 2/3/4  core  P/I/D
void EXTI2_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line2) != RESET) 
	{	
		switch(oled_show_item)
		{
			case 0: break;
			case 1: break;
			case 2: ctrl.roll.core.kp += 0.01f; break; //roll
			case 3: ctrl.pitch.core.kp += 0.01f; break; //pitch
			case 4: ctrl.yaw.core.kp += 0.01f; break; //yaw
		}
		
		EXTI_ClearITPendingBit(EXTI_Line2);
	}
}

void EXTI3_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line3) != RESET) 
	{	
		switch(oled_show_item)
		{
			case 0: break;
			case 1: break;
			case 2: ctrl.roll.core.ki += 0.01f; break; //roll
			case 3: ctrl.pitch.core.ki += 0.01f; break; //pitch
			case 4: ctrl.yaw.core.ki += 0.01f; break; //yaw
		}
		
		EXTI_ClearITPendingBit(EXTI_Line3);  
	}
}

void EXTI4_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line4) != RESET) 
	{	
		switch(oled_show_item)
		{
			case 0: break;
			case 1: break;
			case 2: ctrl.roll.core.kd += 0.01f; break; //roll
			case 3: ctrl.pitch.core.kd += 0.01f; break; //pitch
			case 4: ctrl.yaw.core.kd += 0.01f; break; //yaw
		}
		
		EXTI_ClearITPendingBit(EXTI_Line4); 
	}
}

void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line8) != RESET) 
	{	
		switch(oled_show_item)
		{
			case 0: break;
			case 1: break;
			case 2: ctrl.roll.core.kp -= 0.01f; break; //roll
			case 3: ctrl.pitch.core.kp -= 0.01f; break; //pitch
			case 4: ctrl.yaw.core.kp -= 0.01f; break; //yaw
		}
		
		EXTI_ClearITPendingBit(EXTI_Line8);  
	}
	
	if(EXTI_GetITStatus(EXTI_Line9) != RESET) 
	{
		switch(oled_show_item)
		{
			case 0: break;
			case 1: break;
			case 2: ctrl.roll.core.kd -= 0.01f; break; //roll
			case 3: ctrl.pitch.core.kd -= 0.01f; break; //pitch
			case 4: ctrl.yaw.core.kd -= 0.01f; break; //yaw
		}
	
		EXTI_ClearITPendingBit(EXTI_Line9);     
	}
}


/* 
oled_show_item = 0; //hint 
oled_show_item = 1; //euler
oled_show_item = 2; //roll pid
oled_show_item = 3; //pitch pid
oled_show_item = 4; //yaw pid
*/
void EXTI15_10_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line10) != RESET)  //euler 
	{	
		OLED_Clear();
		oled_show_item = 1;
		
		EXTI_ClearITPendingBit(EXTI_Line10);    
	}
	
	if(EXTI_GetITStatus(EXTI_Line11) != RESET)  //roll.pitch.yaw pid 
	{
		OLED_Clear();
		
		oled_show_item = 2;  	//roll
		//oled_show_item = 3;   //pitch
		//oled_show_item = 4;  	//yaw
		
		EXTI_ClearITPendingBit(EXTI_Line11);     
	}
	
	// rocker key1 
	if(EXTI_GetITStatus(EXTI_Line14) != RESET)  //arm = 1
	{
		rc_data.armed_val = 1;
		
		EXTI_ClearITPendingBit(EXTI_Line14);    
	}
	//rocker key2
	if(EXTI_GetITStatus(EXTI_Line15) != RESET)  //arm = 0
	{
		rc_data.armed_val = 0;
		
		EXTI_ClearITPendingBit(EXTI_Line15);    
	}
}










































/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 



/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
