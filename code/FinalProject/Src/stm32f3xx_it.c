/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "stm32f3xx.h"
#include "stm32f3xx_it.h"

/* USER CODE BEGIN 0 */
#include "stdio.h" 
#include "stdlib.h"
#include "time.h"
#include "LiquidCrystal.h"
extern int yeks;
int ses,ti,tj;
int stop=0;
int p1tick,p2tick,p3tick,p4tick,p5tick,p6tick,p7tick,p8tick;
int avlm=5,oldsp=5,pc=0,dpc=2;
extern unsigned char data[1];
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
static void MX_TIM2_Init(int spd)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  switch (spd){
		case 5:
			htim2.Init.Prescaler = 1439;
		  htim2.Init.Period = 100000;
		break;
		case 10:
			htim2.Init.Prescaler = 7199;
		  htim2.Init.Period = 10000;
		break;
		case 15:
			htim2.Init.Prescaler = 4799;
		  htim2.Init.Period = 10000;
		break;
		case 20:
			htim2.Init.Prescaler = 3599;
		  htim2.Init.Period = 10000;
    break;
		case 25:
			htim2.Init.Prescaler = 2879;
		  htim2.Init.Period = 10000;
    break;
		case 30:
			htim2.Init.Prescaler = 2399;
		  htim2.Init.Period = 10000;
    break;
	}
  htim2.Instance = TIM2;
  //htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  //htim2.Init.Period = 100000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}
extern int speed;
int sum=5,tir=0;
int chng=1;
int w=0,a=0,flag=0,d=0,tim=0;
extern int x,y;
extern int i,j,f,l,le;
typedef unsigned char byte;
extern byte up[8]; 
extern byte upvsib[8];
extern byte dwnvsib[8];
extern byte usib[8];
extern byte dsib[8];
extern byte dwn[8];
extern byte dota[8];
extern byte astin2[8];
extern byte shast[8];
extern byte dast[8];
extern byte avalsar[8];
extern byte dahan[8];
extern byte kolebadn[8];
extern byte siah[8];
extern byte sarebar[8];
extern byte tahebar[8];
extern byte badanebar[8];
extern byte harbar[8];
extern byte dobar[8];
extern byte nghte[8];
void numbertobcd(int i){
int x1=i&1;
int x2=i&2;
int x3=i&4;
int x4=i&8;
if(x1>0)
	x1=1;
if(x2>0)
	x2=1;
if(x3>0)
	x3=1;
if(x4>0)
	x4=1;
HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,x1);
HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,x2);
HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,x3);
HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,x4);

}
int mjl(int jl){
	switch (jl){
		case 0:
			return(0);
		break;
		case 1:
			return(0);
		break;
		case 2:
			return(1);
		break;
		case 3:
			return(1);
		break;
		case 4:
			return(2);
		break;
		case 5:
			return(2);
		break;
		case 6:
			return(3);
		break;
		case 7:
			return(3);
		break;
	}
}
extern int arr[20][8];
int rel,rej;
int vc=0;
void checkfl(){
	arr[f][l]=0;
		rel=mjl(l);
		setCursor(f,rel);
	if(l%2==0){
			if(arr[f][l+1]==0) print(" ");
      else if(arr[f][l+1]==1) write(2);			
			else if(arr[f][l+1]==3) write(5);
		}
		else if(l%2!=0){
			if(arr[f][l-1]==0) print(" ");
      else if(arr[f][l-1]==1) write(1);			
			else if(arr[f][l-1]==3) write(4);
		}
}
void checkijij(){
arr[i][j]=1;
		rej=mjl(j);
		setCursor(i,rej);
			if(j%2==0){
			if(arr[i][j+1]==0) write(1);	
			else if(arr[i][j+1]==1) write(3);
			else if(arr[i][j+1]==3) write(7);
		}
		else if(j%2!=0){
			if(arr[i][j-1]==0) write(2);	
			else if(arr[i][j-1]==1) write(3);
			else if(arr[i][j-1]==3) write(6);
		}
}
void checkij(){
		if(arr[i][j]!=1){
		arr[i][j]=1;
		rej=mjl(j);
		setCursor(i,rej);
			if(j%2==0){
			if(arr[i][j+1]==0) write(1);	
			else if(arr[i][j+1]==1) write(3);
			else if(arr[i][j+1]==3) write(7);
		}
		else if(j%2!=0){
			if(arr[i][j-1]==0) write(2);	
			else if(arr[i][j-1]==1) write(3);
			else if(arr[i][j-1]==3) write(6);
		}
		}
		else if(arr[i][j]==1) {
			clear();
			setCursor(4,1);
			print("Game Over!");
			int avg=((sum/chng)+le)*10;
			char aa[10];
			sprintf(aa,"%2d",avg);
			setCursor(7,3);
			print(aa);
			HAL_TIM_Base_Stop_IT(&htim2);
			vc=30;
			HAL_TIM_Base_Start_IT(&htim3);
		}
}
void checktij(){
		rej=mjl(tj);
		setCursor(ti,rej);
			if(j%2==0){
				write(8);
		}
		else if(j%2!=0){
			print("*");
		}
}
void lastcheck(){
	if(f==19&&arr[0][l]==1)f=0;
	else if(f==0&&arr[19][l]==1)f=19;
	else if(l==7&&arr[f][0]==1&&arr[f+1][l]!=1&&arr[f-1][l]!=1&&arr[f][l-1]!=1)l=0;
	else if(l==0&&arr[f][7]==1)l=7;
	else{
		if(arr[f][l-1]==1&&arr[f-1][l]==1&&arr[f-1][l+1]==1)l--;
		//else if(arr[f][l+1]==1&&arr[f+1][l]==1&&arr[f+1][l-1]==1)l++;
		else if(f!=0&&arr[f-1][l]==1)f--;
		else if(arr[f][l-1]==1)l--;
//		else if(arr[f][l+1]==1&&arr[f+1][l]==1&&arr[f+1][l-1]==1)l++;
		else if(arr[f][l+1]==1)l++;
		else if(arr[f+1][l]==1)f++;
		//else if(arr[f][l+1]==1)l++;
//		 if(arr[f][l-1]==1)l--;
//		else if(arr[f+1][l]==1) f++;
//		else if(arr[f-1][l]==1)f--;
//		else if(arr[f][l+1]==1)l++;
	}
}
extern int cnt,shuru;
int adad;
void apple(int time){
	if(speed>avlm){
		speed=avlm;
		MX_TIM2_Init(speed);
		}
	if(i==x&&j==y){
		//if(yeks<100) {tir=1;ses=0;}
		arr[x][y]=1;
		if(speed<avlm){
		speed=speed+5;
		}
		else{
		speed=avlm;
		}
		MX_TIM2_Init(speed);
		le++;
		int ay;
		if(pc==1){
			switch (dpc){
		case 1:
			if(j==0)j=8;
			j--;
		break;
		case 2:
			if(j==7)j=-1;
			j++;
		break;
		case 3:
			if(i==19)i=-1;
			i++;
		break;
		case 4:
			if(i==0)i=20;
			i--;
    break;
	}
		}
		else{
			switch (d){
		case 1:
			if(j==0)j=8;
			j--;
		break;
		case 2:
			if(j==7)j=-1;
			j++;
		break;
		case 3:
			if(i==19)i=-1;
			i++;
		break;
		case 4:
			if(i==0)i=20;
			i--;
    break;
	}
}
			checkij();
	//checkijij();
		do{
		//srand(time); 
		srand((int)__HAL_TIM_GET_COUNTER(&htim4)); 
	 x=rand()%20;
	 y=rand()%8;
	}
	while(arr[x][y]!=0);
	//yeks=0;
		arr[x][y]=3;
	ay=mjl(y);
	setCursor(x,ay);
			if(y%2==0){
			if(arr[x][y+1]==0) write(4);
      else if(arr[x][y+1]==1) write(6);	
		}
		else if(y%2!=0){
			if(arr[x][y-1]==0) write(5);
      else if(arr[x][y-1]==1) write(7);	
		}	
	}
	else{
		if(j>y&&x>i) {
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,1);
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15,0);
		}
		else if(j>y&&x<i) {
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,1);
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10|GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15,0);
		}
		else if(j<y&&x<i) {
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,1);
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10|GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_8|GPIO_PIN_15,0);
		}
		else if(j<y&&x>i) {
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,1);
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10|GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_8|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15,0);
		}
		else if(j>y&&x==i) {
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,1);
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10|GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15,0);
		}
		else if(j<y&&x==i) {
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,1);
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10|GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_8|GPIO_PIN_14|GPIO_PIN_15,0);
		}
		else if(j==y&&x<i) {
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,1);
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10|GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_8,0);
		}
		else if(j==y&&x>i) {
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,1);
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10|GPIO_PIN_9|GPIO_PIN_8|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15,0);
		}
	}
}
void checkemt(){
	if(arr[i][j+1]!=1){
		if(dpc!=1){
				if(j==7)j=-1;
			  j++;
				dpc=2;
				}
				else{
					if(arr[i+1][j]==1&&i!=19){
						checkemt();
		      }
					else{
						if(i==19)i=-1;
			      i++;
				    dpc=3;
					}
				}
	}
	else if(arr[i+1][j]!=1){
		if(dpc!=4){
				if(i==19)i=-1;
			  i++;
				dpc=3;
				}
				else{
					if(arr[i][j+1]==1&&j!=7){
						checkemt();
		      }
					else{
						if(j==7)j=-1;
			      j++;
				    dpc=2;
					}
				}
	}
	else if(arr[i-1][j]!=1){
		if(dpc!=3){
				if(i==0)i=20;
			  i--;
				dpc=4;
				}
				else{
					if(arr[i][j+1]==1&&j!=7){
						checkemt();
		      }
					else{
						if(j==7)j=-1;
			      j++;
				    dpc=2;
					}
				}
	}
	else if(arr[i][j-1]!=1){
		if(dpc!=2){
				if(j==0)j=8;
			  j--;
				dpc=1;
				}
				else{
					if(arr[i+1][j]==1&&i!=19){
						checkemt();
		      }
					else{
						if(i==19)i=-1;
			      i++;
				    dpc=3;
					}
				}
	}
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart2;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles ADC1 and ADC2 interrupts.
*/
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */

  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc2);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */
	int vlm=HAL_ADC_GetValue(&hadc2);
	adad=((int)vlm*15/63);
	switch (adad){
		case 9:
			avlm=5;
		break;
		case 10:
			avlm=10;
		break;
		case 11:
			avlm=15;
		break;
		case 12:
			avlm=20;
    break;
		case 13:
			avlm=25;
    break;
		case 14:
			avlm=30;
    break;
	}
	HAL_ADC_Start_IT(&hadc2);
  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
* @brief This function handles EXTI line[9:5] interrupts.
*/
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */
	for(int i=0;i<4;i++){
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
	
		if(i==0){
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET);
			if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_6)){
				if(HAL_GetTick()>p4tick+5){
					p4tick=HAL_GetTick();
				d=4;
			}
		}
		}
		if(i==1){
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET);
			if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_6)){
				if(HAL_GetTick()>p5tick+5){
					p5tick=HAL_GetTick();
				d=2;
			}
		}
		}
		if(i==2){
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);
			if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_6)){
				if(HAL_GetTick()>p6tick+5){
					p6tick=HAL_GetTick();
				d=3;
				}
			}
		}
	}
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
	if(oldsp!=speed){
		sum=sum+speed;
		chng++;
		oldsp=speed;
	}
	//ti=i;
	//tj=j;
	checkfl();
  lastcheck();
	if(pc==0){
	switch (d){
		case 1:
		if(j==0)j=8;
		j--;
		break;			
		case 2:
    if(j==7)j=-1;
		j++;
		break;		
		case 3:
		if(i==19)i=-1;
		i++;
		break;
		case 4:
  	if(i==0)i=20;
		i--;
    break;
	}
}
	else{
		if(j>y&&x==i) {
			if(arr[i][j-1]==1&&j!=0){
				checkemt();
			}
			else{
				if(dpc!=2){
				if(j==0)j=8;
			  j--;
				dpc=1;
				}
				else{
					if(arr[i+1][j]==1&&i!=19){
						checkemt();
		      }
					else{
						if(i==19)i=-1;
			      i++;
				    dpc=3;
					}
				}
			}
		}
		///////////////////////////////////
		else if(j<y&&x==i) {
			if(arr[i][j+1]==1&&j!=7){
				checkemt();
			}
			else{
				if(dpc!=1){
				if(j==7)j=-1;
			  j++;
				dpc=2;
				}
				else{
					if(arr[i+1][j]==1&&i!=19){
						checkemt();
		      }
					else{
						if(i==19)i=-1;
			      i++;
				    dpc=3;
					}
				}
			}
		}
		///////////////////////////////////
		else if(j==y&&x<i) {
			if(arr[i-1][j]==1&&i!=0){
				checkemt();
			}
			else{
				if(dpc!=3){
				if(i==0)i=20;
			  i--;
				dpc=4;
				}
				else{
					if(arr[i][j+1]==1&&j!=7){
						checkemt();
		      }
					else{
						if(j==7)j=-1;
			      j++;
				    dpc=2;
					}
				}
			}
		}
		/////////////////////////////////
		else if(j==y&&x>i) {
			if(arr[i+1][j]==1&&i!=19){
				checkemt();
			}
			else{
				if(dpc!=4){
				if(i==19)i=-1;
			  i++;
				dpc=3;
				}
				else{
					if(arr[i][j+1]==1&&j!=7){
						checkemt();
		      }
					else{
						if(j==7)j=-1;
			      j++;
				    dpc=2;
					}
				}
			}
		}
		////////////////////////////////
		else if(j>y&&x>i) {
			if(arr[i+1][j]==1&&i!=19){
				checkemt();
			}
			else{
				if(dpc!=4){
				if(i==19)i=-1;
			  i++;
				dpc=3;
				}
				else{
					if(arr[i][j+1]==1&&j!=7){
						checkemt();
		      }
					else{
						if(j==0)j=8;
			      j--;
				    dpc=1;
					}
				}
			}
//			if(arr[i][j-1]==1&&j!=0){
//				checkemt();
//			}
//			else{
//				if(dpc!=2){
//				if(j==0)j=8;
//			  j--;
//				dpc=1;
//				}
//				else{
//					if(arr[i+1][j]==1&&i!=19){
//						checkemt();
//		      }
//					else{
//						if(i==19)i=-1;
//			      i++;
//				    dpc=3;
//					}
//				}
//			}
		}
		else if(j>y&&x<i) {
//			if(arr[i][j-1]==1&&j!=0){
//				checkemt();
//			}
//			else{
//				if(dpc!=2){
//				if(j==0)j=8;
//			  j--;
//				dpc=1;
//				}
//				else{
//					if(arr[i-1][j]==1&&i!=0){
//						checkemt();
//		      }
//					else{
//						if(i==0)i=20;
//			      i--;
//				    dpc=4;
//					}
//				}
//			}
						if(arr[i-1][j]==1&&i!=0){
				checkemt();
			}
			else{
				if(dpc!=3){
				if(i==0)i=20;
			  i--;
				dpc=4;
				}
				else{
					if(arr[i][j-1]==1&&j!=0){
				checkemt();
			}
			else{
				if(j==0)j=8;
			  j--;
				dpc=1;
				}
				}
			}
		}
		else if(j<y&&x<i) {
			if(arr[i-1][j]==1&&i!=0){
				checkemt();
			}
			else{
				if(dpc!=3){
				if(i==0)i=20;
			  i--;
				dpc=4;
				}
				else{
					if(arr[i][j+1]==1&&j!=7){
						checkemt();
		      }
					else{
						if(j==7)j=-1;
			      j++;
				    dpc=2;
					}
				}
			}
		}
		else if(j<y&&x>i) {
//			if(arr[i][j+1]==1&&j!=7){
//				checkemt();
//			}
//			else{
//				if(dpc!=1){
//				if(j==7)j=-1;
//			  j++;
//				dpc=2;
//				}
//				else{
//					if(arr[i+1][j]==1&&i!=19){
//						checkemt();
//		      }
//					else{
//						if(i==19)i=-1;
//			      i++;
//				    dpc=3;
//					}
//				}
//			}
			if(arr[i+1][j]==1&&i!=19){
				checkemt();
			}
			else{
				if(dpc!=4){
				if(i==19)i=-1;
			  i++;
				dpc=3;
				}
				else{
					if(arr[i][j+1]==1&&j!=7){
						checkemt();
		      }
					else{
						if(j==7)j=-1;
			      j++;
				    dpc=2;
					}
				}
			}
		}
//		else{
//						if(j==7)j=-1;
//			      j++;
//				    dpc=2;
//					}
	}
	checkij();
	tim++;
  apple(tim);
  /* USER CODE END TIM2_IRQn 1 */
}

/**
* @brief This function handles TIM3 global interrupt.
*/
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
//	if(tir==1&&ses<300){
//		switch (d){
//		case 1:
//		if(tj==0)tj=j;
//		tj--;
//		break;			
//		case 2:
//    if(tj==7)tj=j;
//		tj++;
//		break;		
//		case 3:
//		if(ti==19)ti=i;
//		ti++;
//		break;
//		case 4:
//  	if(ti==0)ti=i;
//		ti--;
//    break;
//	}
//		checktij();
//	}
	tim++;
	if(vc>29){
		if(vc%4==0){
createChar(4,astin2);
createChar(5,shast);
createChar(7,dast);
setCursor(14,1);
write(4);
setCursor(14,2);
write(5);
setCursor(15,1);
write(7);
		}
		else if(vc%4==1){
createChar(4,astin2);
createChar(5,shast);
createChar(7,dast);
setCursor(14,1);
write(4);
setCursor(14,2);
write(5);
setCursor(15,1);
write(7);
		}
		else if(vc%4==2){
setCursor(14,1);
print(" ");
setCursor(14,2);
print(" ");
setCursor(15,1);
print(" ");
		createChar(4,astin2);
createChar(5,shast);
createChar(7,dast);
setCursor(14,2);
write(4);
setCursor(14,3);
write(5);
setCursor(15,2);
write(7);
		}
		else if(vc%4==3){
		createChar(4,astin2);
createChar(5,shast);
createChar(7,dast);
setCursor(14,2);
write(4);
setCursor(14,3);
write(5);
setCursor(15,2);
write(7);
			setCursor(14,2);
print(" ");
setCursor(14,3);
print(" ");
setCursor(15,2);
print(" ");
		}
	}
else if(shuru==1){
	clear();
createChar(1,up);
createChar(2,dwn);
createChar(3,dota);
createChar(4,usib);
createChar(5,dsib);
createChar(6,dwnvsib);
createChar(7,upvsib);
createChar(8,nghte);
home();

write(1);
write(1);
write(1);
		 i=2;
		 j=0;
		 f=0;
		 l=0;
		 le=3;
		 arr[0][0]=1;
		 arr[1][0]=1;
		 arr[2][0]=1;
	do{
		//srand(tim); 
		srand((int)__HAL_TIM_GET_COUNTER(&htim3));
	 x=rand()%20;
	 y=rand()%8;
	}
	while(arr[x][y]!=0);
	arr[x][y]=3;
	int yy=mjl(y);
	setCursor(x,yy);
			if(y%2==0){
			if(arr[x][y+1]==0) write(4);
      else if(arr[x][y+1]==1) write(6);		
		}
		else if(y%2!=0){
			if(arr[x][y-1]==0) write(5);
      else if(arr[x][y-1]==1) write(7);		
		}		
		
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
		
HAL_UART_Receive_IT(&huart2,data,sizeof(data));
		HAL_ADC_Start_IT(&hadc2);
		HAL_TIM_Base_Start_IT(&htim4);
		d=3;
		HAL_TIM_Base_Stop_IT(&htim3);
	}
else{
	if(vc<21){
	createChar(1,avalsar);
	createChar(2,dahan);
	createChar(3,kolebadn);
	createChar(4,siah);
	if(vc==0){
	setCursor(5,1);
	write(4);
	setCursor(9,1);
	write(4);
	setCursor(13,1);
	write(4);
	setCursor(17,1);
	write(4);
	}
	if(vc<20){
		setCursor(vc-1,1);
	write(3);
	setCursor(vc,1);
	write(1);
	}
	if(vc>1&&vc<19&&vc%4==0){
	setCursor(vc,1);
	write(2);
	}
}
	if(vc==21) clear();
else if(vc>20){
	setCursor(6,1);
	print("Snake");
	setCursor(3,3);
	print("Human");
	setCursor(9,3);
	print("Computer");
	vc--;
 }
if(vc==23){
	clear();
setCursor(7,1);
		print("Human");
}
if(vc==24){
	clear();
setCursor(6,1);
		print("Computer");
}

	if(vc==24||vc==23){
		createChar(1,sarebar);
	createChar(2,tahebar);
	createChar(3,harbar);
	createChar(4,badanebar);
		createChar(5,dobar);
		setCursor(5,2);
	write(1);
	setCursor(6,2);
	write(4);
	setCursor(7,2);
	write(4);
		setCursor(8,2);
	write(4);
		setCursor(9,2);
	write(4);
		setCursor(10,2);
	write(4);
		setCursor(11,2);
	write(4);
		setCursor(12,2);
	write(4);
	setCursor(13,2);
	write(4);
	setCursor(14,2);
	write(2);
	
		setCursor(6,2);
	write(3);
	setCursor(6,2);
	write(5);
		setCursor(7,2);
	write(3);
		setCursor(7,2);
	write(5);
		setCursor(8,2);
	write(3);
		setCursor(8,2);
	write(5);
		setCursor(9,2);
	write(3);
	setCursor(9,2);
	write(5);
	setCursor(10,2);
	write(3);
	setCursor(10,2);
	write(5);
	setCursor(11,2);
	write(3);
	setCursor(11,2);
	write(5);
	setCursor(12,2);
	write(3);
	setCursor(12,2);
	write(5);
	setCursor(13,2);
	write(3);
	setCursor(13,2);
	write(5);
	shuru=1;
			HAL_TIM_Base_Stop_IT(&htim3);
			HAL_TIM_Base_Start_IT(&htim2);
	}
}
		vc++;
  /* USER CODE END TIM3_IRQn 1 */
}

/**
* @brief This function handles TIM4 global interrupt.
*/
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */
	if(cnt%4==0){
	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,1);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,1);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,1);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,0);
		numbertobcd (speed/10);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,0);
	}
	else if(cnt%4==1){
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,0);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,1);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,1);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,1);
		numbertobcd (speed%10);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,1);
	}
	//le speed
	else if(cnt%4==2){
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,1);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,0);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,1);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,1);
		numbertobcd (le/10);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,0);
	}
	else if(cnt%4==3){
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,1);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,1);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,0);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,1);
		numbertobcd (le%10);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,0);
	}
		cnt++;
//	if(tir==1)ses++;
//	yeks++;
  /* USER CODE END TIM4_IRQn 1 */
}

/**
* @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
*/
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
	
	switch (data[0]){
		case 'w':
			d=1;
		//HAL_TIM_Base_Start_IT(&htim2);
		break;
		case 's':
			d=2;
		//HAL_TIM_Base_Start_IT(&htim2);
		break;
		case 'd':
			d=3;
		//HAL_TIM_Base_Start_IT(&htim2);
		break;
		case 'a':
			d=4;
		//HAL_TIM_Base_Start_IT(&htim2);
    break;
		case 'e':
		  vc=24;
		  //HAL_TIM_Base_Start_IT(&htim3);
    break;
		case 'r':
			pc=1;
			vc=25;
		  //HAL_TIM_Base_Start_IT(&htim3);
    break;
	}
	HAL_UART_Receive_IT(&huart2,data,sizeof(data));
  /* USER CODE END USART2_IRQn 1 */
}

/**
* @brief This function handles EXTI line[15:10] interrupts.
*/
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */
for(int i=0;i<4;i++){
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
	
		if(i==1){
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET);
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15)){
				if(HAL_GetTick()>p1tick+5){
					p1tick=HAL_GetTick();
						d=1;
				}
			}
		}
		if(i==2){
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15)){
				if(HAL_GetTick()>p2tick+5){
					p2tick=HAL_GetTick();
				vc=24;
			}
		}
		}
		if(i==3){
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15)){
				if(HAL_GetTick()>p3tick+5){
					p3tick=HAL_GetTick();
				vc=25;
				pc=1;
			}
		}
		}
		if(i==4){
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET);
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15)){
				if(HAL_GetTick()>p7tick+5){
					p7tick=HAL_GetTick();
					if(stop%2==0){
						HAL_TIM_Base_Stop_IT(&htim4);
						HAL_TIM_Base_Stop_IT(&htim3);
						HAL_TIM_Base_Stop_IT(&htim2);
					}
						else if(stop%2==1){
							HAL_TIM_Base_Start_IT(&htim4);
						HAL_TIM_Base_Start_IT(&htim3);
						HAL_TIM_Base_Start_IT(&htim2);
						}
				stop++;
			}
		}
		}
	}
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
