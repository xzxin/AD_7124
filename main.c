/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include <math.h>
#include "dac.h"
#include "dma.h"
#include "i2c.h"
#include "sdadc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "types.h"
#include "Communication.h"
#include "AD7124.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
extern DMA_HandleTypeDef hdma_sdadc1;
void SystemClock_Config(void);
void Error_Handler(void);
void AD7124_Init();
uint8 gu8main_UartTest[100]={0},gu8main_UartTest2[100],sorttemp[64];
uint8 gu8main_UartRecvTest[100];
uint32_t gu32main_SDADCTest[500];
int32_t  gi32main_7124Test[100]={0};
uint16_t gu16main_actual[200];
uint32_t avg=0;
uint8 gu8main_UartRecvTestBuf[1];
uint8 gu8main_TransOrder[10];
uint8 gu8main_RecvOrder[10];
volatile int32_t ad_temp;
volatile uint16_t SDADCcollect_num;
volatile char dataCorrect = 1;
volatile char datahead[16]={0};//存储上个数据的数据头 判断数据是否为异常数据
uint8 sizea=0;
float correct;
int32_t *p32 = &ad_temp;//gi32main_7124Test;
uint8 use16CH;
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
void NOP(int nop)
{while (nop--);}
char dataCheck()
{
	uint8_t i;

		for (i=0;i<16;i++)
		{
			if ((gu8main_UartTest[5*i+2]&0xF8)!=(datahead[i]&0xF8)) 
			return 1;
		}

 return 0;
}

void dataSort()
{
	uint8_t i,j;
	for (i=0;i<8;i++)
	{
		for (j=0;j<8;j++)
		{
			if ((gu8main_UartTest[5*j+5]&7)==i)
			{
			sorttemp[i*4] = gu8main_UartTest[5*j+2];
			sorttemp[i*4+1] = gu8main_UartTest[5*j+3];
			sorttemp[i*4+2] = gu8main_UartTest[5*j+4];
			sorttemp[i*4+3] = gu8main_UartTest[5*j+5];		 
			}
			if ((gu8main_UartTest[5*(j+8)+5]&7)==i&&use16CH)
			{
			sorttemp[(i+8)*4] = gu8main_UartTest[5*(j+8)+2];
			sorttemp[(i+8)*4+1] = gu8main_UartTest[5*(j+8)+3];
			sorttemp[(i+8)*4+2] = gu8main_UartTest[5*(j+8)+4];
			sorttemp[(i+8)*4+3] = gu8main_UartTest[5*(j+8)+5];		 
			}
		}
	}
	for (i=0;i<8;i++)
	{
		gu8main_UartTest[1+5*i+1]= sorttemp[(7-i)*4];
		gu8main_UartTest[1+5*i+2]= sorttemp[(7-i)*4+1];
		gu8main_UartTest[1+5*i+3]= sorttemp[(7-i)*4+2];
		gu8main_UartTest[1+5*i+4]= sorttemp[(7-i)*4+3];
		
		gu8main_UartTest[1+5*(i+8)+1]= sorttemp[(15-i)*4];
		gu8main_UartTest[1+5*(i+8)+2]= sorttemp[(15-i)*4+1];
		gu8main_UartTest[1+5*(i+8)+3]= sorttemp[(15-i)*4+2];
		gu8main_UartTest[1+5*(i+8)+4]= sorttemp[(15-i)*4+3];
	}
}
int main(void)
{

  /* USER CODE BEGIN 1 */
	uint8 i,flag=0;
	char SDADC_SEND[20];
	double temper_volt,temper;
	uint8 *p;
	int32_t temp=1;
	uint32_t sdadc_temp=0;
	//int32_t *p32 = &temp;
	uint8_t buf[4]={0},firstData=1;
  /* USER CODE END 1 */
	if (1)
	{
		use16CH = 1;
	}
	else
	{
		use16CH = 0;
	}
  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
 // MX_DAC1_Init();
//  MX_I2C1_Init();
  MX_SDADC1_Init();
  MX_SDADC3_Init();
//  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
//  MX_TIM3_Init();
//  MX_TIM14_Init();
//  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();	
	AD7124_Init();
	flag=reg1[AD7124_ID_REG].value;

 //从通道0开始
 while (1)
 {
	 gu8main_UartTest[0]=45;
	 HAL_SDADC_Start_DMA(&hsdadc1,gu32main_SDADCTest,300);
	 if (dataCheck())
	 {
		for (i=0;i<16;i++) //读  下次比较
			{
				datahead[i] =gu8main_UartTest[1+5*i+1];
			}
		}
	 
		
	for(i=0;i<8;i++)
	{
		//HAL_Delay(3);
		while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9));
		NOP(500);
		while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9));
		NOP(500);

		AD7124_ReadData(&ad7124_1, p32);
		gu8main_UartTest[5*i+1] = 55;//7124数据头 有datastatus位
		gu8main_UartTest[5*i+2] = ((*p32)>>24);
		gu8main_UartTest[5*i+3] = ((*p32)>>16);
		gu8main_UartTest[5*i+4] = ((*p32)>>8);
		gu8main_UartTest[5*i+5] = ((*p32));//后3位代表通道
		//p32++;
		//HAL_Delay(3);
		while (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9));
		NOP(500);
		while (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9));
		NOP(500);
	}
		if (use16CH)
	{
		gu8main_UartTest[0]=85;
		for(i=0;i<8;i++)
		{
			//HAL_Delay(3);
			while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11));
			NOP(500);
			while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11));
			NOP(500);

			AD7124_ReadData(&ad7124_2, p32);
			gu8main_UartTest[5*(i+8)+1] = 55;//7124数据头 有datastatus位
			gu8main_UartTest[5*(i+8)+2] = ((*p32)>>24);
			gu8main_UartTest[5*(i+8)+3] = ((*p32)>>16);
			gu8main_UartTest[5*(i+8)+4] = ((*p32)>>8);
			gu8main_UartTest[5*(i+8)+5] = ((*p32));//后3位代表通道
		//p32++;
		//HAL_Delay(3);
			while (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11));
			NOP(500);
			while (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11));
			NOP(500);
		}
	}
		dataSort();
	HAL_SDADC_Stop_DMA(&hsdadc1);
	SDADCcollect_num = 300 -__HAL_DMA_GET_COUNTER(&hdma_sdadc1);
	sdadc_temp = 0;
	for (i=0;i<SDADCcollect_num/2;i++)
	{
		sdadc_temp+=(gu32main_SDADCTest[i]&0xFFFF)+(gu32main_SDADCTest[i]>>16)&0xFFFF;
	}
		sdadc_temp /= SDADCcollect_num;
		gu8main_UartTest[41+use16CH*40] = 66;
		gu8main_UartTest[42+use16CH*40] = sdadc_temp>>8;
		gu8main_UartTest[43+use16CH*40] = sdadc_temp;
		gu8main_UartTest[44+use16CH*40] = 0;
		gu8main_UartTest[45+use16CH*40] = 0x30;//
	/*
		if (firstData)
	{
				for (i=0;i<16;i++) //读  下次比较
			{
				datahead[i] =gu8main_UartTest[1+5*i+1];
			}
				HAL_UART_Transmit(&huart2,gu8main_UartTest,46+use16CH*40, 10000);
				HAL_UART_Transmit(&huart3,gu8main_UartTest,46+use16CH*40, 10000);
	}
	else
	{
		if (!dataCheck())
		{
			HAL_UART_Transmit(&huart2,gu8main_UartTest,46+use16CH*40, 10000);
			HAL_UART_Transmit(&huart3,gu8main_UartTest,46+use16CH*40, 10000);
		}
	}
	*/
			firstData=0;	
			temper_volt=(double)sdadc_temp/65536*2.487;//3.3;//
			temper=temper_volt/1.588*1000-273.15-2.5;//ad592AN MAX标定误差2.5V
			sprintf(SDADC_SEND,"%lf",temper);
			SDADC_SEND[strlen(SDADC_SEND)]='\n';
			SDADC_SEND[strlen(SDADC_SEND)]='\0';
			HAL_UART_Transmit(&huart3,SDADC_SEND,strlen(SDADC_SEND), 10000);
			
		

}


//p=gu8main_UartRecvTest;
//HAL_SDADC_Start_DMA(&hsdadc1,gu32main_SDADCTest,200);
//HAL_Delay(200);
for (i=0;i<100;i++)
{
	gu16main_actual[2*i] = gu32main_SDADCTest[i];
	gu16main_actual[2*i+1] = gu32main_SDADCTest[i]>>16;
	avg+=gu16main_actual[2*i];
	avg+=gu16main_actual[2*i+1];  
	
}
avg /=200;
while (1)
//{
//	if (*p != 0)
//	HAL_UART_Transmit(&huart1, p++, 1, 10000);
//	if (p == gu8main_UartRecvTest+100) p=gu8main_UartRecvTest;
///* USER CODE BEGIN 2 */
//}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_SDADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
  PeriphClkInit.SdadcClockSelection = RCC_SDADCSYSCLK_DIV12;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_PWREx_EnableSDADC(PWR_SDADC_ANALOG1);

  HAL_PWREx_EnableSDADC(PWR_SDADC_ANALOG3);

  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 
//	 while (1)
// {
//	 HAL_SDADC_Start_DMA(&hsdadc1,gu32main_SDADCTest,500);
//	 HAL_Delay(400);
//	 HAL_SDADC_Stop_DMA(&hsdadc1);
//		sdadc_temp = 0;
//	 sdadc_temp2=0;
//	for (i=0;i<250;i++)
//	{
//		sdadc_temp+=(gu32main_SDADCTest[i]&0xFFFF)+(gu32main_SDADCTest[i]>>16)&0xFFFF;
//	}
//		sdadc_temp /= 500;
//		for (i=0;i<250;i++)
//	{
//		sdadc_temp+=(gu32main_SDADCTest[i]&0xFFFF)+(gu32main_SDADCTest[i]>>16)&0xFFFF;
//		sdadc_temp2+=pow(((gu32main_SDADCTest[i]&0xFFFF)-sdadc_temp),2)+pow(((gu32main_SDADCTest[i]>>16)&0xFFFF-sdadc_temp),2);
//	}
//	correct = 16 - log(sdadc_temp2/500+1)/log(2)/2;
// }
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
