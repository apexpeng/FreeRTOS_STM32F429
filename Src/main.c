/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "usart.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO口地址映射
#define GPIOA_ODR_Addr    (GPIOA_BASE+20) //0x40020014
#define GPIOB_ODR_Addr    (GPIOB_BASE+20) //0x40020414 
#define GPIOC_ODR_Addr    (GPIOC_BASE+20) //0x40020814 
#define GPIOD_ODR_Addr    (GPIOD_BASE+20) //0x40020C14 
#define GPIOE_ODR_Addr    (GPIOE_BASE+20) //0x40021014 
#define GPIOF_ODR_Addr    (GPIOF_BASE+20) //0x40021414    
#define GPIOG_ODR_Addr    (GPIOG_BASE+20) //0x40021814   
#define GPIOH_ODR_Addr    (GPIOH_BASE+20) //0x40021C14    
#define GPIOI_ODR_Addr    (GPIOI_BASE+20) //0x40022014 
#define GPIOJ_ODR_ADDr    (GPIOJ_BASE+20) //0x40022414
#define GPIOK_ODR_ADDr    (GPIOK_BASE+20) //0x40022814

#define GPIOA_IDR_Addr    (GPIOA_BASE+16) //0x40020010 
#define GPIOB_IDR_Addr    (GPIOB_BASE+16) //0x40020410 
#define GPIOC_IDR_Addr    (GPIOC_BASE+16) //0x40020810 
#define GPIOD_IDR_Addr    (GPIOD_BASE+16) //0x40020C10 
#define GPIOE_IDR_Addr    (GPIOE_BASE+16) //0x40021010 
#define GPIOF_IDR_Addr    (GPIOF_BASE+16) //0x40021410 
#define GPIOG_IDR_Addr    (GPIOG_BASE+16) //0x40021810 
#define GPIOH_IDR_Addr    (GPIOH_BASE+16) //0x40021C10 
#define GPIOI_IDR_Addr    (GPIOI_BASE+16) //0x40022010 
#define GPIOJ_IDR_Addr    (GPIOJ_BASE+16) //0x40022410 
#define GPIOK_IDR_Addr    (GPIOK_BASE+16) //0x40022810 

//IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入

#define PHout(n)   BIT_ADDR(GPIOH_ODR_Addr,n)  //输出 
#define PHin(n)    BIT_ADDR(GPIOH_IDR_Addr,n)  //输入

#define PIout(n)   BIT_ADDR(GPIOI_ODR_Addr,n)  //输出 
#define PIin(n)    BIT_ADDR(GPIOI_IDR_Addr,n)  //输入

#define PJout(n)   BIT_ADDR(GPIOJ_ODR_Addr,n)  //输出 
#define PJin(n)    BIT_ADDR(GPIOJ_IDR_Addr,n)  //输入

#define PKout(n)   BIT_ADDR(GPIOK_ODR_Addr,n)  //输出 
#define PKin(n)    BIT_ADDR(GPIOK_IDR_Addr,n)  //输入
#define LED0 PBout(1)   //LED0
#define LED1 PBout(0)   //LED1

#define KEY0        HAL_GPIO_ReadPin(GPIOH,GPIO_PIN_3)  //KEY0按键PH3
#define KEY1        HAL_GPIO_ReadPin(GPIOH,GPIO_PIN_2)  //KEY1按键PH2
#define KEY2        HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13) //KEY2按键PC13
#define WK_UP       HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)  //WKUP按键PA0

ADC_HandleTypeDef hadc1;
DAC_HandleTypeDef hdac;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
void start_task(void *pvParameters);
void KEY_task(void *pvParameters);
void DA_task(void *pvParameters);
void AD_task(void *pvParameters);
void led0_task(void *pvParameters);
void led1_task(void *pvParameters);
void Command_task(void *pvParameters);
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
//static void MX_USART1_UART_Init(void);

TaskHandle_t StartTask_Handler;
#define START_TASK_PRIO		1
#define START_STK_SIZE 		128

TaskHandle_t KEY_Task_Handler;
#define KEY_TASK_PRIO		5
#define KEY_STK_SIZE 		50

TaskHandle_t AD_Task_Handler;
#define AD_TASK_PRIO		6
#define AD_STK_SIZE 		128

TaskHandle_t DA_Task_Handler;
#define DA_TASK_PRIO		2
#define DA_STK_SIZE 		128

TaskHandle_t LED0Task_Handler;
#define LED0_TASK_PRIO		8
#define LED0_STK_SIZE 		50

TaskHandle_t LED1Task_Handler;
#define LED1_TASK_PRIO		9
#define LED1_STK_SIZE 		50

TaskHandle_t CommandTask_Handler;
#define COMMAND_TASK_PRIO		10
#define COMMAND_STK_SIZE 		128


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
unsigned char uart_buff[1]; 
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  uart_init(115200);

  /* USER CODE BEGIN 2 */
  //HAL_UART_Receive_IT(&huart1, uart_buff, sizeof(uart_buff));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    xTaskCreate((TaskFunction_t )start_task,            //任务函数
                (const char*    )"start_task",          //任务名称
                (uint16_t       )START_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )START_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&StartTask_Handler);   //任务句柄              
    vTaskStartScheduler();          //开启任务调度
			
}

//开始任务任务函数
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //进入临界区
	
//	xTaskCreate((TaskFunction_t ) KEY_task,
//			(const char*    )"KEY_task",   	
//			(uint16_t       )KEY_STK_SIZE, 
//			(void*          )NULL,				
//			(UBaseType_t    )KEY_TASK_PRIO,	
//			(TaskHandle_t*  )&KEY_Task_Handler);
			
	xTaskCreate((TaskFunction_t ) DA_task,     	
			(const char*    )"DA_task",   	
			(uint16_t       )DA_STK_SIZE, 
			(void*          )NULL,				
			(UBaseType_t    )DA_TASK_PRIO,	
			(TaskHandle_t*  )&DA_Task_Handler);   

	xTaskCreate((TaskFunction_t ) AD_task,
			(const char*    )"AD_task",   	
			(uint16_t       )AD_STK_SIZE, 
			(void*          )NULL,				
			(UBaseType_t    )AD_TASK_PRIO,	
			(TaskHandle_t*  )&AD_Task_Handler);   
	
//    //创建LED0任务
//    xTaskCreate((TaskFunction_t )led0_task,     	
//                (const char*    )"led0_task",   	
//                (uint16_t       )LED0_STK_SIZE, 
//                (void*          )NULL,				
//                (UBaseType_t    )LED0_TASK_PRIO,	
//                (TaskHandle_t*  )&LED0Task_Handler);   
//    //创建LED1任务
//    xTaskCreate((TaskFunction_t )led1_task,     
//                (const char*    )"led1_task",   
//                (uint16_t       )LED1_STK_SIZE, 
//                (void*          )NULL,
//                (UBaseType_t    )LED1_TASK_PRIO,
//                (TaskHandle_t*  )&LED1Task_Handler);        
//    //浮点测试任务
    xTaskCreate((TaskFunction_t )Command_task,     
                (const char*    )"com_task",   
                (uint16_t       )COMMAND_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )COMMAND_TASK_PRIO,
                (TaskHandle_t*  )&CommandTask_Handler);  
    vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
}

//KEY0, 1
void KEY_task(void *pvParameters)
{
    while(1)
    {
		//DO NOTHING
        //LED0=~LED0;
        vTaskDelay(500);
    }
}   

void DA_task(void *pvParameters)
{
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	double vol=3.18;
    while(1)
    {
		if(vol > 0) {
			vol = vol -1;
		} else {
			vol = vol +4;
		}
		double temp = vol;
		temp/=1000;
		temp=temp*4096/3.3;
		HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,temp);
		printf("~~\r\n");
		vTaskDelay(500);
    }
}   

void AD_task(void *pvParameters)
{
	u16 adc_reg = 0;
	float value;
	ADC_ChannelConfTypeDef ADC1_ChanConf;
	ADC1_ChanConf.Channel=5;                                   //通道
	ADC1_ChanConf.Rank=1;                                       //第1个序列，序列1
	ADC1_ChanConf.SamplingTime=ADC_SAMPLETIME_480CYCLES;        //采样时间
	ADC1_ChanConf.Offset=0;                 
	HAL_ADC_ConfigChannel(&hadc1,&ADC1_ChanConf);        //通道配置
	HAL_ADC_Start(&hadc1);                               //开启ADC
	HAL_ADC_PollForConversion(&hadc1,10);                //轮询转换
    while(1)
    {
		adc_reg = (u16)HAL_ADC_GetValue(&hadc1);	        //返回最近一次ADC1规则组的转换结果
		value = adc_reg * 3.3 /4096;
		printf("the adc value --- is %f\r\n", value);
        vTaskDelay(500);
    }
}

//LED0任务函数 
void led0_task(void *pvParameters)
{
    while(1)
    {
        //LED0=~LED0;
        vTaskDelay(500);
    }
}   

//LED1任务函数
void led1_task(void *pvParameters)
{
    while(1)
    {
        //LED1=0;
        vTaskDelay(200);
        //LED1=1;
        vTaskDelay(800);
    }
}

void Command_task(void *pvParameters)
{
	u8 len = 0;
	u16 times = 0;
	while(1) {
       if(USART_RX_STA&0x8000)
		{					   
			len=USART_RX_STA&0x3fff;//得到此次接收到的数据长度
			printf("\r\n您发送的消息为:\r\n");
			HAL_UART_Transmit(&UART1_Handler,(uint8_t*)USART_RX_BUF,len,1000);	//发送接收到的数据
			while(__HAL_UART_GET_FLAG(&UART1_Handler,UART_FLAG_TC)!=SET);		//等待发送结束
			printf("\r\n\r\n");//插入换行
			USART_RX_STA=0;
		}else
		{
			times++;
			if(times%5000==0)
			{
				printf("\r\nALIENTEK 阿波罗STM32F429开发板 串口实验\r\n");
				printf("正点原子@ALIENTEK\r\n\r\n\r\n");
			}
			if(times%200==0)printf("请输入数据,以回车键结束\r\n");  
			//if(times%30==0)LED0=!LED0;//闪烁LED,提示系统正在运行.
			vTaskDelay(100);   
		} 
	}
}



/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  HAL_PWREx_EnableOverDrive();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = DISABLE;
  HAL_ADC_Init(&hadc1);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

}

/* DAC init function */
void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac.Instance = DAC;
  HAL_DAC_Init(&hdac);

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1);

}

/* USART1 init function */
//void MX_USART1_UART_Init(void)
//{

//  huart1.Instance = USART1;
//  huart1.Init.BaudRate = 115200;
//  huart1.Init.WordLength = UART_WORDLENGTH_8B;
//  huart1.Init.StopBits = UART_STOPBITS_1;
//  huart1.Init.Parity = UART_PARITY_NONE;
//  huart1.Init.Mode = UART_MODE_TX_RX;
//  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
//  HAL_UART_Init(&huart1);

//}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PH2 PH3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET);

    
    //中断线0-PA0
//    HAL_NVIC_SetPriority(EXTI0_IRQn,2,0);       //抢占优先级为2，子优先级为0
//    HAL_NVIC_EnableIRQ(EXTI0_IRQn);             //使能中断线0
    
    //中断线2-PH2
    HAL_NVIC_SetPriority(EXTI2_IRQn,2,0);       //抢占优先级为2，子优先级为1
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);             //使能中断线2
    
    //中断线3-PH3
    HAL_NVIC_SetPriority(EXTI3_IRQn,2,0);       //抢占优先级为2，子优先级为2
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);             //使能中断线2
    
    //中断线13-PC13
    HAL_NVIC_SetPriority(EXTI15_10_IRQn,2,0);   //抢占优先级为2，子优先级为3
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);         //使能中断线13  


}

/* USER CODE BEGIN 4 */

////中断服务函数
//void EXTI0_IRQHandler(void)
//{
//    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);//调用中断处理公用函数
//}

void EXTI2_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);//调用中断处理公用函数
}

void EXTI3_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);//调用中断处理公用函数
}

void EXTI15_10_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);//调用中断处理公用函数
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    delay_ms(100);      //消抖
	printf("get key\r\n");
    switch(GPIO_Pin)
    {
//        case GPIO_PIN_0:
//            if(WK_UP==1) 
//            {
//				LED1=!LED1;//控制LED0,LED1互斥点亮
//				LED0=!LED1;
//            }
//            break;
        case GPIO_PIN_2:
            if(KEY1==0)  //LED1翻转
            {
                LED1=!LED1;    
            }
            break;
        case GPIO_PIN_3:
            if(KEY0==0)  //同时控制LED0,LED1翻转 
            {
                LED0=!LED0;
				LED1=!LED1;
            }
            break;

        case GPIO_PIN_13:
            if(KEY2==0)  
            {
				LED0=!LED0;//控制LED0翻转
            }
            break;
    }
}

/* USER CODE END 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
