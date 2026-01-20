    
/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdio.h> // иначе snprintf - warning. Пришлось добавить
#include "TM1637.h"
#include "diff_com.h"

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart2;


//DMA_HandleTypeDef hdma_adc1;


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);


uint8_t iset = 1;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	iset = 1;
}



int main(void)
{

  HAL_Init();
	SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM1_Init();
	MX_TIM2_Init();
  MX_TIM3_Init();
	MX_USART2_UART_Init();
	MX_ADC1_Init();	

	
	uint8_t hum1, h1, h2;
	int8_t temp1, t1, t2 ;
	float hum_update;
	char msg[50];
	char msg_adc[40];
	volatile uint32_t mes = 0;	 
	uint8_t adc_valuec[2];
	float adc_voltage = 0;
	float adc_voltage_2 = 0;
	
	uint32_t hold_time = 0;
	uint8_t new_brightness = 0;
	
	
	//int8_t a = char2segments('p');  // использовал для теста вывода 1 сегмента
	
	//*****************************************************************************************************
	//tm1637_TurnOff();
	
	func_hello();
	HAL_Delay(1000);
	func_name_device();
	HAL_Delay(1000);
	
	DHT_data data  = DHT_getData(DHT22);
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);	
	delay_ms(500);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);	
	HAL_Delay(500);
	
	
  while (1)
  {
		
		DHT_data data  = DHT_getData(DHT22);
		
		HAL_Delay(50);
		
		hum1 = data.hum - 10;
		temp1 = data.temp;
		hum_update = data.hum - 10;
		
		snprintf (msg, sizeof(msg), "Humidity: %.2f%%\n\r", hum_update);
		if (iset == 1){
			HAL_UART_Transmit_IT(&huart2, (uint8_t*)msg, strlen(msg));
			iset = 0;
		}
		
		HAL_Delay(2);
		
		snprintf (msg, sizeof(msg), "Temperature: %.2f C\n\r", data.temp);
		if (iset == 1){
			HAL_UART_Transmit_IT(&huart2, (uint8_t*)msg, strlen(msg));
			iset = 0;
		}
		
		//********************************* FOR CHECK 4_SEG_INDICATOR ******************************************************
		//	tm1637_WriteData(C5H, &a, 1);
		//data_disp_4seg(t1, t2, h1, h2, temp1, hum1);
		

		
		data_disp_6seg (t1, t2, h1, h2, temp1, hum1);
		
    if (button_released_flag)
    {
        button_released_flag = false; 
        
         hold_time = button_hold_duration;
         new_brightness = current_brightness;
        
       
        if (hold_time >= 5000) {
            new_brightness = MAX_BRIGHT;     
            blink_led(7);                    
        } 
        else if (hold_time >= 3000) {
            new_brightness = MIDDLE_BRIGHT;   
            blink_led(5);                     
        } 
        else if (hold_time >= 1000) {
            new_brightness = MIN_BRIGHT;      
            blink_led(3);                     
        }

							
				 if (new_brightness != current_brightness)
        {
            current_brightness = new_brightness;
            tm1637_SetBrightness(current_brightness);
            print_brightness_info();
        }
        
				
			}
			
		if (adc_mec_flag)
		{
			adc_voltage = adc_mes(); 
      adc_voltage_2 = adc_voltage + adc_voltage;
      int8_t *ptr = func_get_adc_mec(adc_voltage_2);
			func_voltage_on_displey(ptr);
		
			adc_mec_flag = false;
		
		}	                     
	
		HAL_Delay(1000);
		
		float adc_voltage = adc_mes(); 
	//	low_batt(adc_voltage);
		mes ++;
		sprintf (msg, "\n\rCOUNT: %d , Voltage = %1.2f\n\r\n\r", mes,adc_voltage);
		
		if (iset == 1){
		HAL_UART_Transmit_IT(&huart2, (uint8_t*)msg, strlen(msg));
		iset = 0;
		}
		if(mes > 65535)
				mes = 0;
		
	
		HAL_Delay(1000);

		
  }
}

	

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 15999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xFFFF;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

	HAL_TIM_Base_Start(&htim1);
	
}

static void MX_TIM2_Init(void)
{


  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
 
	HAL_TIM_Base_Start(&htim2);
  

}


static void MX_TIM3_Init(void)
{


  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xFFFF;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
 
	HAL_TIM_Base_Start(&htim3);
  

}



static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
 
}


static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1);
	
  /*Configure GPIO pin : PA5  1-WIRE */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7  LED*/ 
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	
	/*Configure GPIO pin : PB4 DATA_DISPLEY*/ 
	GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	/*Configure GPIO pin : PB5 CLK_DISPLEY*/ 
	GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
		  /*Configure GPIO pin : PB8  BUTTON*/
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode =  GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	
	/* EXTI interrupt init*/// NVIC_FOR_BUTTON GPIO pin: PB8
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	
	//===========================================================================
  GPIO_InitStruct.Pin = GPIO_PIN_4;    // MEAS_VOLT
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	//===========================================================================
	
}

/* USER CODE BEGIN 4 */

static void MX_ADC1_Init(void)
{
  __HAL_RCC_ADC1_CLK_ENABLE();
  ADC_ChannelConfTypeDef sConfig = {0};

	hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
	//hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;  
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
 
  HAL_ADCEx_Calibration_Start(&hadc1);
 
	
	/*
	sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
	
	*/
	
  

}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_8) 
    {
        
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == 0) 
        {
           
            if (!button_pressed_flag)  // Защита от дребезга
            {
                button_pressed_time = HAL_GetTick();
                button_pressed_flag = true;
                adc_mec_flag = true;  
                
            }
        }
        else 
        {
           
            if (button_pressed_flag) 
            {
                button_hold_duration = HAL_GetTick() - button_pressed_time;
                button_released_flag = true;
							
                button_pressed_flag = false; 
            }
        }
    }
}
