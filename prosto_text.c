// https://naviawireless.ru/wp-content/uploads/pn6280_hardware-design_v1.0.pdf?ysclid=ma6sja3ogx383492268
// https://www.euromobile.ru/upload/iblock/2d0/pn6280_at_command_user_guide_v1.0.pdf?ysclid=ma6ttj1x3m803827390


MAIN 

#include "hw_init.h"    
#include "hw_config.h"

typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

int main()
{
	//uint8_t count = 2;	
	uint8_t data_tx_spi[] = {1, 2, 3, 4, 5};
	uint8_t data_rx_spi[BufferSize];

	DMA_Init_();
	SPI_MASTER_Init();
	SPI_SLAVE_Init();
	SystemInit();
	GPIO_LED_init();
	GPIO_Config();




 
    DMA_Cmd(SPI_SLAVE_Rx_DMA_Channel, ENABLE); 
    SPI_I2S_DMACmd(SPI_SLAVE, SPI_I2S_DMAReq_Rx, ENABLE); 

    while (1)
    {
       
       for (uint8_t i = 0; i < BufferSize; i++)
        {
            while (SPI_I2S_GetFlagStatus(SPI_MASTER, SPI_I2S_FLAG_TXE) == RESET);
             SPI_I2S_SendData(SPI_MASTER, data_tx_spi[i]);
        }

       
       while (!DMA_GetFlagStatus(DMA1_FLAG_TC1)); 

        for (uint8_t i = 0; i < BufferSize; i++)
        {
           if(data_rx_spi[i] != data_tx_spi[i])
						blink_led();
					 	blink_led();
				}

       
        DMA_ClearFlag(DMA1_FLAG_TC1);
        
        delay_ms(500); 
    }
}


//-----------------------------------------------------------------------------------------------------------------------------



#include "hw_init.h"             // Device config   
#include "hw_config.h"

                  
	
	
 const uint8_t BufferSize = 5;
 uint8_t data_tx_spi[BufferSize];
 uint8_t data_rx_spi[BufferSize];
   
/* ------------------------------------------------- SPI_MASTER configuration -----------------------------------------------------------------*/

void SPI_MASTER_Init(void)
{
	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	SPI_InitTypeDef SPI_InitStructure;	
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
	
  SPI_Init(SPI_MASTER, &SPI_InitStructure);
	SPI_Cmd(SPI_MASTER, ENABLE);
	
}

/* --------------------------------------------------- SPI_SLAVE configuration -----------------------------------------------------------------*/

void SPI_SLAVE_Init(void)
{
	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	SPI_InitTypeDef SPI_InitStructure;	
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Rx;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
	
  SPI_Init(SPI_SLAVE, &SPI_InitStructure);
	SPI_Cmd(SPI_SLAVE, ENABLE);
	
}


//---------------------------------------------------GPIO_Config_from_SPI------------------------------------------------------------------------

void GPIO_Config(void) // моя
{
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
		
	GPIO_InitTypeDef GPIOA_Struct_Init;
	
	// Configure SPI_MASTER 
	
	GPIOA_Struct_Init.GPIO_Pin = SPI_MASTER_PIN_SCK | SPI_MASTER_PIN_MOSI;
  GPIOA_Struct_Init.GPIO_Speed = GPIO_Speed_50MHz;
  GPIOA_Struct_Init.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(SPI_MASTER_GPIO, &GPIOA_Struct_Init);

 
  GPIOA_Struct_Init.GPIO_Pin = SPI_MASTER_PIN_NSS;
  GPIOA_Struct_Init.GPIO_Speed = GPIO_Speed_50MHz;
  GPIOA_Struct_Init.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(SPI_MASTER_GPIO_NSS, &GPIOA_Struct_Init);
	
	// Configure SPI_SLAVE 
	
	GPIOA_Struct_Init.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIOA_Struct_Init.GPIO_Pin = SPI_SLAVE_PIN_SCK | SPI_SLAVE_PIN_MISO;
	GPIO_Init(SPI_SLAVE_GPIO, &GPIOA_Struct_Init);
	
	GPIOA_Struct_Init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIOA_Struct_Init.GPIO_Pin = SPI_SLAVE_PIN_NSS;
	GPIO_Init(SPI_SLAVE_GPIO, &GPIOA_Struct_Init);
	

}	

//---------------------------------------------------DMA_Config------------------------------------------------------------------------

void DMA_Init_(void)
{
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	DMA_InitTypeDef DMA_InitStructure;
	
	DMA_DeInit(SPI_SLAVE_Rx_DMA_Channel);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)SPI_SLAVE_DR_Base;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint8_t)data_rx_spi;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = BufferSize;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	
  DMA_Init(SPI_SLAVE_Rx_DMA_Channel, &DMA_InitStructure);
	//DMA_Cmd(SPI_SLAVE_Rx_DMA_Channel, ENABLE);

}

//------------------------------------------------_LED_Transmit_Indication------------------------------------------------------------------------

void GPIO_LED_init(void)
{
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef PORTA;
	
	PORTA.GPIO_Speed = GPIO_Speed_2MHz;
	PORTA.GPIO_Mode = GPIO_Mode_Out_PP;
	PORTA.GPIO_Pin = GPIO_Pin_2;
	
	GPIO_Init(GPIOA, &PORTA); 	
	
}

//--------------------------------------------------BLINK_RED_LAD----------------------------------------------------------------------------------

void blink_led(void)
{
		
	GPIO_SetBits(GPIOA, GPIO_Pin_2);
	delay_ms(50);
	GPIO_ResetBits(GPIOA, GPIO_Pin_2);
	delay_ms(50);
	GPIO_SetBits(GPIOA, GPIO_Pin_2);
	delay_ms(50);
	GPIO_ResetBits(GPIOA, GPIO_Pin_2);
	delay_ms(50);
		
}

//---------------------------------------------------DELAY_MS-------------------------------------------------------------------------------------

    
void delay_ms(uint32_t ms)
{
    
  SysTick->LOAD = (SystemCoreClock / 1000) - 1; // Загрузка для 1 мс
  SysTick->VAL = 0; 
  SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; 

  for (uint32_t i = 0; i < ms; i++) {
      while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)); 
  }

  SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk; // Останавливаем SysTick
}







//************************************************************************************************************************************************************************************************

#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	const uint8_t buffer_size = 5;
	uint8_t data_tx_spi[buffer_size] = {1,2,3,4,5};
	uint8_t data_rx_spi[buffer_size];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
	HAL_Delay(200);

HAL_SPI_TransmitReceive_IT(&hspi1, data_tx_spi, data_rx_spi, buffer_size);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	HAL_Delay(200);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}




/**
  * @brief System Clock Configuration
  * @retval None
  */

void blink_led(void)
{
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);
	HAL_Delay(200);
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);
}


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
		{
      HAL_SPI_Transmit_IT(&hspi2, data_rx_spi, buffer_size);
    
		} else if (hspi->Instance == SPI2)
		{
      for (int i = 0; i < buffer_size; i++)
			{
        if (data_rx_spi[i] != data_tx_spi[i])
				   blink_led();
          
       }
    }
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}






















