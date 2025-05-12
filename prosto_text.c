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






























