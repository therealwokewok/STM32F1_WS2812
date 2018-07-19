
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
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
#include "main.h"
#include "stm32f1xx_hal.h"

/* Number of LED's connected.  Remember, 0.3 watts per LED.  Don't blow things up -_- */
#define LED_NO    7

/* Private handlers */
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);

/* The big array of hex values mapping all values for SPI transfer */
const uint8_t leddata[256*3] = { // size = 256 * 3
0X92 , 0X49 , 0X24 , // 0
0X92 , 0X49 , 0X26 , // 1
0X92 , 0X49 , 0X34 , // 2
0X92 , 0X49 , 0X36 ,
0X92 , 0X49 , 0XA4 ,
0X92 , 0X49 , 0XA6 ,
0X92 , 0X49 , 0XB4 ,
0X92 , 0X49 , 0XB6 ,
0X92 , 0X4D , 0X24 ,
0X92 , 0X4D , 0X26 ,
0X92 , 0X4D , 0X34 ,
0X92 , 0X4D , 0X36 ,
0X92 , 0X4D , 0XA4 ,
0X92 , 0X4D , 0XA6 ,
0X92 , 0X4D , 0XB4 ,
0X92 , 0X4D , 0XB6 ,
0X92 , 0X69 , 0X24 ,
0X92 , 0X69 , 0X26 ,
0X92 , 0X69 , 0X34 ,
0X92 , 0X69 , 0X36 ,
0X92 , 0X69 , 0XA4 ,
0X92 , 0X69 , 0XA6 ,
0X92 , 0X69 , 0XB4 ,
0X92 , 0X69 , 0XB6 ,
0X92 , 0X6D , 0X24 ,
0X92 , 0X6D , 0X26 ,
0X92 , 0X6D , 0X34 ,
0X92 , 0X6D , 0X36 ,
0X92 , 0X6D , 0XA4 ,
0X92 , 0X6D , 0XA6 ,
0X92 , 0X6D , 0XB4 ,
0X92 , 0X6D , 0XB6 ,
0X93 , 0X49 , 0X24 ,
0X93 , 0X49 , 0X26 ,
0X93 , 0X49 , 0X34 ,
0X93 , 0X49 , 0X36 ,
0X93 , 0X49 , 0XA4 ,
0X93 , 0X49 , 0XA6 ,
0X93 , 0X49 , 0XB4 ,
0X93 , 0X49 , 0XB6 ,
0X93 , 0X4D , 0X24 ,
0X93 , 0X4D , 0X26 ,
0X93 , 0X4D , 0X34 ,
0X93 , 0X4D , 0X36 ,
0X93 , 0X4D , 0XA4 ,
0X93 , 0X4D , 0XA6 ,
0X93 , 0X4D , 0XB4 ,
0X93 , 0X4D , 0XB6 ,
0X93 , 0X69 , 0X24 ,
0X93 , 0X69 , 0X26 ,
0X93 , 0X69 , 0X34 ,
0X93 , 0X69 , 0X36 ,
0X93 , 0X69 , 0XA4 ,
0X93 , 0X69 , 0XA6 ,
0X93 , 0X69 , 0XB4 ,
0X93 , 0X69 , 0XB6 ,
0X93 , 0X6D , 0X24 ,
0X93 , 0X6D , 0X26 ,
0X93 , 0X6D , 0X34 ,
0X93 , 0X6D , 0X36 ,
0X93 , 0X6D , 0XA4 ,
0X93 , 0X6D , 0XA6 ,
0X93 , 0X6D , 0XB4 ,
0X93 , 0X6D , 0XB6 ,
0X9A , 0X49 , 0X24 ,
0X9A , 0X49 , 0X26 ,
0X9A , 0X49 , 0X34 ,
0X9A , 0X49 , 0X36 ,
0X9A , 0X49 , 0XA4 ,
0X9A , 0X49 , 0XA6 ,
0X9A , 0X49 , 0XB4 ,
0X9A , 0X49 , 0XB6 ,
0X9A , 0X4D , 0X24 ,
0X9A , 0X4D , 0X26 ,
0X9A , 0X4D , 0X34 ,
0X9A , 0X4D , 0X36 ,
0X9A , 0X4D , 0XA4 ,
0X9A , 0X4D , 0XA6 ,
0X9A , 0X4D , 0XB4 ,
0X9A , 0X4D , 0XB6 ,
0X9A , 0X69 , 0X24 ,
0X9A , 0X69 , 0X26 ,
0X9A , 0X69 , 0X34 ,
0X9A , 0X69 , 0X36 ,
0X9A , 0X69 , 0XA4 ,
0X9A , 0X69 , 0XA6 ,
0X9A , 0X69 , 0XB4 ,
0X9A , 0X69 , 0XB6 ,
0X9A , 0X6D , 0X24 ,
0X9A , 0X6D , 0X26 ,
0X9A , 0X6D , 0X34 ,
0X9A , 0X6D , 0X36 ,
0X9A , 0X6D , 0XA4 ,
0X9A , 0X6D , 0XA6 ,
0X9A , 0X6D , 0XB4 ,
0X9A , 0X6D , 0XB6 ,
0X9B , 0X49 , 0X24 ,
0X9B , 0X49 , 0X26 ,
0X9B , 0X49 , 0X34 ,
0X9B , 0X49 , 0X36 ,
0X9B , 0X49 , 0XA4 ,
0X9B , 0X49 , 0XA6 ,
0X9B , 0X49 , 0XB4 ,
0X9B , 0X49 , 0XB6 ,
0X9B , 0X4D , 0X24 ,
0X9B , 0X4D , 0X26 ,
0X9B , 0X4D , 0X34 ,
0X9B , 0X4D , 0X36 ,
0X9B , 0X4D , 0XA4 ,
0X9B , 0X4D , 0XA6 ,
0X9B , 0X4D , 0XB4 ,
0X9B , 0X4D , 0XB6 ,
0X9B , 0X69 , 0X24 ,
0X9B , 0X69 , 0X26 ,
0X9B , 0X69 , 0X34 ,
0X9B , 0X69 , 0X36 ,
0X9B , 0X69 , 0XA4 ,
0X9B , 0X69 , 0XA6 ,
0X9B , 0X69 , 0XB4 ,
0X9B , 0X69 , 0XB6 ,
0X9B , 0X6D , 0X24 ,
0X9B , 0X6D , 0X26 ,
0X9B , 0X6D , 0X34 ,
0X9B , 0X6D , 0X36 ,
0X9B , 0X6D , 0XA4 ,
0X9B , 0X6D , 0XA6 ,
0X9B , 0X6D , 0XB4 ,
0X9B , 0X6D , 0XB6 ,
0XD2 , 0X49 , 0X24 ,
0XD2 , 0X49 , 0X26 ,
0XD2 , 0X49 , 0X34 ,
0XD2 , 0X49 , 0X36 ,
0XD2 , 0X49 , 0XA4 ,
0XD2 , 0X49 , 0XA6 ,
0XD2 , 0X49 , 0XB4 ,
0XD2 , 0X49 , 0XB6 ,
0XD2 , 0X4D , 0X24 ,
0XD2 , 0X4D , 0X26 ,
0XD2 , 0X4D , 0X34 ,
0XD2 , 0X4D , 0X36 ,
0XD2 , 0X4D , 0XA4 ,
0XD2 , 0X4D , 0XA6 ,
0XD2 , 0X4D , 0XB4 ,
0XD2 , 0X4D , 0XB6 ,
0XD2 , 0X69 , 0X24 ,
0XD2 , 0X69 , 0X26 ,
0XD2 , 0X69 , 0X34 ,
0XD2 , 0X69 , 0X36 ,
0XD2 , 0X69 , 0XA4 ,
0XD2 , 0X69 , 0XA6 ,
0XD2 , 0X69 , 0XB4 ,
0XD2 , 0X69 , 0XB6 ,
0XD2 , 0X6D , 0X24 ,
0XD2 , 0X6D , 0X26 ,
0XD2 , 0X6D , 0X34 ,
0XD2 , 0X6D , 0X36 ,
0XD2 , 0X6D , 0XA4 ,
0XD2 , 0X6D , 0XA6 ,
0XD2 , 0X6D , 0XB4 ,
0XD2 , 0X6D , 0XB6 ,
0XD3 , 0X49 , 0X24 ,
0XD3 , 0X49 , 0X26 ,
0XD3 , 0X49 , 0X34 ,
0XD3 , 0X49 , 0X36 ,
0XD3 , 0X49 , 0XA4 ,
0XD3 , 0X49 , 0XA6 ,
0XD3 , 0X49 , 0XB4 ,
0XD3 , 0X49 , 0XB6 ,
0XD3 , 0X4D , 0X24 ,
0XD3 , 0X4D , 0X26 ,
0XD3 , 0X4D , 0X34 ,
0XD3 , 0X4D , 0X36 ,
0XD3 , 0X4D , 0XA4 ,
0XD3 , 0X4D , 0XA6 ,
0XD3 , 0X4D , 0XB4 ,
0XD3 , 0X4D , 0XB6 ,
0XD3 , 0X69 , 0X24 ,
0XD3 , 0X69 , 0X26 ,
0XD3 , 0X69 , 0X34 ,
0XD3 , 0X69 , 0X36 ,
0XD3 , 0X69 , 0XA4 ,
0XD3 , 0X69 , 0XA6 ,
0XD3 , 0X69 , 0XB4 ,
0XD3 , 0X69 , 0XB6 ,
0XD3 , 0X6D , 0X24 ,
0XD3 , 0X6D , 0X26 ,
0XD3 , 0X6D , 0X34 ,
0XD3 , 0X6D , 0X36 ,
0XD3 , 0X6D , 0XA4 ,
0XD3 , 0X6D , 0XA6 ,
0XD3 , 0X6D , 0XB4 ,
0XD3 , 0X6D , 0XB6 ,
0XDA , 0X49 , 0X24 ,
0XDA , 0X49 , 0X26 ,
0XDA , 0X49 , 0X34 ,
0XDA , 0X49 , 0X36 ,
0XDA , 0X49 , 0XA4 ,
0XDA , 0X49 , 0XA6 ,
0XDA , 0X49 , 0XB4 ,
0XDA , 0X49 , 0XB6 ,
0XDA , 0X4D , 0X24 ,
0XDA , 0X4D , 0X26 ,
0XDA , 0X4D , 0X34 ,
0XDA , 0X4D , 0X36 ,
0XDA , 0X4D , 0XA4 ,
0XDA , 0X4D , 0XA6 ,
0XDA , 0X4D , 0XB4 ,
0XDA , 0X4D , 0XB6 ,
0XDA , 0X69 , 0X24 ,
0XDA , 0X69 , 0X26 ,
0XDA , 0X69 , 0X34 ,
0XDA , 0X69 , 0X36 ,
0XDA , 0X69 , 0XA4 ,
0XDA , 0X69 , 0XA6 ,
0XDA , 0X69 , 0XB4 ,
0XDA , 0X69 , 0XB6 ,
0XDA , 0X6D , 0X24 ,
0XDA , 0X6D , 0X26 ,
0XDA , 0X6D , 0X34 ,
0XDA , 0X6D , 0X36 ,
0XDA , 0X6D , 0XA4 ,
0XDA , 0X6D , 0XA6 ,
0XDA , 0X6D , 0XB4 ,
0XDA , 0X6D , 0XB6 ,
0XDB , 0X49 , 0X24 ,
0XDB , 0X49 , 0X26 ,
0XDB , 0X49 , 0X34 ,
0XDB , 0X49 , 0X36 ,
0XDB , 0X49 , 0XA4 ,
0XDB , 0X49 , 0XA6 ,
0XDB , 0X49 , 0XB4 ,
0XDB , 0X49 , 0XB6 ,
0XDB , 0X4D , 0X24 ,
0XDB , 0X4D , 0X26 ,
0XDB , 0X4D , 0X34 ,
0XDB , 0X4D , 0X36 ,
0XDB , 0X4D , 0XA4 ,
0XDB , 0X4D , 0XA6 ,
0XDB , 0X4D , 0XB4 ,
0XDB , 0X4D , 0XB6 ,
0XDB , 0X69 , 0X24 ,
0XDB , 0X69 , 0X26 ,
0XDB , 0X69 , 0X34 ,
0XDB , 0X69 , 0X36 ,
0XDB , 0X69 , 0XA4 ,
0XDB , 0X69 , 0XA6 ,
0XDB , 0X69 , 0XB4 ,
0XDB , 0X69 , 0XB6 ,
0XDB , 0X6D , 0X24 ,
0XDB , 0X6D , 0X26 ,
0XDB , 0X6D , 0X34 ,
0XDB , 0X6D , 0X36 ,
0XDB , 0X6D , 0XA4 ,
0XDB , 0X6D , 0XA6 ,
0XDB , 0X6D , 0XB4 , // 0xFE
0XDB , 0X6D , 0XB6 , // 0xFF

};

/* Buffer for each transfer */
uint8_t ws_buffer[9 * LED_NO];

void encode_byte( uint8_t data, int16_t buffer_index )
{
   int index = data * 3;
   ws_buffer[buffer_index++ ] = leddata[index++];
   ws_buffer[buffer_index++ ] = leddata[index++];
   ws_buffer[buffer_index++ ] = leddata[index++];
}
void generate_ws_buffer( uint8_t RData,uint8_t GData,uint8_t BData, int16_t led_no )
{
/* GRB - MSB First */
   int offset = led_no * 9;
   encode_byte( GData, offset );
   encode_byte( RData, offset+3 );
   encode_byte( BData, offset+6 );
}
void Send_2812(void)
 {
#if 1
    HAL_SPI_Transmit_DMA( &hspi1, ws_buffer, 9 * LED_NO );
    // wait until finished
    while(__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_BSY ));
#else
    HAL_SPI_Transmit( &hspi1, ws_buffer, 9 * LED_NO, 300 );
#endif
 }

void setAllPixelColor(uint8_t r, uint8_t g, uint8_t b)
{
   int i;
   for(i=0;i< LED_NO;i++) {
      generate_ws_buffer( r, g, b, i );
   }
   Send_2812();
}
 void setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b)
 {
   generate_ws_buffer( r, g, b, n );
   Send_2812();
}

int main(void)
{
  /* System Setup */
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();

  /* Main Loop */
  while (1)
  {
	  int8_t i;
	  setAllPixelColor( 0, 0, 0);
	  HAL_Delay(50);
	  // red
	  for ( i = 0; i < LED_NO; i++) {
		 setPixelColor( i, 250, 0, 0 );
		 HAL_Delay(10);
	  }
	  // green
	  for ( i = 0; i < LED_NO; i++) {
		 setPixelColor( i, 0, 250, 0 );
		 HAL_Delay(10);
	  }
	  // blue
	  for ( i = 0; i < LED_NO; i++) {
		 setPixelColor( i, 0, 0, 250 );
		 HAL_Delay(10);
	  }
  }
}

/* CubeMX Stuff.  Don't touch it -_- */
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(UserLED_GPIO_Port, UserLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : UserLED_Pin */
  GPIO_InitStruct.Pin = UserLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(UserLED_GPIO_Port, &GPIO_InitStruct);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
