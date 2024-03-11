/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c-lcd.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define NUMBER_PASSWORD 6
#define HIGH 1
#define LOW 0
#define COUNTER_DELAY 200
#define FLASH_ADDRESS_PAGE31 0x08007C00
#define FLASH_ADDRESS_PAGE127	0x0801FC00
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
char check;
static uint8_t i = 0;
static char PASSWORD[NUMBER_PASSWORD] = {'1', '2', '3', '4', '5', '6'};
static char CURR_PASSWORD[NUMBER_PASSWORD] = {0, 0, 0, 0, 0, 0};
uint8_t TEST[NUMBER_PASSWORD] = {1,2,3,4,5,6};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


char CHECK_COL()
{
	// =============== ROW 1 ==============
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, HIGH);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, LOW);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, LOW);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, LOW);
	//HAL_Delay(100);
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15))
	{
		while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15));
		return 'A';
	}
	
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14))
	{
		while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14));
		return '3';
	}
	
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13))
	{
		while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13));
		return '2';
	}
	
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12))
	{
		while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12));
		return '1';
	}
	// =============== ROW 2 ==============
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, LOW);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, HIGH);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, LOW);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, LOW);
	//HAL_Delay(100);
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15))
	{
		while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15));
		return 'B';
	}
	
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14))
	{
		while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14));
		return '6';
	}
	
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13))
	{
		while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13));
		return '5';
	}
	
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12))
	{
		while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12));
		return '4';
	}
	// =============== ROW 3 ==============
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, LOW);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, LOW);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, HIGH);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, LOW);
	//HAL_Delay(100);
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15))
	{
		while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15));
		return 'C';
	}
	
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14))
	{
		while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14));
		return '9';
	}
	
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13))
	{
		while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13));
		return '8';
	}
	
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12))
	{
		while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12));
		return '7';
	}
	// =============== ROW 4 ==============
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, LOW);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, LOW);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, LOW);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, HIGH);
	//HAL_Delay(100);
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15))
	{
		while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15));
		return 'D';
	}
	
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14))
	{
		while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14));
		return '#';
	}
	
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13))
	{
		while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13));
		return '0';
	}
	
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12))
	{
		while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12));
		return '*';
	}
	return 0;
}

void FLASH_ERASE(uint32_t Address)
{
	HAL_FLASH_Unlock();
	FLASH_EraseInitTypeDef EraseInit;
	EraseInit.TypeErase	= FLASH_TYPEERASE_PAGES;
	EraseInit.PageAddress = Address;
	EraseInit.NbPages = 1;
	EraseInit.Banks = 1;
	uint32_t PageError = 0;
	HAL_FLASHEx_Erase(&EraseInit, &PageError);
	HAL_FLASH_Lock();
}
uint8_t FLASH_WRITE_DATA(uint32_t Address, char *arr, uint16_t len)
{
	HAL_FLASH_Unlock();
	uint8_t *pt = (uint8_t*)arr;
	for(uint8_t n = 0; n < len ; n++){
		if((HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, Address + 2*n, *pt++)) != HAL_OK)
		{
			return HAL_ERROR;
		}
	}
	HAL_FLASH_Lock();
	return 0;
}

uint32_t FLASH_READ_DATA(uint32_t address){
    return *(uint8_t*)address;
}


void DEL(Lcd_HandleTypeDef lcd)	
{
	if(i > 0) 
	{
		i--;
		CURR_PASSWORD[i] = '\0';
		Lcd_cursor(&lcd, 1,i);
		Lcd_string(&lcd, " ");
	}
	else if(i == 0)
	{
		i = 0;
		CURR_PASSWORD[i] = '\0';
		Lcd_cursor(&lcd, 1,i);
		Lcd_string(&lcd, " ");
	}
}

//Prescaler: F clock/ F wanna generate -> / counter period + 1
void SG90()
{
	for(int t=250 ;t<1250;t+=50)
	{
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,t);
		HAL_Delay(3);
	}
		HAL_Delay(1000);
		for(int u=1250 ;u>250;u-=50)
		{
			__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,u);	
			HAL_Delay(3);
		}
}

uint8_t CHECK_PASSWORD(char CURRENT[])
{
	char TEMP[6];
	uint32_t CHECK;
	i = 0;
	
	CHECK = FLASH_READ_DATA(FLASH_ADDRESS_PAGE31);
	for(uint8_t t = 0; t < NUMBER_PASSWORD; t++)
	{
		if(CHECK == 0xFF)
		{
			TEMP[t] = PASSWORD[t];
		}
		else
		{
			TEMP[t] = FLASH_READ_DATA(FLASH_ADDRESS_PAGE31 + 2*t);	
		}
	}
	
	uint8_t flag = 0;
	for(uint8_t temp = 0; temp < NUMBER_PASSWORD; temp++)
	{
		if(CURRENT[temp] == TEMP[temp])
		{
			flag++;
		}
	}
	if(flag == NUMBER_PASSWORD)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

void ENTER_PASSWORD(Lcd_HandleTypeDef lcd)
{
	
	uint8_t flag = CHECK_PASSWORD(CURR_PASSWORD);
	if(flag == 1)
	{
		Lcd_clear(&lcd);
		Lcd_cursor(&lcd, 0,0);
		Lcd_string(&lcd, "THE DOOR OPENED");
		HAL_Delay(1000);
		i = 0;
		SG90();
		//Delete password
		for(uint8_t o=0 ;o<6;o++)
		{
			CURR_PASSWORD[o] = 0;
			PASSWORD[o] = '0';
		}
	}
	else
	{
		Lcd_clear(&lcd);
		Lcd_cursor(&lcd, 0,0);
		Lcd_string(&lcd, "WRONG PASSWORD");
		i = 0;
		HAL_Delay(1000);
	}
}

void CHANGE_PASSWORD(Lcd_HandleTypeDef lcd)
{
	char TEMP_PASSWORD[NUMBER_PASSWORD];
	uint8_t flag = 0;
	uint8_t temp;
	i = 0;
	Lcd_clear(&lcd);
	do
	{
		Lcd_cursor(&lcd, 0,0);
		Lcd_string(&lcd, "Current password:");
		check = CHECK_COL();
		if(check == 'A') DEL(lcd);
		if(check == 'B'){	Lcd_clear(&lcd);	break;}
		if(i < 6 && check != 0 && check != 'B' && check != 'A' && check != 'C')
		{
			Lcd_cursor(&lcd, 1,i);
			Lcd_string(&lcd, &check);
			TEMP_PASSWORD[i] = check;
			i++;
		}
		if(check == 'D')
		{
			temp = CHECK_PASSWORD(TEMP_PASSWORD);
			if(temp == 1)
			{
				i = 0;
				Lcd_clear(&lcd);
				do
				{
					Lcd_cursor(&lcd, 0,0);
					Lcd_string(&lcd, "New password:");
					check = CHECK_COL();
					if(check == 'A') DEL(lcd);
					if(check == 'B'){	Lcd_clear(&lcd);	break;}
					if(i < NUMBER_PASSWORD && check != 0 && check != 'B' && check != 'A' && check != 'C' && check != 'D')
					{
						Lcd_cursor(&lcd, 1,i);
						Lcd_string(&lcd, &check);
						TEMP_PASSWORD[i] = check;
						i++;
					}
					if(check == 'D' && i == NUMBER_PASSWORD)
					{
						HAL_FLASH_Unlock();
						for(uint8_t j = 0; j < NUMBER_PASSWORD; j++)
						{
							PASSWORD[j] = TEMP_PASSWORD[j];
						}
						FLASH_ERASE(FLASH_ADDRESS_PAGE31);
						FLASH_WRITE_DATA(FLASH_ADDRESS_PAGE31, PASSWORD, NUMBER_PASSWORD);
						flag = 1;
						HAL_FLASH_Lock();
						Lcd_clear(&lcd);
					}
				}while(!flag);
				
			}
			else
			{
				Lcd_clear(&lcd);
				Lcd_cursor(&lcd, 0,0);
				Lcd_string(&lcd, "WRONG PASSWORD");
				i = 0;
				HAL_Delay(800);
			}
		}

//		if(check == 'D')	flag = 2;
	}while(!flag);
}
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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	
  Lcd_PortType ports[] = { GPIOB, GPIOB, GPIOB, GPIOB };
  // Lcd_PinType pins[] = {D4_Pin, D5_Pin, D6_Pin, D7_Pin};
  Lcd_PinType pins[] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_10, GPIO_PIN_11};
  Lcd_HandleTypeDef lcd;
  // Lcd_create(ports, pins, RS_GPIO_Port, RS_Pin, EN_GPIO_Port, EN_Pin, LCD_4_BIT_MODE);
  lcd = Lcd_create(ports, pins, GPIOA, GPIO_PIN_1, GPIOA, GPIO_PIN_3, LCD_4_BIT_MODE);
//	uint8_t CHECK;
//	FLASH_ERASE(FLASH_ADDRESS_PAGE31);
//	FLASH_WRITE_DATA(FLASH_ADDRESS_PAGE31,PASSWORD,NUMBER_PASSWORD);
//	CHECK = FLASH_READ_DATA(FLASH_ADDRESS_PAGE31);
	
	Lcd_cursor(&lcd, 0,1);
	Lcd_string(&lcd, "Welcome Home!!");
	HAL_Delay(1000);
	Lcd_clear(&lcd);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		Lcd_cursor(&lcd, 0,0);
		Lcd_string(&lcd, "Enter password:");
		

		check = CHECK_COL();
		if(check != 0)
		{
			HAL_Delay(50);
			if(check != 0)
			{
					//Delete
					if(check == 'A')
					{
						DEL(lcd);
					}
					
					//Change password
					else if(check == 'C')
					{
						
						CHANGE_PASSWORD(lcd);
						i = 0;
					}
					
					//Enter password
					else if(check == 'D')
					{
						ENTER_PASSWORD(lcd);
					}
					
					else
					{
						if(i <= 5 && check != 'B')
						{
							Lcd_cursor(&lcd, 1,i);
							CURR_PASSWORD[i] = check;
							Lcd_string(&lcd, &check);
							
							HAL_Delay(100);
							Lcd_cursor(&lcd, 1,i);
							Lcd_string(&lcd, "*");
							i++;
						}
					}
			}
		}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 144;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA3 PA8 PA9
                           PA10 PA11 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB10 PB11
                           PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
