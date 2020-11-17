/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"

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

/* USER CODE BEGIN PV */
uint16_t expe;
int blue_mode = 0;
int T_on = 0;
int T_off = 0;
int counter_systick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void RTC_wakeup_init(int delay);
void RTC_wakeup_init_from_standby_or_shutdown (int delay);
void Systick_config (int ticks);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

   if (expe==0){
	 LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
     LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
     LL_PWR_EnableBkUpAccess();

     NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
     SystemClock_Config();
     MX_GPIO_Init();
     //MX_RTC_Init();
     GPIO_init();
     //RTC_wakeup_init (100);
     Systick_config (SystemCoreClock/100);
     expe++;
  }


  /* System interrupt init*/

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */


  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */



  /* USER CODE BEGIN 2 */
  LL_RTC_DisableWriteProtection(RTC);
  LL_RTC_WriteReg(RTC, BKP0R, expe);
  LL_RTC_EnableWriteProtection(RTC);


  expe = LL_RTC_ReadReg(RTC,BKP0R);
  if	( BLUE_BUTTON() ){
  			LED_GREEN(1);
  			expe++;
  	}


  //expe = LL_RTC_ReadReg(RTC,BKP0R);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	if	( BLUE_BUTTON() ){
			LED_GREEN(1);
			blue_mode=1;
	} else {
			T_off = 10000-1000*expe;
			T_on = 1000*expe;
			LED_GREEN(0);
			LL_mDelay(T_off);
			LED_GREEN(1);
			LL_mDelay(T_on);
		 }
	if (blue_mode == 1)
	{
		LL_PWR_EnableLowPowerRunMode();
		LL_LPM_EnableDeepSleep();
	}

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  /*while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_4)
  {
  }*/
  //LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
 // LL_RCC_LSI_Enable();

   /* Wait till LSI is ready */
 /* while(LL_RCC_LSI_IsReady() != 1)
  {

  }*/
  LL_RCC_MSI_Enable();

   /* Wait till MSI is ready */
  while(LL_RCC_MSI_IsReady() != 1)
  {

  }
  /*LL_RCC_MSI_EnableRangeSelection();
  LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_6);
  LL_RCC_MSI_SetCalibTrimming(0);*/

  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_MSI, LL_RCC_PLLM_DIV_1, 40, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_Enable();
  LL_RCC_PLL_EnableDomain_SYS();
  while(LL_RCC_PLL_IsReady() != 1)
  	{ };

  /* Sysclk activation on the main PLL */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  	{ };

   /* Wait till System clock is ready */
 /* while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_MSI)
  {};*/

  //LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  /*LL_SYSTICK_IsActiveCounterFlag();


  LL_RCC_EnableIT_MSIRDY();
  while(!LL_RCC_IsEnabledIT_MSIRDY())
		{}
  LL_RCC_EnableIT_PLLRDY();


  LL_LPM_EnableEventOnPend();

  LL_Init1msTick(4000000);*/

  LL_SetSystemCoreClock(80000000);
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  LL_RTC_InitTypeDef RTC_InitStruct = {0};
  if(LL_RCC_LSE_IsReady()==0){
  /* Peripheral clock enable */
  LL_RCC_ForceBackupDomainReset();
  LL_RCC_ReleaseBackupDomainReset();
  LL_RCC_LSE_Enable();
  while(LL_RCC_LSE_IsReady()==0)
  {}
  LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSE);
  LL_RCC_EnableRTC();
  }
  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC and set the Time and Date
  */
  LL_RTC_DisableWriteProtection(RTC);
  LL_RTC_EnableInitMode(RTC);
  while(LL_RTC_IsActiveFlag_INIT(RTC)==0)
  {}

  RTC_InitStruct.HourFormat = LL_RTC_HOURFORMAT_24HOUR;
  RTC_InitStruct.AsynchPrescaler = 127;
  RTC_InitStruct.SynchPrescaler = 255;
  LL_RTC_Init(RTC, &RTC_InitStruct);
  LL_RTC_DisableInitMode(RTC);
  LL_RTC_EnableWriteProtection(RTC);
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

static void RTC_wakeup_init (int delay)
{
	LL_RTC_DisableWriteProtection(RTC);
	LL_RTC_WAKEUP_Disable(RTC);
	while (!LL_RTC_IsActiveFlag_WUTW(RTC))
	{}

	LL_RTC_WAKEUP_SetClock(RTC, LL_RTC_WAKEUPCLOCK_CKSPRE);
	LL_RTC_WAKEUP_SetAutoReload(RTC, delay);
	LL_RTC_ClearFlag_WUT(RTC);
	LL_RTC_EnableIT_WUT(RTC);
	LL_RTC_WAKEUP_Enable(RTC);
	//expe = LL_RTC_ReadReg(RTC,BKP0R);
	LL_RTC_EnableWriteProtection(RTC);
}

void RTC_wakeup_init_from_standby_or_shutdown (int delay)
{
	RTC_wakeup_init(delay);
	LL_PWR_EnableInternWU();
}

void RTC_wakeup_init_from_stop (int delay)
{
	RTC_wakeup_init(delay);
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_20);
	LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_20);
	NVIC_SetPriority(RTC_WKUP_IRQn,1);
	NVIC_EnableIRQ(RTC_WKUP_IRQn);
}
/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_10);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}
void Systick_config (int ticks)
{
	SysTick->VAL = 0;
    SysTick->LOAD =ticks-1;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk;
    NVIC_SetPriority( SysTick_IRQn, 7 );
    LL_SYSTICK_EnableIT();
    while(!LL_SYSTICK_IsEnabledIT())
      {}
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk;
}
void Systick_Handler (void)
{
	counter_systick = 0;
	LED_GREEN(0);
	LED_GREEN(1);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
