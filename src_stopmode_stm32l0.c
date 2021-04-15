/******************************************************

@author: Sampath Kumar Krishna Murthy

MCU: STM32L051C6

  - This code is written to bring stm32l051c6 microcontroller into stop mode.
  - Here the stop mode is considered to be efficient for low-power consumption.
  
  
  /* Here the MCU enters into stop mode*/
  
void Enter_wakeup_Mode(unit16_t GPIO_pin)
{
    if(GPIO_Pin == GPIO_PIN_13)
    {
      SystemClock_Config();
      _HAL_ResumeTick();
}

void Enter_Stop_Mode(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

 /* Configure all GPIO port pins in Analog Input mode (floating input trigger OFF) */
 GPIO_InitStruct.Pin = GPIO_PIN_All;
 GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
 GPIO_InitStruct.Pull = GPIO_NOPULL;
 HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); 
 HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

 /* Disable GPIOs clock */
 __HAL_RCC_GPIOA_CLK_DISABLE();
 __HAL_RCC_GPIOB_CLK_DISABLE();

 HAL_TIM_Base_Stop_IT(&htim22);
 HAL_ADC_Stop_IT(&hadc);

 /* Enter Stop Mode */

 HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
 
 HAL_SuspendTick();

 /* init the SYS Clocks */
 SystemClock_Config();

 /*Init IO's */
 MX_GPIO_Init();

 /* Clear Peripheral States for MspInit *7
 huart1.gState = HAL_UART_STATE_RESET;
 huart2.gState = HAL_UART_STATE_RESET;
 hi2c1.State   = HAL_I2C_STATE_RESET;
 hadc.State    = HAL_ADC_STATE_RESET;

 MX_USART2_UART_Init();  
 MX_USART1_UART_Init();    
 MX_I2C1_Init();
 MX_RTC_Init();
 MX_TIM22_Init();
 MX_ADC_Init();
 HAL_ADCEx_Calibration_Start(&hadc,ADC_SINGLE_ENDED);   
 }
