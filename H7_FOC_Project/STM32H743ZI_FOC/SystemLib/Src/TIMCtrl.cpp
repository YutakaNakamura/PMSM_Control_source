/*
 * TIMCtrl.cpp
 *
 *  Created on: 2019/09/22
 *      Author: watashi
 */

#include "TIMCtrl.hpp"
#include "paramsetting.h"

//staticの実体を作る
TIM_HandleTypeDef TIMCtrl::mHandleTIM1;

TIMCtrl::TIMCtrl() {
	// TODO Auto-generated constructor stub

}

TIMCtrl::~TIMCtrl() {
	// TODO Auto-generated destructor stub
}




//Initialize
void TIMCtrl::TIM1Init_HAL(void) {
	MX_TIM1_Init();
}

void TIMCtrl::TIM1DeInit_HAL(void) {
	MX_TIM1_DeInit();
}

//Start
void TIMCtrl::TIM1PWMStart(void) {
	HAL_TIM_PWM_Start(&mHandleTIM1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&mHandleTIM1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&mHandleTIM1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&mHandleTIM1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&mHandleTIM1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&mHandleTIM1, TIM_CHANNEL_3);

	HAL_TIM_PWM_Start(&mHandleTIM1, TIM_CHANNEL_4);
}

//SetCOMPARE
void TIMCtrl::TIM1SetCOMP_ch1(const int &pCompare) {
	TIM1 -> CCR1 = pCompare;
}
void TIMCtrl::TIM1SetCOMP_ch2(const int &pCompare) {
	TIM1 -> CCR2 = pCompare;
}
void TIMCtrl::TIM1SetCOMP_ch3(const int &pCompare) {
	TIM1 -> CCR3 = pCompare;
}
void TIMCtrl::TIM1SetCOMP_ch4(const int &pCompare) {
	TIM1 -> CCR4 = pCompare;
}
void TIMCtrl::TIM1SetCOMP_ch5(const int &pCompare) {
	TIM1 -> CCR5 = pCompare;
}
void TIMCtrl::TIM1SetCOMP_ch6(const int &pCompare) {
	TIM1 -> CCR6 = pCompare;
}

void TIMCtrl::floatDuty_ch1(const float &pfDuty) {
	int duty = pfDuty * (float)mHandleTIM1.Init.Period;
	TIM1SetCOMP_ch1(duty);
}

void TIMCtrl::floatDuty_ch2(const float &pfDuty) {
	int duty = pfDuty * (float)mHandleTIM1.Init.Period;
	TIM1SetCOMP_ch2(duty);
}

void TIMCtrl::floatDuty_ch3(const float &pfDuty) {
	int duty = pfDuty * (float)mHandleTIM1.Init.Period;
	TIM1SetCOMP_ch3(duty);
}

void TIMCtrl::floatDuty_ch4(const float &pfDuty) {
	int duty = pfDuty * (float)mHandleTIM1.Init.Period;
	TIM1SetCOMP_ch4(duty);
}

void TIMCtrl::MotorDuty_ch1(float pfDuty) {
	int duty = (pfDuty + 1)/2 * (float)mHandleTIM1.Init.Period;
	TIM1SetCOMP_ch1(duty);
}

void TIMCtrl::MotorDuty_ch2(float pfDuty) {
	int duty = (pfDuty + 1)/2 * (float)mHandleTIM1.Init.Period;
	TIM1SetCOMP_ch2(duty);
}

void TIMCtrl::MotorDuty_ch3(float pfDuty) {
	int duty = (pfDuty + 1)/2 * (float)mHandleTIM1.Init.Period;
	TIM1SetCOMP_ch3(duty);
}

void TIMCtrl::MotorDuty_ch4(float pfDuty) {
	int duty = (pfDuty + 1)/2 * (float)mHandleTIM1.Init.Period;
	TIM1SetCOMP_ch4(duty);
}


//以下HALコード置き場
void TIMCtrl::MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  mHandleTIM1.Instance = TIM1;
  mHandleTIM1.Init.Prescaler = 0;
  mHandleTIM1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  mHandleTIM1.Init.Period = PWM_PERIOD_COUNT;
  mHandleTIM1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  mHandleTIM1.Init.RepetitionCounter = 0;
  mHandleTIM1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&mHandleTIM1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&mHandleTIM1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&mHandleTIM1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&mHandleTIM1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_OC4REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&mHandleTIM1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&mHandleTIM1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&mHandleTIM1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&mHandleTIM1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = PWM_PERIOD_COUNT-1;
  if (HAL_TIM_PWM_ConfigChannel(&mHandleTIM1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = PWM_DEADTIME_COUNT;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&mHandleTIM1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&mHandleTIM1);

}

void TIMCtrl::MX_TIM1_DeInit(void) {
	if (HAL_TIM_PWM_DeInit(&mHandleTIM1) != HAL_OK) {
		Error_Handler();
	}
}


void TIMCtrl::HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspInit 0 */

  /* USER CODE END TIM1_MspInit 0 */
    /* TIM1 clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();

    /* TIM1 interrupt Init */
    HAL_NVIC_SetPriority(TIM1_BRK_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_BRK_IRQn);
    HAL_NVIC_SetPriority(TIM1_UP_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
    HAL_NVIC_SetPriority(TIM1_TRG_COM_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_TRG_COM_IRQn);
    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
  /* USER CODE BEGIN TIM1_MspInit 1 */

  /* USER CODE END TIM1_MspInit 1 */
  }
}
void TIMCtrl::HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

	  GPIO_InitTypeDef GPIO_InitStruct = {0};
	  if(timHandle->Instance==TIM1)
	  {
	  /* USER CODE BEGIN TIM1_MspPostInit 0 */

	  /* USER CODE END TIM1_MspPostInit 0 */

	    __HAL_RCC_GPIOE_CLK_ENABLE();
	    /**TIM1 GPIO Configuration
	    PE8     ------> TIM1_CH1N
	    PE9     ------> TIM1_CH1
	    PE10     ------> TIM1_CH2N
	    PE11     ------> TIM1_CH2
	    PE12     ------> TIM1_CH3N
	    PE13     ------> TIM1_CH3
	    PE14     ------> TIM1_CH4
	    */

	    //UH UL
	    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
	    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	    //VH VL
	    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
	    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	    //WH WL
	    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
	    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	    //timing out
	    GPIO_InitStruct.Pin = GPIO_PIN_14;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
	    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	  /* USER CODE BEGIN TIM1_MspPostInit 1 */

	  /* USER CODE END TIM1_MspPostInit 1 */
  }

}

void TIMCtrl::HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspDeInit 0 */

  /* USER CODE END TIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();

    /* TIM1 interrupt Deinit */
//    HAL_NVIC_DisableIRQ(TIM1_BRK_IRQn);
//    HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);
//    HAL_NVIC_DisableIRQ(TIM1_TRG_COM_IRQn);
//    HAL_NVIC_DisableIRQ(TIM1_CC_IRQn);

    HAL_NVIC_SetPriority(TIM1_BRK_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_BRK_IRQn);
    HAL_NVIC_SetPriority(TIM1_UP_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
    HAL_NVIC_SetPriority(TIM1_TRG_COM_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_TRG_COM_IRQn);
    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
  /* USER CODE BEGIN TIM1_MspDeInit 1 */

  /* USER CODE END TIM1_MspDeInit 1 */
  }
}
