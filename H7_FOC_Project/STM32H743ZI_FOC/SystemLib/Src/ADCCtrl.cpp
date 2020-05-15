/*
 * ADCCtrl.cpp
 *
 *  Created on: Sep 21, 2019
 *      Author: watashi
 */

#include "ADCCtrl.hpp"

//staticの実体を作る
ADC_HandleTypeDef ADCCtrl::mHandleADC2;
ADC_HandleTypeDef ADCCtrl::mHandleADC3;


ADCCtrl::ADCCtrl() {
	// TODO Auto-generated constructor stub

}

ADCCtrl::~ADCCtrl() {
	// TODO Auto-generated destructor stub
}


//ADC2
void ADCCtrl::ADC2Init_HAL() {
	MX_ADC2_Init();
}

void ADCCtrl::ADC2DeInit_HAL() {
	MX_ADC2_DeInit();
}

void ADCCtrl::ADC2Calibration() {
	HAL_ADCEx_Calibration_Start(&mHandleADC2, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
}

void ADCCtrl::ADC2Start_Conversion() {
	HAL_ADC_Start(&mHandleADC2);
}

void ADCCtrl::ADC2Conversion_wait(int pTimeout) {
	HAL_ADC_PollForConversion(&mHandleADC2, pTimeout);
}

int ADCCtrl::ADC2_Read() {
	return HAL_ADC_GetValue(&mHandleADC2);
}


//ADC3
void ADCCtrl::ADC3Init_HAL() {
	MX_ADC3_Init();
}

void ADCCtrl::ADC3DeInit_HAL() {
	MX_ADC3_DeInit();
}

void ADCCtrl::ADC3Calibration() {
	//HAL_ADCEx_Calibration_Start(&mHandleADC3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&mHandleADC3, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);//線形キャリブレーション
}

void ADCCtrl::ADC3InjectedStart_IT() {
	HAL_ADCEx_InjectedStart_IT(&mHandleADC3);
}

//int ADCCtrl::ADC3_INJ_Read_ch1() const{
//	return ADC3 -> JDR1;
//}
//
//int ADCCtrl::ADC3_INJ_Read_ch2() const{
//	return ADC3 -> JDR2;
//}
//
//int ADCCtrl::ADC3_INJ_Read_ch3() const{
//	return ADC3 -> JDR3;
//}


void ADCCtrl::ADC3IRQHandler() {
	HAL_ADC_IRQHandler(&mHandleADC3);
}


//以下HALコード置き場
void ADCCtrl::MX_ADC2_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  /** Common config
  */
  mHandleADC2.Instance = ADC2;
  mHandleADC2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV6;
  mHandleADC2.Init.Resolution = ADC_RESOLUTION_16B;
  mHandleADC2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  mHandleADC2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  mHandleADC2.Init.LowPowerAutoWait = DISABLE;
  mHandleADC2.Init.ContinuousConvMode = DISABLE;
  mHandleADC2.Init.NbrOfConversion = 1;
  mHandleADC2.Init.DiscontinuousConvMode = DISABLE;
  mHandleADC2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  mHandleADC2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  mHandleADC2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  mHandleADC2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  mHandleADC2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  mHandleADC2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&mHandleADC2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&mHandleADC2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}


void ADCCtrl::MX_ADC2_DeInit(void) {
	if (HAL_ADC_DeInit(&mHandleADC2) != HAL_OK) {
		Error_Handler();
	}
}


void ADCCtrl::MX_ADC3_Init(void)
{
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /** Common config
  */
  mHandleADC3.Instance = ADC3;
  mHandleADC3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV6;
  mHandleADC3.Init.Resolution = ADC_RESOLUTION_16B;
  mHandleADC3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  mHandleADC3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  mHandleADC3.Init.LowPowerAutoWait = DISABLE;
  mHandleADC3.Init.ContinuousConvMode = DISABLE;
  mHandleADC3.Init.NbrOfConversion = 1;
  mHandleADC3.Init.DiscontinuousConvMode = DISABLE;
  mHandleADC3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  mHandleADC3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  mHandleADC3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&mHandleADC3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Disable Injected Queue
  */
  HAL_ADCEx_DisableInjectedQueue(&mHandleADC3);
  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedLeftBitShift = ADC_LEFTBITSHIFT_NONE;
  sConfigInjected.InjectedNbrOfConversion = 3;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO2;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&mHandleADC3, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_0;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&mHandleADC3, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_6;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
  if (HAL_ADCEx_InjectedConfigChannel(&mHandleADC3, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

}


void ADCCtrl::MX_ADC3_DeInit(void) {
	if (HAL_ADC_DeInit(&mHandleADC3) != HAL_OK) {
		Error_Handler();
	}
}


void ADCCtrl::HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC3)
  {
  /* USER CODE BEGIN ADC3_MspInit 0 */

  /* USER CODE END ADC3_MspInit 0 */
    /* ADC3 clock enable */
    __HAL_RCC_ADC3_CLK_ENABLE();

    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**ADC3 GPIO Configuration
    PF10     ------> ADC3_INP6
    PC2_C     ------> ADC3_INP0
    PC3_C     ------> ADC3_INP1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC2, SYSCFG_SWITCH_PC2_OPEN);

    HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC3, SYSCFG_SWITCH_PC3_OPEN);

    /* ADC3 interrupt Init */

    HAL_NVIC_SetPriority(ADC3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC3_IRQn);

    /* USER CODE BEGIN ADC3_MspInit 1 */

  /* USER CODE END ADC3_MspInit 1 */
  }
}

void ADCCtrl::HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC3)
  {
  /* USER CODE BEGIN ADC3_MspDeInit 0 */

  /* USER CODE END ADC3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC3_CLK_DISABLE();

    /**ADC3 GPIO Configuration
    PF10     ------> ADC3_INP6
    PC2_C     ------> ADC3_INP0
    PC3_C     ------> ADC3_INP1
    */
    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_10);

    /* ADC3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(ADC3_IRQn);
  /* USER CODE BEGIN ADC3_MspDeInit 1 */

  /* USER CODE END ADC3_MspDeInit 1 */
  }
}
