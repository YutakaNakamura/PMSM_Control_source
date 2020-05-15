/*
 * ADCCtrl.hpp
 *
 *  Created on: Sep 21, 2019
 *      Author: watashi
 */

#ifndef INC_ADCCTRL_HPP_
#define INC_ADCCTRL_HPP_

#include "STM32SystemPack.h"

class ADCCtrl {
private:
public:
	static ADC_HandleTypeDef mHandleADC2;
	static ADC_HandleTypeDef mHandleADC3;


	ADCCtrl();
	virtual ~ADCCtrl();

	//static void ADC2Init();
	static void ADC2Enable();
	static void ADC2Disable();
	//HAL関数を使った定義
	static void ADC2Init_HAL();
	static void ADC2DeInit_HAL();
	static void ADC2Calibration();
	static void ADC2Start_Conversion();
	static void ADC2Conversion_wait(int pTimeout);
	static int ADC2_Read();
	//以下HALのWrapper
	static void MX_ADC2_Init(void);
	static void MX_ADC2_DeInit(void);


	//static void ADC3Init();
	static void ADC3Enable();
	static void ADC3Disable();


	//HAL関数を使った定義
	static void ADC3Init_HAL();
	static void ADC3DeInit_HAL();
	static void ADC3Calibration();
	static void ADC3InjectedStart_IT();

	volatile static inline bool ADC3_JEOS () {
//		bool test = ADC3 -> JSQR; 				//JL
		volatile bool jeos = ADC3->ISR && ADC_ISR_JEOS;
//		bool jeos = ADC3 -> ADC_ISR_JEOS;
		return jeos;
	}

	//レジスタ叩く時には必ずvolatile宣言をつける
	static volatile int ADC3_INJ_Read_ch1() { return ADC3 -> JDR1; }
	static volatile int ADC3_INJ_Read_ch2() { return ADC3 -> JDR2; }
	static volatile int ADC3_INJ_Read_ch3() { return ADC3 -> JDR3; }

	static void ADC3IRQHandler();

	//以下HALのWrapper
	//TODO:考慮すること12/12  static宣言必要か？
	static void MX_ADC3_Init(void);
	static void MX_ADC3_DeInit(void);
	static void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle);
	static void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle);
};



#endif /* INC_ADCCTRL_HPP_ */
