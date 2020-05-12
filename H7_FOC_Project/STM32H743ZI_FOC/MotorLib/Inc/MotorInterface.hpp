/*
 * MotorInterface.hpp
 *
 *  Created on: Dec 13, 2019
 *      Author: watashi
 */

#ifndef SRC_MOTORINTERFACE_HPP_
#define SRC_MOTORINTERFACE_HPP_

#include "paramsetting.h" //パラメータのマクロ
#include "MotorLibDefPack.hpp"

// ../SystemLib/Inc/
#include "STM32SystemPack.h"

#include "ADCCtrl.hpp"
#include "TIMCtrl.hpp"
#include "SPICtrl.hpp"
#include "EncoderABZCtrl.hpp"

//#include "GPIOInit.hpp"
//#include "USARTInit.hpp"

#include "MotorInfo.hpp"
#include "UIStatus.hpp"

#include "ArgCtrl.hpp"
#include "MotorMath.hpp"
#include "PID.hpp"
#include "Observer.hpp"
//#include "HFConvolution.hpp"
#include "SineWaveConvCalculator.hpp"

//高周波

//鏡相推定器
#include "MirrorPhaseEstimator.hpp"
//PLL
#include "PhaseSynchronizer.hpp"
//高周波生成器
#include "HighFreqVoltageCommander.hpp"


#include <array>
#include <vector>

#include "PWM.hpp"
//#include "UART.hpp"

#include "DebugCtrl.hpp"
//#include "DebugOutput.hpp"



#include "WaveGenerator.hpp"//パラメータ測定用

//インターフェイスだからな！！ここに実装するなよ！！！
class MotorInterface {
private:
public:
	MotorInterface();
	virtual ~MotorInterface();

	virtual void InitSystem(void) = 0;
	virtual void DeInitSystem(void) = 0;
	virtual void InitMotorControl(void) = 0;
	virtual void InitObserver(void) = 0;

	virtual void HighFreqTask(void) = 0;

	//virtual void MotorMeasure(void) = 0;ここは共通でないからインターフェイスではない
	//virtual void MotorDrive(void) = 0;ここは共通でないからインターフェイスではない
	//MotorDrive内呼び出し関数
	virtual void ReadCurrentTask() = 0;
	virtual void setIuvw(float pIu, float pIv, float pIw) = 0;
	virtual void ReadVoltageTask() = 0;

	virtual void ReadAngleTask(void) = 0;
	virtual fp_rad GetAngleForOpenLoop() = 0;
	virtual fp_rad GetAngleForFOC() = 0;

	virtual void clarkTransform(void) = 0;
	virtual void parkTransform(void) = 0;
	virtual void parkGanmaDelta(void) = 0;
	virtual void parkabtogd(void) = 0;

	//virtual void ObserverTask() = 0;

	virtual void CurrentControlTask() = 0;
	virtual std::array<float, 2> GetCurrentTarget() = 0;
	//virtual void CurrentFeedForwardTask() = 0;
	virtual void CurrentPITask() = 0;
	virtual std::array<float, 2> PIDgd_control(std::array<float, 2> pErrIgd) = 0;//どちらかが死にclassになるけど毎回呼ぶ作業でif文使いたくない。
	virtual void setVgd(std::array<float, 2> pVgd) = 0;

	virtual std::array<float, 2> getIdq(void) = 0;
	virtual std::array<float, 2> getIgd(void) = 0;

	virtual void invParkgdtoab(void) = 0;
	virtual void invParkGanmaDelta(void) = 0;
	virtual void invParkTransform(void) = 0;
	virtual void invClarkTransform(void) = 0;
	virtual void SVM(void) = 0;

	virtual void VoltageOutputTask(void) = 0;

	//virtual void VelocityPIDTask() = 0;
	virtual void GPIODebugTask() = 0;

	//Handler
	virtual void ControlModeHandler() = 0;

	//UI
	virtual void BtnAct(void) = 0;
	virtual void BtnActOFF(void) = 0;
	virtual void BtnActON(void) = 0;


};

#endif /* SRC_MOTORINTERFACE_HPP_ */
