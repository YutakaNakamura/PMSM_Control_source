/*
 * MotorMeasure.cpp
 *
 *  Created on: Dec 14, 2019
 *      Author: watashi
 */

#include "MotorMeasure.hpp"

MotorMeasure::MotorMeasure() {
	// TODO Auto-generated constructor stub


}

MotorMeasure::~MotorMeasure() {
	// TODO Auto-generated destructor stub
}

void MotorMeasure::InitSystem(void) {
	//以下CubeMXに頼らない定義たち
	//mainで既に定義されているとうまく動かないから、Mainで定義する前に呼び出すこと。
//	GPIOInit::Init();//これめんどい
//	USARTInit::Init();//これめんどい


	//Timer Initialize
	TIMCtrl::MX_TIM1_Init();

	TIMCtrl::MotorDuty_ch1(0);//50%duty
	TIMCtrl::MotorDuty_ch2(0);
	TIMCtrl::MotorDuty_ch3(0);
	//TIMCtrl::MotorDuty_ch4(0.9);//9割タイミングで打つ
	TIMCtrl::TIM1SetCOMP_ch4(PWM_PERIOD_COUNT - 1);

	//ENABLE信号 PWMSet_Pin|OCSet_Pin|GateEnable_Pin
	HAL_GPIO_WritePin(PWMSet_GPIO_Port, PWMSet_Pin, GPIO_PIN_RESET);//6PWM
	//HAL_GPIO_WritePin(PWMSet_GPIO_Port, PWMSet_Pin, GPIO_PIN_SET);//3PWM
	HAL_GPIO_WritePin(OCSet_GPIO_Port, OCSet_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GateEnable_GPIO_Port, GateEnable_Pin, GPIO_PIN_SET);

	TIMCtrl::TIM1PWMStart();

	//Encoder Initialize
	//EncoderABZCtrl::MX_TIM4_Init();
	//EncoderABZCtrl::EncoderStart();

	mMeasureTarget = MeasureTarget::Ldd;
	//mMeasureTarget = MeasureTarget::Lqq;
	mWaveGen.InitFrequency(440);
	mFreqMode = 0;


	//ADC Initialize
	ADCCtrl::ADC2Init_HAL();
	ADCCtrl::ADC2Calibration();

	ADCCtrl::ADC3Init_HAL();
	ADCCtrl::ADC3Calibration();
	ADCCtrl::ADC3InjectedStart_IT();
}


void MotorMeasure::DeInitSystem(void) {
	HAL_GPIO_WritePin(GateEnable_GPIO_Port, GateEnable_Pin, GPIO_PIN_RESET);//Gate Disable

	//以降Initの逆順にDeInitしていくこと
	ADCCtrl::ADC3DeInit_HAL();
	ADCCtrl::ADC2DeInit_HAL();

	//TIMをDeIntする前に安全のため、50%Duty,I=0に戻す
	TIMCtrl::MotorDuty_ch1(0);//50%duty
	TIMCtrl::MotorDuty_ch2(0);
	TIMCtrl::MotorDuty_ch3(0);
	//TIMCtrl::MotorDuty_ch4(0.9);//9割タイミングで打つ
	TIMCtrl::TIM1SetCOMP_ch4(PWM_PERIOD_COUNT - 1);

	//Timer Initialize
	TIMCtrl::MX_TIM1_DeInit();

	//ENABLE信号 PWMSet_Pin|OCSet_Pin|GateEnable_Pin
	HAL_GPIO_WritePin(PWMSet_GPIO_Port, PWMSet_Pin, GPIO_PIN_RESET);//6PWM
	//HAL_GPIO_WritePin(PWMSet_GPIO_Port, PWMSet_Pin, GPIO_PIN_SET);//3PWM
	HAL_GPIO_WritePin(OCSet_GPIO_Port, OCSet_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GateEnable_GPIO_Port, GateEnable_Pin, GPIO_PIN_RESET);
}


void MotorMeasure::InitMotorControl(void) {
	//mOperationMode = Drive;//起動時、動作をまずは運転にする。
	mControlMode = OpenLoop;//起動時、動作をまずは強制転流にする。

	mArgCtrl.Init();

	mIganmaPID.SetParam(PID_GAIN_IGANMA_P, PID_GAIN_IGANMA_I, PID_GAIN_IGANMA_D);
	mIdeltaPID.SetParam(PID_GAIN_IDELTA_P, PID_GAIN_IDELTA_I, PID_GAIN_IDELTA_D);
	mIganmaPID.SetSampleTime(PID_CYCLE_TIME);
	mIdeltaPID.SetSampleTime(PID_CYCLE_TIME);

	mVelocityPID.SetParam(PID_GAIN_VEL_P, PID_GAIN_VEL_I, PID_GAIN_VEL_D);
	mVelocityPID.SetSampleTime(PID_CYCLE_TIME_VEL);
}

void MotorMeasure::InitObserver(void) {
	mObserver.InitEMFObs(OBSERVER_CYCLE_TIME, M_PARAM_R, M_PARAM_LD, M_PARAM_LQ, OBSERVER_GAIN_ALPHA);
	mObserver.InitPhaseEstimator(OBSERVER_CYCLE_TIME, OBSERVER_GAIN_K1, OBSERVER_GAIN_K2, OBSERVER_GAIN_K3);
}


void MotorMeasure::HighFreqTask(void) {

	Measure();

}

void MotorMeasure::Measure(void) {//モータを測定するモード

	GPIODebugTask();//GPIOからオシロに波形を出力する

	SPITask();

	GPIODebugTask();//GPIOからオシロに波形を出力する

	//これは指令値決定用ADC。開始直後にADC2を読み取って、変換時間を演算処理の中で相殺する。
	//ADCCtrl::ADC2Start_Conversion();
	//ADCCtrl::ADC2Conversion_wait(10);

	ReadCurrentTask();
	ReadVoltageTask();

	ReadAngleTask(); //mgdArgを取得する。測定の時はどうするの？要エンコーダにする？

	//Iuvw -> Iab
	clarkTransform();
	//Iab -> Igd
	parkabtogd();//mgdArgで回転

	GPIODebugTask();//GPIOからオシロに波形を出力する

	CurrentControlTask();//電流制御とか

	GPIODebugTask();//GPIOからオシロに波形を出力する

	//Vgd　->　Vab
	invParkgdtoab();
	//Vab -> Vuvw
	SVM();

	VoltageOutputTask();//PWM出力

	ControlModeHandler();//次の運転制御モードを決める関数

	//DEBUG
	GPIODebugTask();//GPIOからオシロに波形を出力する

	if(DEBUG_MODE){//デバッグモードで入る処理
		mDebugCtrl.RTTOutput(mMotorInfo, mUIStatus);

	}
	GPIODebugTask();//GPIOからオシロに波形を出力する
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
}

void MotorMeasure::SPITask(void) {
	SPICtrl::GetInstance().SPITransmitReceive();
	std::array<int,(SPI_DATA_SIZE/4)> rxint = SPICtrl::GetInstance().GetRxInt();
	float Vh = (float)rxint.at(0)/(float)4095;
	mMotorInfo.mVh = Vh;
	float Voffset = (float)rxint.at(1)/(float)4095;
	mMotorInfo.mVoffset = Voffset;
}

void MotorMeasure::ReadCurrentTask(void) {
	//ReadCurrent
	//エンコーダ読み取り
	float Iu,Iv,Iw;
	//増幅率のバイアス考慮してない。あとで計算すること。
	Iu = (float)ADCCtrl::ADC3_INJ_Read_ch1() * BOARD_IV_RATIO * ADC_VOLTAGE_RATIO + BOARD_IV_OFFSET;
	Iv = (float)ADCCtrl::ADC3_INJ_Read_ch2() * BOARD_IV_RATIO * ADC_VOLTAGE_RATIO + BOARD_IV_OFFSET;
	Iw = (float)ADCCtrl::ADC3_INJ_Read_ch3() * BOARD_IV_RATIO * ADC_VOLTAGE_RATIO + BOARD_IV_OFFSET;
	Iu = -Iu;//モータ「に」流す電流にするため、反転。
	Iv = -Iv;
	Iw = -Iw;

	std::array<float,3> Iuvw = {Iu, Iv, Iw};

	//Iw = -Iu - Iv;
	mMotorInfo.setIuvw(Iuvw);
}


void MotorMeasure::ReadVoltageTask() {
	//ReadVoltage
	//電圧測定(入力)
	mMotorInfo.mVoltageVCC = VCC_VOLTAGE;
}


////Motor
//void MotorMeasure::setIuvw(float pIu, float pIv, float pIw){
//	mMotorInfo.mIuvw.at(0) = pIu;
//	mMotorInfo.mIuvw.at(1) = pIv;
//	mMotorInfo.mIuvw.at(2) = pIw;
//}

void MotorMeasure::ReadAngleTask(void) {
	if(mControlMode == OpenLoop || mControlMode == OpenLoopToFOC) {
		mMotorInfo.mgdArg = GetAngleForOpenLoop();
	}else if(mControlMode == FOC) {
		mMotorInfo.mgdArg = GetAngleForFOC();
	}
}


fp_rad MotorMeasure::GetAngleForOpenLoop(void) {
	//強制転流では、gd軸を回転させる。
	if(mUIStatus.mStartStopTRG) {
	mArgCtrl.accelerationForOpenLoop();
	} else {
	mArgCtrl.decelerationForOpenLoop();
	}
	return mArgCtrl.getArg(); //gd軸のみ回す。
}


fp_rad MotorMeasure::GetAngleForFOC(void) {
	return mObserver.GetEstTheta();//オブザーバから読み取る
}


void MotorMeasure::clarkTransform(void) {
	std::array<float, 3> Iuvw = mMotorInfo.getIuvw();
	std::array<float, 2> Iab = MotorMath::clarkTransform(Iuvw);
	mMotorInfo.mIab = Iab;
}

void MotorMeasure::parkTransform(void) {
	fp_rad dqArg = mMotorInfo.mdqArg;
	std::array<float, 2> Iab = mMotorInfo.mIab;
	std::array<float, 2> Idq = MotorMath::parkTransform(dqArg, Iab);
	mMotorInfo.mIdq = Idq;
}

void MotorMeasure::parkGanmaDelta(void) {
	fp_rad ArgErr = mMotorInfo.mArgErr;
	std::array<float, 2> Idq = mMotorInfo.mIdq;
	fp_rad InvArgErr = -1.0f * ArgErr;
	std::array<float, 2> Igd = MotorMath::parkTransform(InvArgErr, Idq);
	mMotorInfo.mIgd = Igd;
}

//optional function
void MotorMeasure::parkabtogd(void) {
	fp_rad gdArg = mMotorInfo.mgdArg;
	std::array<float, 2> Iab = mMotorInfo.mIab;
	std::array<float, 2> Igd = MotorMath::parkTransform(gdArg, Iab);
	mMotorInfo.mIgd = Igd;
}

std::array<float, 2> MotorMeasure::getIdq() {
	return mMotorInfo.mIdq;
}

std::array<float, 2> MotorMeasure::getIgd() {
	return mMotorInfo.mIgd;
}


void MotorMeasure::CurrentControlTask() {
	//Current Target Setting
	mMotorInfo.mIgdTarget = GetCurrentTarget();
	CurrentPITask();
}

std::array<float, 2> MotorMeasure::GetCurrentTarget() {
	//SPIを読み取って、
	//mMotorInfo.mIgdTargetを操作するTask

	//のはずなんだけどすでにSPITaskで処理しちゃってるから、どうしようかなあ…


	float offsetCurrent =  0;
	//TODO:ここのoffsetを外部入力できるようにしたほうがいい

	std::array<float, 2> IgdTarget = {0, 0};

	if(mMeasureTarget == Ldd) {
		IgdTarget.at(1) = offsetCurrent;//IdTarget [A]
		return IgdTarget;
	} else if (mMeasureTarget == Lqd) {
		IgdTarget.at(1) = offsetCurrent;
		return IgdTarget;
	} else if (mMeasureTarget == Ldq) {
		IgdTarget.at(0) = offsetCurrent;
		return IgdTarget;
	} else if (mMeasureTarget == Lqq) {
		IgdTarget.at(0) = offsetCurrent;
		return IgdTarget;
	}

	return IgdTarget;
}


void MotorMeasure::CurrentPITask() {
	//PI Control Start

	if(mMeasureTarget == Ldd || mMeasureTarget == Lqd) {
		//float adc2_input = (float)ADCCtrl::ADC2_Read() / 65535;

		//float Vh = mMotorInfo.mVoltageVCC * adc2_input;//[V]
		float Vh = mMotorInfo.mVh;
		//float VqOffset = 0 * adc2_input;
		float VgOffset = mMotorInfo.mVoffset;
		float VgInput = Vh * mWaveGen.OutputWaveform() + VgOffset;

		mMotorInfo.mIgdErr.at(0) = 0;//ganmaの方はPI制御はかけない(sin波入れる)
		mMotorInfo.mIgdErr.at(1) = mMotorInfo.mIgdTarget.at(1) - mMotorInfo.mIgd.at(1);
		std::array<float,2> CulcVgd = PIDgd_control(mMotorInfo.mIgdErr);
		float CulcVd = CulcVgd.at(1);

		setVg(VgInput);
		setVd(CulcVd);
		return;

	} else if (mMeasureTarget == Lqq || mMeasureTarget == Ldq) {
		//float adc2_input = (float)ADCCtrl::ADC2_Read() / 65535;

		//float Vh = mMotorInfo.mVoltageVCC * adc2_input;//[V]
		float Vh = mMotorInfo.mVh;
		//float VqOffset = 0 * adc2_input;
		float VdOffset = mMotorInfo.mVoffset;
		float VdInput = Vh * mWaveGen.OutputWaveform() + VdOffset;

		mMotorInfo.mIgdErr.at(0) = mMotorInfo.mIgdTarget.at(0) - mMotorInfo.mIgd.at(0);
		mMotorInfo.mIgdErr.at(1) = 0;//deltaの方はPI制御はかけない(sin波入れる)
		std::array<float,2> CulcVgd = PIDgd_control(mMotorInfo.mIgdErr);
		float CulcVg = CulcVgd.at(0);

		setVg(CulcVg);
		setVd(VdInput);
		return;
	}

}


std::array<float, 2> MotorMeasure::PIDgd_control(std::array<float, 2> pErrIgd) {
		float ErrIganma = pErrIgd.at(0);
		float ErrIdelta = pErrIgd.at(1);
		mIganmaPID.ErrorUpdate(ErrIganma);
		mIdeltaPID.ErrorUpdate(ErrIdelta);

		float Vganma = mMotorInfo.mVgd.at(0);
		float Vdelta = mMotorInfo.mVgd.at(1);

		Vganma = mIganmaPID.OutPut();
		Vdelta = mIdeltaPID.OutPut();

		return {Vganma, Vdelta};
}


void MotorMeasure::setVg(float pVg) {
	float Vganma = pVg;
	//上限Limit
	if( Vganma > PID_IGANMA_MAX_VOLTAGE ) {
		Vganma = PID_IGANMA_MAX_VOLTAGE;
	}
	//下限Limit
	if( Vganma < PID_IGANMA_MIN_VOLTAGE ) {
		Vganma = PID_IGANMA_MIN_VOLTAGE;
	}
	//Limitを適用
	mMotorInfo.mVgd.at(0) = Vganma;
}


void MotorMeasure::setVd(float pVd) {
	float Vdelta = pVd;
	//上限Limit
	if( Vdelta > PID_IDELTA_MAX_VOLTAGE ) {
		Vdelta = PID_IDELTA_MAX_VOLTAGE;
	}
	//下限Limit
	if( Vdelta < PID_IDELTA_MIN_VOLTAGE ) {
		Vdelta = PID_IDELTA_MIN_VOLTAGE;
	}
	//Limitを適用
	mMotorInfo.mVgd.at(1) = Vdelta;
}


void MotorMeasure::setVgd(std::array<float, 2> pVgd) {
	float Vganma = pVgd.at(0);
	float Vdelta = pVgd.at(1);
	//上限Limit
	if( Vganma > PID_IGANMA_MAX_VOLTAGE ) {
		Vganma = PID_IGANMA_MAX_VOLTAGE;
	}
	if( Vdelta > PID_IDELTA_MAX_VOLTAGE ) {
		Vdelta = PID_IDELTA_MAX_VOLTAGE;
	}
	//下限Limit
	if( Vganma < PID_IGANMA_MIN_VOLTAGE ) {
		Vganma = PID_IGANMA_MIN_VOLTAGE;
	}
	if( Vdelta < PID_IDELTA_MIN_VOLTAGE ) {
		Vdelta = PID_IDELTA_MIN_VOLTAGE;
	}
	//Limitを適用
	mMotorInfo.mVgd = {Vganma, Vdelta};
}


//optional function
void MotorMeasure::invParkgdtoab(void) {
	std::array<float, 2> Vgd = mMotorInfo.mVgd;
	fp_rad gdArg = mMotorInfo.mgdArg;
	std::array<float, 2> Vab = MotorMath::InvparkTransform(gdArg, Vgd);
	mMotorInfo.mVab = Vab;
}


void MotorMeasure::invParkGanmaDelta(void) {
	std::array<float, 2> Vgd = mMotorInfo.mVgd;
	fp_rad ArgErr = mMotorInfo.mArgErr;
	fp_rad InvArgErr = -1.0f * ArgErr;
	std::array<float, 2> Vdq = MotorMath::InvparkTransform(InvArgErr, Vgd);
	mMotorInfo.mVdq = Vdq;
}


void MotorMeasure::invParkTransform(void) {
	fp_rad dqArg = mMotorInfo.mdqArg;
	std::array<float, 2> Vdq= mMotorInfo.mVdq;
	std::array<float, 2> Vab = MotorMath::InvparkTransform(dqArg, Vdq);
	mMotorInfo.mVab = Vab;
}


//void MotorMeasure::invClarkTransform(void) {
//	std::array<float, 2> Vab = {mMotorInfo.mVab.at(0)/mMotorInfo.mVoltageVCC,
//											  mMotorInfo.mVab.at(1)/mMotorInfo.mVoltageVCC};
//	std::array<float, 3> Vuvw = MotorMath::InvclarkTransform(Vab);
//	mMotorInfo.mDutyuvw = Vuvw;
//}


void MotorMeasure::SVM(void) {
	mMotorInfo.mDutyuvw = MotorMath::SVM(mMotorInfo.mVab, mMotorInfo.mVoltageVCC);
}


void MotorMeasure::VoltageOutputTask(void) {
	//0~1のDutyをPWMで出力する。
	//0 <= mMotorInfo.mDuty <= 1
	TIMCtrl::floatDuty_ch1(mMotorInfo.mDutyuvw.at(0));
	TIMCtrl::floatDuty_ch2(mMotorInfo.mDutyuvw.at(1));
	TIMCtrl::floatDuty_ch3(mMotorInfo.mDutyuvw.at(2));
}


void MotorMeasure::ControlModeHandler() { //状態遷移を管理する関数
	float OpenLoopOmega = mArgCtrl.getArgOmega();
	float ObserverOmega = mObserver.GetEstOmegaE();

	switch(mControlMode) {
	case OpenLoop:
		if(OpenLoopOmega > OPENLOOP_END_OMEGA) {
			mControlMode = OpenLoopToFOC;
		}
		break;
	case OpenLoopToFOC:
		//OpenLoopへの遷移
		if(OpenLoopOmega < OPENLOOP_END_OMEGA) {
			mControlMode = OpenLoop;
		}
		//FOCへの遷移
		if(OPEN_TO_FOC_TRANSITION_COUNT_STEP2 < mTransitionCountForOpenToFOC2) {
			mControlMode = FOC; //何が起こるかわくわく　ここで本来書くべきではない。Handerが管理するべき。
			mTransitionCountForOpenToFOC = 0;
			mTransitionCountForOpenToFOC2 = 0;
		}
		break;
	case FOC:
		if(200 > ObserverOmega ){ //定常状態は400
			mControlMode = OpenLoopToFOC;
		}
		break;
	default:
		break;
	}

}

void MotorMeasure::GPIODebugTask() {//Lチカでタイミングをオシロで見る
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
	asm("NOP");
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
}


//UICtrl
void MotorMeasure::BtnAct(void){//強制転流開始へのトリガ 割り込みから叩くためにここでラッパする
//	if(mUIStatus.mStartStopTRG){
//		BtnActOFF();
//	} else {
//		BtnActON();
//	}
	ChangeFreq();
}


void MotorMeasure::BtnActOFF(void){//強制転流開始へのトリガOFF
	mUIStatus.mStartStopTRG = MotorStop;
	mControlMode = OpenLoop;
}


void MotorMeasure::BtnActON(void){//強制転流開始へのトリガON
	mUIStatus.mStartStopTRG = MotorStart;
}

void MotorMeasure::ChangeFreq(void){//周波数の変更


	if(mFreqMode == 0) {
		mWaveGen.InitFrequency(261);
		mFreqMode++;
		return;
	}

	else if(mFreqMode == 1) {
		mWaveGen.InitFrequency(329);
		mFreqMode++;
		return;
	}

	else if(mFreqMode == 2) {
		mWaveGen.InitFrequency(391);
		mFreqMode++;
		return;
	}

	else if(mFreqMode == 3) {
		mWaveGen.InitFrequency(523);
		mFreqMode++;
		return;
	}

	else if(mFreqMode == 4) {
		mWaveGen.InitFrequency(0);
		mWaveGen.ResetPhase();
		mFreqMode = 0;
		return;
	}

}

