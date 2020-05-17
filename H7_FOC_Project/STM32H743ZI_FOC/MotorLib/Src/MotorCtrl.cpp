/*
 * MotorCtrl.cpp
 *
 *  Created on: Aug 5, 2019
 *      Author: watashi
 */

#include "MotorCtrl.hpp"

//制御器全般
constexpr float controllerSamplingTime = CONTROLLER_SAMPLING_TIME;
constexpr float IntegratorGain = 1;


//電流制御器
constexpr float CurrPIConstOmega = 10; //[rad/s] 電流PIの応答速度
constexpr float CurrPIConstTime = 1/CurrPIConstOmega; //[s] 電流PIの時間

constexpr float CurrPIGainIganma_P = M_PARAM_LD/CurrPIConstTime;
constexpr float CurrPIGainIganma_I = CurrPIConstTime/M_PARAM_R;

constexpr float CurrPIGainIdelta_P = M_PARAM_LQ/CurrPIConstTime;
constexpr float CurrPIGainIdelta_I = CurrPIConstTime/M_PARAM_R;


//高周波重畳
constexpr float ConvHighFreq = HF_CONV_FREQ;
constexpr float HighFreqOmega = ConvHighFreq * 2 * M_PI;

constexpr float MirrorPhaseEstimator_A0 = 250*250;
constexpr float MirrorPhaseEstimator_A1 = 2*250;
constexpr float MirrorPhaseEstimator_B0 = 250*250;
constexpr float MirrorPhaseEstimator_B1 = 0;
constexpr float MirrorPhaseEstimator_B2 = 0;

constexpr float PhaseSynchronizer_Cn0 = 5625;
constexpr float PhaseSynchronizer_Cn1 = 150;

MotorCtrl::MotorCtrl()
: mIganmaPID(controllerSamplingTime, CurrPIGainIganma_P, CurrPIGainIganma_I, 0),
  mIdeltaPID(controllerSamplingTime, CurrPIGainIdelta_P, CurrPIGainIdelta_I, 0),
  mPhaseSpeedEstimator(controllerSamplingTime, IntegratorGain,
		MirrorPhaseEstimator_A0, MirrorPhaseEstimator_A1,
		MirrorPhaseEstimator_B0, MirrorPhaseEstimator_B1, MirrorPhaseEstimator_B2,
		PhaseSynchronizer_Cn0,PhaseSynchronizer_Cn1)
{
}

MotorCtrl::~MotorCtrl() {
	DeInitSystem();
}

void MotorCtrl::InitSystem(void) {
	//以下CubeMXに頼らない定義たち
	//mainで既に定義されているとうまく動かないから、Mainで定義する前に呼び出すこと。
//	GPIOInit::Init();//これめんどい
//	USARTInit::Init();//これめんどい


	//Timer Initialize
	TIMCtrl::MX_TIM1_Init();

	TIMCtrl::MotorDuty_ch1(0);//50%duty
	TIMCtrl::MotorDuty_ch2(0);
	TIMCtrl::MotorDuty_ch3(0);
	TIMCtrl::MotorDuty_ch4(0.99);//9割9分でタイミングで打つ
	//TIMCtrl::TIM1SetCOMP_ch4(PWM_PERIOD_COUNT - 1);

	//ENABLE信号 PWMSet_Pin|OCSet_Pin|GateEnable_Pin
	HAL_GPIO_WritePin(PWMSet_GPIO_Port, PWMSet_Pin, GPIO_PIN_RESET);//6PWM
	//HAL_GPIO_WritePin(PWMSet_GPIO_Port, PWMSet_Pin, GPIO_PIN_SET);//3PWM
	HAL_GPIO_WritePin(OCSet_GPIO_Port, OCSet_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GateEnable_GPIO_Port, GateEnable_Pin, GPIO_PIN_SET);

	TIMCtrl::TIM1PWMStart();

	//Encoder Initialize
	//EncoderABZCtrl::MX_TIM4_Init();
	//EncoderABZCtrl::EncoderStart();

	//ADC Initialize
	ADCCtrl::ADC2Init_HAL();
	ADCCtrl::ADC2Calibration();

	ADCCtrl::ADC3Init_HAL();
	ADCCtrl::ADC3Calibration();
	ADCCtrl::ADC3InjectedStart_IT();
}


void MotorCtrl::DeInitSystem(void) {
	HAL_GPIO_WritePin(GateEnable_GPIO_Port, GateEnable_Pin, GPIO_PIN_RESET);//Gate Disable

	//以降Initの逆順にDeInitしていくこと
	ADCCtrl::ADC3DeInit_HAL();
	ADCCtrl::ADC2DeInit_HAL();

	//TIMをDeIntする前に安全のため、50%Duty,I=0に戻す
	TIMCtrl::MotorDuty_ch1(0);//50%duty
	TIMCtrl::MotorDuty_ch2(0);
	TIMCtrl::MotorDuty_ch3(0);

	//Timer Initialize
	TIMCtrl::MX_TIM1_DeInit();

	//ENABLE信号 PWMSet_Pin|OCSet_Pin|GateEnable_Pin
	HAL_GPIO_WritePin(PWMSet_GPIO_Port, PWMSet_Pin, GPIO_PIN_RESET);//6PWM
	//HAL_GPIO_WritePin(PWMSet_GPIO_Port, PWMSet_Pin, GPIO_PIN_SET);//3PWM
	HAL_GPIO_WritePin(OCSet_GPIO_Port, OCSet_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GateEnable_GPIO_Port, GateEnable_Pin, GPIO_PIN_RESET);
}


void MotorCtrl::InitMotorControl(void) {

	mControlMode = OpenLoop;//起動時、動作をまずは強制転流にする。

	if(SINEWAVE_CONVOLUTION) { //高周波重畳アプリケーション
		mControlMode = FOC_Convolution;
		mMotorInfo.mHighFreqOmega = HighFreqOmega;
	}

	mArgCtrl.Init();

//	mIganmaPID.SetParam(PID_GAIN_IGANMA_P, PID_GAIN_IGANMA_I, PID_GAIN_IGANMA_D);
//	mIdeltaPID.SetParam(PID_GAIN_IDELTA_P, PID_GAIN_IDELTA_I, PID_GAIN_IDELTA_D);
//	mIganmaPID.SetSampleTime(PID_CYCLE_TIME);
//	mIdeltaPID.SetSampleTime(PID_CYCLE_TIME);

	mVelocityPID.SetParam(PID_GAIN_VEL_P, PID_GAIN_VEL_I, PID_GAIN_VEL_D);
	mVelocityPID.SetSampleTime(PID_CYCLE_TIME_VEL);
}

void MotorCtrl::InitObserver(void) {
	//オブザーバ
	mObserver.InitEMFObs(OBSERVER_CYCLE_TIME, M_PARAM_R, M_PARAM_LD, M_PARAM_LQ, OBSERVER_GAIN_ALPHA);
	mObserver.InitPhaseEstimator(OBSERVER_CYCLE_TIME, OBSERVER_GAIN_K1, OBSERVER_GAIN_K2, OBSERVER_GAIN_K3);

}


void MotorCtrl::HighFreqTask(void) {
	MotorDrive();
}


void MotorCtrl::MotorDrive(void) { //モータを動かすモード.他に測定モードを用意する予定

	GPIODebugTask();//GPIOからオシロに波形を出力する

	SetCurrentTarget();
	if(mControlMode == FOC_Convolution) {
		SetVhTask();
		//WaveGenTask();推定器内に移動
	}

	GPIODebugTask();//GPIOからオシロに波形を出力する

	ReadCurrentTask();
	ReadVoltageTask();

	ReadAngleTask(); //mgdArgを取得する。(OpenLoop or FOC分岐あり)

	//Iuvw -> Iab
	clarkTransform();
	//Iab -> Igd
	parkabtogd();//mgdArgで回転

	GPIODebugTask();//GPIOからオシロに波形を出力する

	ObserverTask();//オブザーバ
	VelocityPIDTask();//速度PID制御

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

	if(mDEBUGorSPI == SPI) {
		SPITask();//エラッタなどの理由でとても重いから、２回のうち１回にする
		//mDebugCtrl.RTTOutput(mMotorInfo, mUIStatus);
		//mDebugCtrl.AddOutputString(mMotorInfo);
		//mDebugCtrl.RTTOutput(mMotorInfo, mUIStatus);

	}
	if(mDEBUGorSPI == Debug) {
		if(DEBUG_MODE){//デバッグモードで入る処理
			mDebugCtrl.RTTOutput(mMotorInfo, mUIStatus);
		}
	}

	GPIODebugTask();//GPIOからオシロに波形を出力する
}


void MotorCtrl::SPITask(void) {
	SPICtrl::GetInstance().SPITransmitReceive();
}


void MotorCtrl::SetCurrentTarget() {
	std::array<int,(SPI_DATA_SIZE/4)> rxint = SPICtrl::GetInstance().GetRxInt();
	mMotorInfo.mCurrentTargetInput = (float)rxint.at(0)/(float)4095;
}

void MotorCtrl::SetVhTask() {
	std::array<int,(SPI_DATA_SIZE/4)> rxint = SPICtrl::GetInstance().GetRxInt();
	float Vh = (float)rxint.at(1)/(float)4095;

	if(HF_ARG_ZERO_FIX) {
//		mMotorInfo.mVh = 0;
		mMotorInfo.mVh = Vh;
	} else {
		mMotorInfo.mVh = Vh;
		//mMotorInfo.mVh = 0.06; //通常時は0.3固定にする
		//mMotorInfo.mVh = 0.3; //通常時は0.3固定にする
	}

}


//void MotorCtrl::WaveGenTask() {
//	std::array<float,2> waves = mHFVC.OutputWaves(mMotorInfo.mHighFreqOmega);
//	mMotorInfo.mSinForConv = waves.at(0);
//	mMotorInfo.mCosForConv = waves.at(1);
//}


void volatile MotorCtrl::ReadCurrentTask(void) {
	//ReadCurrent
	//ADC電流読み取り
	while( !ADCCtrl::ADC3_JEOS() ){
		asm volatile ("nop");
	}
	float Iu = (float)ADCCtrl::ADC3_INJ_Read_ch1() * BOARD_IV_RATIO * ADC_VOLTAGE_RATIO + BOARD_IV_OFFSET;
	float Iv = (float)ADCCtrl::ADC3_INJ_Read_ch2() * BOARD_IV_RATIO * ADC_VOLTAGE_RATIO + BOARD_IV_OFFSET;
	float Iw = (float)ADCCtrl::ADC3_INJ_Read_ch3() * BOARD_IV_RATIO * ADC_VOLTAGE_RATIO + BOARD_IV_OFFSET;
	//モータ「に」流す電流にするため、反転。
	Iu = -Iu;
	Iv = -Iv;
	Iw = -Iw;
	std::array<float,3> Iuvw = {Iu, Iv, Iw};

	mMotorInfo.setIuvw(Iuvw);
	return;
}


void MotorCtrl::ReadVoltageTask() {
	//ReadVoltage
	//電圧測定(入力)
	mMotorInfo.mVoltageVCC = VCC_VOLTAGE;
}


void MotorCtrl::ReadAngleTask(void) {
	if(mControlMode == OpenLoop || mControlMode == OpenLoopToFOC) {
		mMotorInfo.mgdArg = GetAngleForOpenLoop();
	}else if(mControlMode == FOC) {
		mMotorInfo.mgdArg = GetAngleForFOC();
	}else if(mControlMode == FOC_Convolution) {
		mMotorInfo.mgdArg = mMotorInfo.mEstTheta;
		if(HF_ARG_ZERO_FIX) {
			mMotorInfo.mgdArg = 0;
		}
	}

}


fp_rad MotorCtrl::GetAngleForOpenLoop(void) {
	//強制転流では、gd軸を回転させる。
	if(mUIStatus.mStartStopTRG) {
	mArgCtrl.accelerationForOpenLoop();
	} else {
	mArgCtrl.decelerationForOpenLoop();
	}
	return mArgCtrl.getArg(); //gd軸のみ回す。
}


fp_rad MotorCtrl::GetAngleForFOC(void) {
	return mObserver.GetEstTheta();//オブザーバから読み取る
}


void MotorCtrl::clarkTransform(void) {
	std::array<float, 3> Iuvw = mMotorInfo.getIuvw();

//	std::array<float, 3> Iuvw = {mMotorInfo.mIuvw.at(0),
//											   mMotorInfo.mIuvw.at(1),
//											   -mMotorInfo.mIuvw.at(0) - mMotorInfo.mIuvw.at(1)};
	std::array<float, 2> Iab = MotorMath::clarkTransform<float>(Iuvw);
	mMotorInfo.mIab = Iab;
}

void MotorCtrl::parkTransform(void) {
	fp_rad dqArg = mMotorInfo.mdqArg;
	std::array<float, 2> Iab = mMotorInfo.mIab;
	std::array<float, 2> Idq = MotorMath::parkTransform<float>(dqArg, Iab);
	mMotorInfo.mIdq = Idq;
}

//optional function
void MotorCtrl::parkabtogd(void) {
	fp_rad gdArg = mMotorInfo.mgdArg;
	std::array<float, 2> Iab = mMotorInfo.mIab;
	std::array<float, 2> Igd = MotorMath::parkTransform(gdArg, Iab);
	mMotorInfo.mIgd = Igd;
}

std::array<float, 2> MotorCtrl::getIdq() {
	return mMotorInfo.mIdq;
}

std::array<float, 2> MotorCtrl::getIgd() {
	return mMotorInfo.mIgd;
}


void MotorCtrl::ObserverTask() {
	if(mControlMode == OpenLoop || mControlMode == OpenLoopToFOC) {
		//Observer
		//オブザーバセット・計算・値取得
		mObserver.SetIGanmaDelta(mMotorInfo.mIgd);
		mObserver.SetVGanmaDelta(mMotorInfo.mVgd);
		mObserver.CalculateOpenLoop( mArgCtrl.getArgOmega() ,mMotorInfo.mgdArg );//強制転流中はこっち。

		//MotorInfoへ情報の格納
		mMotorInfo.mArgErr = mObserver.GetEstAxiErr(); //軸誤差。gdとdqの差。本来はこの情報だけでドライブできる。
		mMotorInfo.mEstOmega = mObserver.GetEstOmegaE();//デバッグ用
		mMotorInfo.mEstTheta = mObserver.GetEstTheta();//デバッグ用

	}else if(mControlMode == FOC) {
		//Observer
		//オブザーバセット・計算・値取得
		mObserver.SetIGanmaDelta(mMotorInfo.mIgd);
		mObserver.SetVGanmaDelta(mMotorInfo.mVgd);
		mObserver.Calculate();//ベクトル制御用

		//MotorInfoへ情報の格納
		mMotorInfo.mArgErr = mObserver.GetEstAxiErr(); //軸誤差。gdとdqの差。本来はこの情報だけでドライブできる。
		mMotorInfo.mEstOmega = mObserver.GetEstOmegaE();//デバッグ用
		mMotorInfo.mEstTheta = mObserver.GetEstTheta();//デバッグ用

	}else if(mControlMode == FOC_Convolution) {
		//高周波重畳位置推定
		float HFomega = mMotorInfo.mHighFreqOmega;//基準高周波速度
		float est_omega;
		float est_theta;
		mPhaseSpeedEstimator.Estimate(est_omega, est_theta, HFomega, mMotorInfo.mIgd);
		mMotorInfo.mEstOmega = est_omega;
		mMotorInfo.mEstTheta = est_theta;

		//重畳高周波作成
		std::array<float,2> waves = mPhaseSpeedEstimator.OutputHFV(HFomega);
		mMotorInfo.mSinForConv = waves.at(0);
		mMotorInfo.mCosForConv = waves.at(1);
	}

}


void MotorCtrl::VelocityPIDTask() {
	if(mControlMode == FOC) {
			float velocityTarget = 1500;
			float velErr = velocityTarget - mMotorInfo.mEstOmega;
			mVelocityPID.ErrorUpdate(velErr);
	}
}

void MotorCtrl::CurrentControlTask() {
	//Current Target Setting
	mMotorInfo.mIgdTarget = GetCurrentTarget();

	if(mControlMode == OpenLoop || mControlMode == OpenLoopToFOC) {

		CurrentFeedForwardTask();//OpenLoop Drive

	} else if (mControlMode == FOC) {
		CurrentPITask();//PI Control

	} else if (mControlMode == FOC_Convolution) {
		CurrentPITaskForConvolution();//PI Control

	}

}

std::array<float, 2> MotorCtrl::GetCurrentTarget() {
	//mMotorInfo.mIgdTargetを操作するTask

	std::array<float, 2> IgdTarget = {0, 0};
	float CurrentTargetInput = mMotorInfo.mCurrentTargetInput;

	if(mControlMode == OpenLoop) {

		IgdTarget.at(0) = CurrentTargetInput;//IgTarget [A]
		IgdTarget.at(1) = 0;//IdTarget [A]

		return IgdTarget;

	}else if(mControlMode == OpenLoopToFOC) {//OpenLoopからFOCに切り替わる時に動作するモード

		if(mTransitionCountForOpenToFOC < OPEN_TO_FOC_TRANSITION_COUNT_STEP1) {
			IgdTarget.at(0) = CurrentTargetInput * (OPEN_TO_FOC_TRANSITION_COUNT_STEP1 - mTransitionCountForOpenToFOC) / OPEN_TO_FOC_TRANSITION_COUNT_STEP1;
			IgdTarget.at(1) = CurrentTargetInput * mTransitionCountForOpenToFOC / OPEN_TO_FOC_TRANSITION_COUNT_STEP1;
			mTransitionCountForOpenToFOC++;
			return IgdTarget;
		} else {
			IgdTarget.at(0) = CurrentTargetInput * (OPEN_TO_FOC_TRANSITION_COUNT_STEP1 - mTransitionCountForOpenToFOC) / OPEN_TO_FOC_TRANSITION_COUNT_STEP1;
			IgdTarget.at(1) = CurrentTargetInput * mTransitionCountForOpenToFOC / OPEN_TO_FOC_TRANSITION_COUNT_STEP1;
			mTransitionCountForOpenToFOC2++;
			return IgdTarget;
		}

	}else if(mControlMode == FOC) {//FOCのときの入力
		IgdTarget.at(0) = 0;
		IgdTarget.at(1) = CurrentTargetInput/5;


		if(VEL_CONTROL) { //ここおかしい。これはHanlderが持つべき仕事であって、電流指令決定が持つべきものではない
			if(mFOCcount>30000){
				IgdTarget.at(1) = mVelocityPID.OutPut();
			} else {
			mFOCcount++;
			}
		}

		return IgdTarget;

	}else if(mControlMode == FOC_Convolution) {
//		IgdTarget.at(0) = CurrentTargetInput;//Idcに指令電流いれてみる。
//		IgdTarget.at(1) = 0;//IqcTarget [A]

		if(HF_ARG_ZERO_FIX) {
			IgdTarget.at(0) = CurrentTargetInput*4;//Idcに指令電流いれてみる。
			IgdTarget.at(1) = 0;//IqcTarget [A]
			return IgdTarget;
		}

		IgdTarget.at(0) = 0;//Idcに指令電流いれてみる。
		IgdTarget.at(1) = CurrentTargetInput*5;//IqcTarget [A]


		return IgdTarget;
	}

}

void MotorCtrl::CurrentFeedForwardTask() {
	//作成途中 OpenLoop用のFeedForward制御をする。

	//モータの電圧方程式を参考に、出力電流から出力電圧を計算する
	float IgTarget = mMotorInfo.mIgdTarget.at(0);
	float IdTarget = mMotorInfo.mIgdTarget.at(1);

	float R = M_PARAM_R;
	float Ld = M_PARAM_LD;
	float Lq = M_PARAM_LQ;
	float Ke = M_PARAM_PHY;

	float omega = mArgCtrl.getArgOmega();

	float Vganma = R * IgTarget - omega * Lq * IdTarget;
	float Vdelta = R * IdTarget - omega * Ld * IgTarget + Ke * omega;

	setVgd({Vganma, Vdelta});
}

void MotorCtrl::CurrentPITask() {
	//PI Control Start
	mMotorInfo.mIgdErr.at(0) = mMotorInfo.mIgdTarget.at(0) - mMotorInfo.mIgd.at(0);
	mMotorInfo.mIgdErr.at(1) = mMotorInfo.mIgdTarget.at(1) - mMotorInfo.mIgd.at(1);
	std::array<float,2> CulcVgd = PIDgd_control(mMotorInfo.mIgdErr);

	setVgd(CulcVgd);
}


void MotorCtrl::CurrentPITaskForConvolution() {

	//PI Control Start
	mMotorInfo.mIgdErr.at(0) = mMotorInfo.mIgdTarget.at(0) - mMotorInfo.mIgd.at(0);
	mMotorInfo.mIgdErr.at(1) = mMotorInfo.mIgdTarget.at(1) - mMotorInfo.mIgd.at(1);

	//帯域計算が済んでないから無効化
	std::array<float,2> CulcVgd= {0,0};
	//std::array<float,2> CulcVgd = {0,0};//PIDgd_control(mMotorInfo.mIgdErr);


	if(HF_ARG_ZERO_FIX) {
		CulcVgd = {mMotorInfo.mCurrentTargetInput,0};
	}

	CulcVgd = {0,mMotorInfo.mCurrentTargetInput*3};
	//CulcVgd = PIDgd_control(mMotorInfo.mIgdErr);//PI

	//float CurrentTargetInput = mMotorInfo.mCurrentTargetInput;

	//ここで重畳させる
	float VgConv = 0;
	float VdConv = 0;
	if(mControlMode == FOC_Convolution) {
		VgConv = mMotorInfo.mVh * mMotorInfo.mCosForConv;
		VdConv = mMotorInfo.mVh * mMotorInfo.mSinForConv;
	}
	CulcVgd.at(0) = CulcVgd.at(0) + VgConv;
	CulcVgd.at(1) = CulcVgd.at(1) + VdConv;
	setVgd(CulcVgd);
}


std::array<float, 2> MotorCtrl::PIDgd_control(const std::array<float, 2> &pErrIgd) {
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




void MotorCtrl::setVgd(std::array<float, 2> pVgd) {
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
	pVgd.at(0) = Vganma;
	pVgd.at(1) = Vdelta;
	mMotorInfo.mVgd = pVgd;
}


//optional function
void MotorCtrl::invParkgdtoab(void) {
	std::array<float, 2> Vgd = mMotorInfo.mVgd;
	fp_rad gdArg = mMotorInfo.mgdArg;
	std::array<float, 2> Vab = MotorMath::InvparkTransform(gdArg, Vgd);
	mMotorInfo.mVab = Vab;
}


void MotorCtrl::invParkGanmaDelta(void) {
	std::array<float, 2> Vgd = mMotorInfo.mVgd;
	fp_rad ArgErr = mMotorInfo.mArgErr;
	fp_rad InvArgErr = -1.0f * ArgErr;
	std::array<float, 2> Vdq = MotorMath::InvparkTransform(InvArgErr, Vgd);
	mMotorInfo.mVdq = Vdq;
}


void MotorCtrl::invParkTransform(void) {
	fp_rad dqArg = mMotorInfo.mdqArg;
	std::array<float, 2> Vdq= mMotorInfo.mVdq;
	std::array<float, 2> Vab = MotorMath::InvparkTransform(dqArg, Vdq);
	mMotorInfo.mVab = Vab;
}


//void MotorCtrl::invClarkTransform(void) {
//	std::array<float, 2> Vab = {mMotorInfo.mVab.at(0)/mMotorInfo.mVoltageVCC,
//											  mMotorInfo.mVab.at(1)/mMotorInfo.mVoltageVCC};
//	std::array<float, 3> Vuvw = MotorMath::InvclarkTransform(Vab);
//	mMotorInfo.mDutyuvw = Vuvw;
//}


void MotorCtrl::SVM(void) {
	mMotorInfo.mDutyuvw = MotorMath::SVM(mMotorInfo.mVab, mMotorInfo.mVoltageVCC);
	//mMotorInfo.mDutyuvw = MotorMath::InvclarkTransform(mMotorInfo.mVab, mMotorInfo.mVoltageVCC);
}


void MotorCtrl::VoltageOutputTask(void) {
	//0~1のDutyをPWMで出力する。
	//0 <= mMotorInfo.mDuty <= 1

	//svm
	TIMCtrl::floatDuty_ch1(mMotorInfo.mDutyuvw.at(0));
	TIMCtrl::floatDuty_ch2(mMotorInfo.mDutyuvw.at(1));
	TIMCtrl::floatDuty_ch3(mMotorInfo.mDutyuvw.at(2));

	//clark
//	TIMCtrl::MotorDuty_ch1(mMotorInfo.mDutyuvw.at(0));
//	TIMCtrl::MotorDuty_ch2(mMotorInfo.mDutyuvw.at(1));
//	TIMCtrl::MotorDuty_ch3(mMotorInfo.mDutyuvw.at(2));

}


void MotorCtrl::ControlModeHandler() { //状態遷移を管理する関数
	float OpenLoopOmega = mArgCtrl.getArgOmega();
	float ObserverOmega = mObserver.GetEstOmegaE();

	float HFOmega = 0;//mSineWaveConvCalculator.GetEstOmegaE();

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
	case FOC_Convolution:
		break;
	default:
		break;
	}


	//Debug or SPI
	if(mDEBUGorSPI == Debug) {//交互に入れ替える
		mDEBUGorSPI = SPI;
	} else {
		mDEBUGorSPI = Debug;
	}

}

void MotorCtrl::GPIODebugTask() {//Lチカでタイミングをオシロで見る
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);//Board to Driver pin2
	asm("NOP");
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
}


//UICtrl
void MotorCtrl::BtnAct(void){//強制転流開始へのトリガ 割り込みから叩くためにここでラッパする
	if(mUIStatus.mStartStopTRG){
		BtnActOFF();
	} else {
		BtnActON();
	}
}


void MotorCtrl::BtnActOFF(void){//強制転流開始へのトリガOFF
	mUIStatus.mStartStopTRG = MotorStop;
	mControlMode = OpenLoop;
}


void MotorCtrl::BtnActON(void){//強制転流開始へのトリガON
	mUIStatus.mStartStopTRG = MotorStart;
}

