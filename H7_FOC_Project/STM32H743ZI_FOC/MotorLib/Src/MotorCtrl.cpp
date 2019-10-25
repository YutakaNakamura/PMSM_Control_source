/*
 * MotorCtrl.cpp
 *
 *  Created on: Aug 5, 2019
 *      Author: watashi
 */

#include "MotorCtrl.hpp"

MotorCtrl::MotorCtrl() {
	// TODO Auto-generated constructor stub

}

MotorCtrl::~MotorCtrl() {
	// TODO Auto-generated destructor stub
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
	//TIMCtrl::MotorDuty_ch4(0.9);//9割タイミングで打つ
	TIMCtrl::TIM1SetCOMP_ch4(PWM_PERIOD_COUNT - 1);

	//ENABLE信号
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);

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


void MotorCtrl::InitMotorControl(void) {
	mOperationMode = Drive;//起動時、動作をまずは運転にする。
	mControlMode = OpenLoop;//起動時、動作をまずは強制転流にする。

	mArgCtrl.Init();

	mIganmaPID.SetParam(PID_GAIN_IGANMA_P, PID_GAIN_IGANMA_I, PID_GAIN_IGANMA_D);
	mIdeltaPID.SetParam(PID_GAIN_IDELTA_P, PID_GAIN_IDELTA_I, PID_GAIN_IDELTA_D);
	mIganmaPID.SetSampleTime(PID_CYCLE_TIME);
	mIdeltaPID.SetSampleTime(PID_CYCLE_TIME);

	mVelocityPID.SetParam(PID_GAIN_VEL_P, PID_GAIN_VEL_I, PID_GAIN_VEL_D);
	mVelocityPID.SetSampleTime(PID_CYCLE_TIME_VEL);
}

void MotorCtrl::InitObserver(void) {
	mObserver.InitEMFObs(OBSERVER_CYCLE_TIME, M_PARAM_R, M_PARAM_LD, M_PARAM_LQ, OBSERVER_GAIN_ALPHA);
	mObserver.InitPII2(OBSERVER_CYCLE_TIME, OBSERVER_GAIN_K1, OBSERVER_GAIN_K2, OBSERVER_GAIN_K3);
}



void MotorCtrl::HighFreqTask(void) {

	if( mOperationMode == Drive ) {
		MotorDrive();
	} else if( mOperationMode == Measure ) {
		//Measure();未実装
	}

}

void MotorCtrl::MotorDrive(void) { //モータを動かすモード.他に測定モードを用意する予定

	GPIODebugTask();//GPIOからオシロに波形を出力する

	//開始直後にADC2を読み取って、変換時間を演算処理の中で相殺する。
	ADCCtrl::ADC2Start_Conversion();
	//ADCCtrl::ADC2Conversion_wait(10);
	ReadCurrentTask();
	ReadVoltageTask();

	ReadAngle(); //mgdArgを取得する。(OpenLoop or FOC分岐あり)

	//Iuvw -> Iab
	clarkTransform();
	//Iab -> Igd
	parkabtogd();//mgdArgで回転

	GPIODebugTask();//GPIOからオシロに波形を出力する

	ObserverTask();//オブザーバ
	VelocityPIDTask();//速度PID制御
	CurrentPITask();//電流PI制御

	GPIODebugTask();//GPIOからオシロに波形を出力する

	//Vgd　->　Vab
	invParkgdtoab();
	//Vab -> Vuvw
	SVM();

	MotorOutputTaskSVM();//PWM出力

	ControlModeHandler();//次の運転制御モードを決める関数

	//DEBUG
	GPIODebugTask();//GPIOからオシロに波形を出力する

	if(DEBUG_MODE){//デバッグモードで入る処理
		JLinkDebug();
	}
	GPIODebugTask();//GPIOからオシロに波形を出力する
}


void MotorCtrl::ReadCurrentTask(void) {
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
	setIuvw(Iu, Iv, Iw);
}


void MotorCtrl::ReadVoltageTask() {
	//ReadVoltage
	//電圧測定(入力)
	mMotorInfo.mVoltageVCC = VCC_VOLTAGE;
}


//Motor
void MotorCtrl::setIuvw(float pIu, float pIv, float pIw){
	mMotorInfo.mIuvw.at(0) = pIu;
	mMotorInfo.mIuvw.at(1) = pIv;
	mMotorInfo.mIuvw.at(2) = pIw;
}


void MotorCtrl::AngleTaskForOpenLoop(void) {
	//強制転流では、gd軸を回転させる。
	if(mUIStatus.mStartStopTRG) {
	mArgCtrl.accelerationForOpenLoop();
	} else {
	mArgCtrl.decelerationForOpenLoop();
	}
	mMotorInfo.mgdArg = mArgCtrl.getArg(); //gd軸のみ回す。
}


void MotorCtrl::AngleTaskForFOC(void) {
	mMotorInfo.mgdArg = mObserver.GetEstTheta();//オブザーバから読み取る
}


void MotorCtrl::ReadAngle(void) {
	if(mControlMode == OpenLoop) {
		AngleTaskForOpenLoop();
	}else if(mControlMode == OpenLoopToFOC) {
		AngleTaskForOpenLoop();
	}else if(mControlMode == FOC) {
		AngleTaskForFOC();
	}
}


void MotorCtrl::clarkTransform(void) {
	std::array<float, 3> Iuvw = {mMotorInfo.mIuvw.at(0),
											   mMotorInfo.mIuvw.at(1),
											   -mMotorInfo.mIuvw.at(0) - mMotorInfo.mIuvw.at(1)};
	std::array<float, 2> Iab = MotorMath::clarkTransform(Iuvw);
	mMotorInfo.mIab = Iab;
}

void MotorCtrl::parkTransform(void) {
	fp_rad dqArg = mMotorInfo.mdqArg;
	std::array<float, 2> Iab = mMotorInfo.mIab;
	std::array<float, 2> Idq = MotorMath::parkTransform(dqArg, Iab);
	mMotorInfo.mIdq = Idq;
}

void MotorCtrl::parkGanmaDelta(void) {
	fp_rad ArgErr = mMotorInfo.mArgErr;
	std::array<float, 2> Idq = mMotorInfo.mIdq;
	fp_rad InvArgErr = -1.0f * ArgErr;
	std::array<float, 2> Igd = MotorMath::parkTransform(InvArgErr, Idq);
	mMotorInfo.mIgd = Igd;
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
	if(mControlMode == OpenLoop) {
		//Observer
		//オブザーバセット・計算・値取得
		mObserver.SetIGanmaDelta(mMotorInfo.mIgd);
		mObserver.SetVGanmaDelta(mMotorInfo.mVgd);
		mObserver.CalculateOpenLoop( mArgCtrl.getArgOmega() ,mMotorInfo.mgdArg );//強制転流中はこっち。

		float EstAxiErr = mObserver.GetEstAxiErr();//軸誤差。gdとdqの差。
		mMotorInfo.mArgErr = EstAxiErr;


	}else if(mControlMode == OpenLoopToFOC){
		//Observer
		//オブザーバセット・計算・値取得
		mObserver.SetIGanmaDelta(mMotorInfo.mIgd);
		mObserver.SetVGanmaDelta(mMotorInfo.mVgd);
		mObserver.CalculateOpenLoop( mArgCtrl.getArgOmega() ,mMotorInfo.mgdArg );//強制転流中はこっち。

		float EstAxiErr = mObserver.GetEstAxiErr();//軸誤差。gdとdqの差。
		mMotorInfo.mArgErr = EstAxiErr;



	}else if(mControlMode == FOC) {
		//Observer
		//オブザーバセット・計算・値取得
		mObserver.SetIGanmaDelta(mMotorInfo.mIgd);
		mObserver.SetVGanmaDelta(mMotorInfo.mVgd);
		mObserver.Calculate();//ベクトル制御用
		float EstAxiErr = mObserver.GetEstAxiErr();//軸誤差。gdとdqの差。
		mMotorInfo.mArgErr = EstAxiErr;
	}

}


void MotorCtrl::VelocityPIDTask() {
	if(mControlMode == FOC) {
			float adc2_input = (float)ADCCtrl::ADC2_Read() / 65535;
			//float velocityTarget = adc2_input * 1000;
			float velocityTarget = 1500;
			float velErr = velocityTarget - mObserver.GetEstOmegaE();
			mVelocityPID.ErrorUpdate(velErr);
	}
}


void MotorCtrl::CurrentPITask() {
	//Current Target Setting
	float adc2_input = (float)ADCCtrl::ADC2_Read() / 65535;

	if(mControlMode == OpenLoop) {

		mMotorInfo.mIgdTarget.at(0) = adc2_input;//IgTarget [A]
		mMotorInfo.mIgdTarget.at(1) = 0;//IdTarget [A]

	}else if(mControlMode == OpenLoopToFOC) {//OpenLoopからFOCに切り替わる時に動作するモード

		if(mTransitionCountForOpenToFOC < OPEN_TO_FOC_TRANSITION_COUNT_STEP1) {
			mMotorInfo.mIgdTarget.at(0) = adc2_input * (OPEN_TO_FOC_TRANSITION_COUNT_STEP1 - mTransitionCountForOpenToFOC) / OPEN_TO_FOC_TRANSITION_COUNT_STEP1;
			mMotorInfo.mIgdTarget.at(1) = adc2_input * mTransitionCountForOpenToFOC / OPEN_TO_FOC_TRANSITION_COUNT_STEP1;
			mTransitionCountForOpenToFOC++;
		} else {
			mMotorInfo.mIgdTarget.at(0) = adc2_input * (OPEN_TO_FOC_TRANSITION_COUNT_STEP1 - mTransitionCountForOpenToFOC) / OPEN_TO_FOC_TRANSITION_COUNT_STEP1;
			mMotorInfo.mIgdTarget.at(1) = adc2_input * mTransitionCountForOpenToFOC / OPEN_TO_FOC_TRANSITION_COUNT_STEP1;
			mTransitionCountForOpenToFOC2++;
		}

	}else if(mControlMode == FOC) {//FOCのときの入力
		mMotorInfo.mIgdTarget.at(0) = 0;
		//mMotorInfo.mIgdTarget.at(1) = adc2_input;
		mMotorInfo.mIgdTarget.at(1) = adc2_input/5;
		if(VEL_CONTROL) {
			if(mFOCcount>30000){

				mMotorInfo.mIgdTarget.at(1) = mVelocityPID.OutPut();
			} else {
			mFOCcount++;
			}

		}
	}

	//PI Control Start
	mMotorInfo.mIgdErr.at(0) = mMotorInfo.mIgdTarget.at(0) - mMotorInfo.mIgd.at(0);
	mMotorInfo.mIgdErr.at(1) = mMotorInfo.mIgdTarget.at(1) - mMotorInfo.mIgd.at(1);
	PIDgd_control(mMotorInfo.mIgdErr);

}


void MotorCtrl::PIDdq_control(std::array<float, 2> pErrIdq) {
	float ErrId = pErrIdq.at(0);
	float ErrIq = pErrIdq.at(1);
	mIdPID.ErrorUpdate(ErrId);
	mIqPID.ErrorUpdate(ErrIq);

	float Vd = mMotorInfo.mVdq.at(0);
	float Vq = mMotorInfo.mVdq.at(1);

	Vd = Vd + mIdPID.OutPut();
	Vq = Vq + mIqPID.OutPut();
	mMotorInfo.mVdq = {Vd, Vq};
}


void MotorCtrl::PIDgd_control(std::array<float, 2> pErrIgd) {
	{
		float ErrIganma = pErrIgd.at(0);
		float ErrIdelta = pErrIgd.at(1);
		mIganmaPID.ErrorUpdate(ErrIganma);
		mIdeltaPID.ErrorUpdate(ErrIdelta);

		float Vganma = mMotorInfo.mVgd.at(0);
		float Vdelta = mMotorInfo.mVgd.at(1);

		Vganma = mIganmaPID.OutPut();
		Vdelta = mIdeltaPID.OutPut();

		//LIMITを入れる
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

		mMotorInfo.mVgd = {Vganma, Vdelta};

	}
}


void MotorCtrl::setVdq(std::array<float, 2> pVdq) {
	//強制転流用
	mMotorInfo.mVdq = pVdq;
}


void MotorCtrl::setVgd(std::array<float, 2> pVgd) {
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


void MotorCtrl::invClarkTransform(void) {
	std::array<float, 2> Vab = {mMotorInfo.mVab.at(0)/mMotorInfo.mVoltageVCC,
											  mMotorInfo.mVab.at(1)/mMotorInfo.mVoltageVCC};
	std::array<float, 3> Vuvw = MotorMath::InvclarkTransform(Vab);
	mMotorInfo.mDutyuvw = Vuvw;
}


void MotorCtrl::SVM(void) {
	mMotorInfo.mDutyuvw = MotorMath::SVM(mMotorInfo.mVab, mMotorInfo.mVoltageVCC);
}


void MotorCtrl::MotorOutputTask(void) {
	TIMCtrl::MotorDuty_ch1(mMotorInfo.mDutyuvw.at(0));
	TIMCtrl::MotorDuty_ch2(mMotorInfo.mDutyuvw.at(1));
	TIMCtrl::MotorDuty_ch3(mMotorInfo.mDutyuvw.at(2));
}


void MotorCtrl::MotorOutputTaskSVM(void) {
	TIMCtrl::floatDuty_ch1(mMotorInfo.mDutyuvw.at(0));
	TIMCtrl::floatDuty_ch2(mMotorInfo.mDutyuvw.at(1));
	TIMCtrl::floatDuty_ch3(mMotorInfo.mDutyuvw.at(2));
}


void MotorCtrl::ControlModeHandler() { //状態遷移を管理する関数
	float OpenLoopOmega = mArgCtrl.getArgOmega();
	float ObserverOmega = mObserver.GetEstOmegaE();

	//OpenLoopからの遷移ハンドル
	if(mControlMode == OpenLoop) {
		if(OpenLoopOmega > OPENLOOP_END_OMEGA) {
			mControlMode = OpenLoopToFOC;
		}
		return;
	}

	//OpenLoopToFOCからの遷移ハンドル
	if(mControlMode == OpenLoopToFOC) {
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
		return;
	}

	//FOCからの遷移ハンドル
	if(mControlMode == FOC) {
		if(200 > ObserverOmega ){ //定常状態は400
			mControlMode = OpenLoopToFOC;
		}
		return;
	}

}

void MotorCtrl::GPIODebugTask() {//Lチカでタイミングをオシロで見る
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	asm("NOP");
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
}


void MotorCtrl::JLinkDebug() {

	mDebugC++;

	if(mDebugC >= 2) {
		mDebugC = 0;
		return;
	}

	int milIu = (int)( mMotorInfo.mIuvw.at(0) * 1000 );
	int milIv = (int)( mMotorInfo.mIuvw.at(1) * 1000 );
	int milIw = (int)( mMotorInfo.mIuvw.at(2) * 1000 );
	int DegArg = (int)(mMotorInfo.mgdArg/M_PI * 180 );//指令値の角度
	int DegAxiErr =(int)( mObserver.GetEstAxiErr() / M_PI *180 );
	int milEstOmega =(int)( mObserver.GetEstOmegaE());
	int EstTheta =(int)(mObserver.GetEstTheta()/M_PI * 180 );

	int milIa = (int)( mMotorInfo.mIab.at(0) * 1000 );
	int milIb = (int)( mMotorInfo.mIab.at(1) * 1000 );

	int milIg = (int)( mMotorInfo.mIgd.at(0) * 1000 );
	int milId = (int)( mMotorInfo.mIgd.at(1) * 1000 );

	int milVg = (int)( mMotorInfo.mVgd.at(0) * 1000 );
	int milVd = (int)( mMotorInfo.mVgd.at(1) * 1000 );

	//SVMdebug
	int milVu = (int)(mMotorInfo.mDutyuvw.at(0) * 1000 );
	int milVv = (int)(mMotorInfo.mDutyuvw.at(1) * 1000 );
	int milVw = (int)(mMotorInfo.mDutyuvw.at(2) * 1000 );

	int milIgTarget = (int)(mMotorInfo.mIgdTarget.at(0)*1000);

	int milEstEMFg = (int)(mObserver.GetEstEMFgd().at(0) * 1000);
	int milEstEMFd = (int)(mObserver.GetEstEMFgd().at(1) * 1000);

	//encoder
	//int encoder = (int)(EncoderABZCtrl::GetAngle()*(360.0f/(ENCODER_PERIOD+1)));

	char outputStr[100]={0};//100文字までとりあえず静的確保
	//general
	//sprintf(outputStr,"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n" ,mlogcount, milIgTarget, milVg, milVd, milIg, milId, DegArg, DegAxiErr, milEstOmega, EstTheta);//みやゆうさんご希望のデバッグ

	//velocity
	sprintf(outputStr,"%d,%d,%d,%d,%d,%d,%d\n" ,mlogcount, milVg, milVd, milIg, milId, milEstOmega, EstTheta);

	//encoder
	//sprintf(outputStr,"%d\n",encoder);

	if( !mUIStatus.mStartStopTRG ) {//加速してるときだけ入る Printf
		return;
	}

	SEGGER_RTT_WriteString(0,outputStr);
	//printf("%s" ,outputStr);

	mlogcount++;
	if(	mlogcount > 65535){
		mlogcount=0;
	}

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




////////////////func of debug ///////////////////
void MotorCtrl::DbgUart(std::string pStr) {
	//UART::Transmit(pStr);
}


//Debug
void MotorCtrl::DebugTask(float pIu, float pIv, float pIw, float pArg){
	int sw = mDebug.GetDbgStatus();
	switch(sw){
	case 0:
		//if(mMotorInfo.mSensor.GetArgCount() > 24000){
		mDebug.DbgInfoTinyRegister(pIu, pIv, pIw, pArg);
		//mDebug.DbgInfoRegister(pIu, pIv, pIw, pArg);
		//}
		break;
	case 1:
		//止める動作が必要だと思う
		MotorCtrl::BtnActOFF();
		mDebug.SetDebugStatus(2);
		break;
	case 2:
		//止まるのを確認したら次にすすめる
		//if(mMotorInfo.mSensor.GetArgCount() < 10){
			mDebug.SetDebugStatus(3);
		//}
		break;
	case 3:
		mDebug.PrintStatusTiny();
		//mDebug.PrintStatus();
		HAL_Delay(1);
		mDebug.SetDebugStatus(0);
//		//こんな感じで状態遷移の動作をさせればいいのではないでしょうか。
		break;
	default :
		//例外は何もしない
		break;
	}
}
