/*
 * MotorMeasure.hpp
 *
 *  Created on: Dec 14, 2019
 *      Author: watashi
 */

#ifndef SRC_MOTORMEASURE_HPP_
#define SRC_MOTORMEASURE_HPP_

#include "MotorInterface.hpp"

class MotorMeasure {
public:

	enum ControlMode { //強制転流、FOCなどの動作モード管理
		OpenLoop = 1,
		OpenLoopToFOC,
		FOC,
	};

	enum MeasureTarget { //測定する対象
		Ldd = 1,
		Lqd,
		Ldq,
		Lqq,
		R,
	};


private:
	MotorInfo<float> mMotorInfo;
	Observer mObserver;

	PID mIdPID, mIqPID;
	PID mIganmaPID, mIdeltaPID;

	PID mVelocityPID;

	ArgCtrl mArgCtrl;

	ControlMode mControlMode;
	MeasureTarget mMeasureTarget;

	UIStatus mUIStatus;
	DebugCtrl mDebugCtrl;

	WaveGenerator mWaveGen;//パラメータ測定用
public:
	MotorMeasure();
	virtual ~MotorMeasure();

	void InitSystem(void);
	void DeInitSystem(void);
	void InitMotorControl(void);
	void InitObserver(void);

	void HighFreqTask(void);

	void Measure(void);
	//void MotorDrive(void);
	//MotorDrive内呼び出し関数
	void SPITask(void);
	void ReadCurrentTask();
	void setIuvw(float pIu, float pIv, float pIw);
	void ReadVoltageTask();

	void ReadAngleTask(void);
	fp_rad GetAngleForOpenLoop();
	fp_rad GetAngleForFOC();

	void clarkTransform(void);
	void parkTransform(void);
	void parkGanmaDelta(void);
	void parkabtogd(void);

	//void ObserverTask();

	void CurrentControlTask();
	std::array<float, 2> GetCurrentTarget();
	//void CurrentFeedForwardTask();
	void CurrentPITask();
	std::array<float, 2> PIDgd_control(std::array<float, 2> pErrIgd);//どちらかが死にclassになるけど毎回呼ぶ作業でif文使いたくない。
	void setVg(float pVg);
	void setVd(float pVd);
	void setVgd(std::array<float, 2> pVgd);

	std::array<float, 2> getIdq(void);
	std::array<float, 2> getIgd(void);

	void invParkgdtoab(void);
	void invParkGanmaDelta(void);
	void invParkTransform(void);
	void invClarkTransform(void);
	void SVM(void);

	void VoltageOutputTask(void);

	//void VelocityPIDTask();
	void GPIODebugTask();

	//Handler
	void ControlModeHandler();

	//UI
	void BtnAct(void);
	void BtnActOFF(void);
	void BtnActON(void);

	void ChangeFreq(void);

private:
	int mlogcount = 0;
	int mTransitionCountForOpenToFOC = 0;
	int mTransitionCountForOpenToFOC2 = 0;

	int mFOCcount=0;

	int mFreqMode = 0;

};

#endif /* SRC_MOTORMEASURE_HPP_ */
