/*
 * PID.cpp
 *
 *  Created on: Jul 12, 2019
 *      Author: watashi
 */

/*
 * せつめい！
 * 速度型PID制御をするためのClass
 *
 * コンストラクタでパラメータをセットして、
 * あとはErrorUpdateを叩いた後に、
 * OutPut()でFloatを取り出せば速度型のPID制御が簡単にできるはず。
 */

#include "PID.hpp"

PID::PID(){

}

PID::PID(float pSampleTime, float pGain_p, float pGain_i, float pGain_d)
:mSampleTime(pSampleTime), mGain_p(pGain_p), mGain_i(pGain_i), mGain_d(pGain_d)
{
	// TODO Auto-generated constructor stub
	mOutput = 0;
	mOutValOfLast = 0;
	mError.fill(0);
	mDiff = 0;
}

PID::~PID() {
	// TODO Auto-generated destructor stub
}

void PID::SetParam(float pGain_p, float pGain_i, float pGain_d) {
	mGain_p = pGain_p;
	mGain_i = pGain_i;
	mGain_d = pGain_d;
}

void PID::ErrorUpdate(const float &pError) {
//	mError[1] = pError;
//	mError[2] = mError[1];
//	mError[3] = mError[2];
	mError = {pError,mError.at(0),mError.at(1)};
}

void PID::SetSampleTime(float pSampleTime) {
	mSampleTime = pSampleTime;
}

float PID::OutPut(void){
	mOutValOfLast = mOutput;//前回値の保持
	mDiff = mGain_p*( mError.at(0)-mError.at(1)//P
			          +(mSampleTime/mGain_i)*mError.at(0)//I
					  +(mGain_d/mSampleTime)*(mError.at(0)-2*mError.at(1)+mError.at(2)) );//D
	mOutput = mOutValOfLast + mDiff;
	return mOutput;
}
