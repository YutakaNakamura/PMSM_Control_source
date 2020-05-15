/*
 * MotorInfo.hpp
 *
 *  Created on: Nov 8, 2019
 *      Author: watashi
 */

#ifndef SRC_MOTORINFO_HPP_
#define SRC_MOTORINFO_HPP_

#include "MotorLibDefPack.hpp"
#include <array>

template <typename T> class MotorInfo {//モータの情報を格納するClass
private:
	std::array<T, 3> mIuvw;//単位[A]
public:


	std::array<T, 2> mIab;//単位[A]
	std::array<T, 2> mIdq;//単位[A]
	std::array<T, 2> mIgd; //単位[A] ganmadelta

	std::array<T, 3> mVuvw;//単位[V]
	std::array<T, 2> mVab;//単位[V]
	std::array<T, 2> mVdq;//単位[V]
	std::array<T, 2> mVgd; //単位[V] ganmadelta

	std::array<T, 2> mIgdTarget;//
	std::array<T, 2> mIgdErr;
	T mVoltageVCC;

	//spiの指令値バッファ
	T mCurrentTargetInput = 0;

	//高周波重畳用変数
	T mVh = 0;
	T mVoffset = 0;

	T mHighFreqOmega = 0;
	T mSinForConv = 0;
	T mCosForConv = 0;

	//高周波重畳デバッグ変数
	std::array<T, 2> mConvIdqc = {0,0};
	std::array<T, 2> mIdqch = {0,0};

	std::array<T, 3> mDutyuvw;//単位[%]
	T mdqArg;//単位[rad]
	T mgdArg;//単位[rad]
	T mArgErr;//単位[rad]

	T mEstTheta;//単位[rad]
	T mEstOmega;//単位[rad/s]

//	fp_rad mEstOmega_Observer;//単位[rad/s]
//	fp_rad mEstOmega_HF;//単位[rad/s]

public:
	void setIuvw(const std::array<T, 3> &pIuvw) {
		mIuvw = pIuvw;
	};

	std::array<T, 3> getIuvw(void) const{
		return mIuvw;
	}

};

#endif /* SRC_MOTORINFO_HPP_ */
