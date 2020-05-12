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

class MotorInfo {//モータの情報を格納するClass
public:
	std::array<float, 3> mIuvw;//単位[A]
	std::array<float, 2> mIab;//単位[A]
	std::array<float, 2> mIdq;//単位[A]
	std::array<float, 2> mIgd; //単位[A] ganmadelta

	std::array<float, 3> mVuvw;//単位[V]
	std::array<float, 2> mVab;//単位[V]
	std::array<float, 2> mVdq;//単位[V]
	std::array<float, 2> mVgd; //単位[V] ganmadelta

	std::array<float, 2> mIgdTarget;//
	std::array<float, 2> mIgdErr;
	float mVoltageVCC;

	//spiの指令値バッファ
	float mCurrentTargetInput = 0;

	//高周波重畳用変数
	float mVh = 0;
	float mVoffset = 0;

	float mHighFreqOmega = 0;
	float mSinForConv = 0;
	float mCosForConv = 0;

	//高周波重畳デバッグ変数
	std::array<float, 2> mConvIdqc = {0,0};
	std::array<float, 2> mIdqch = {0,0};

	std::array<float, 3> mDutyuvw;//単位[%]
	fp_rad mdqArg;//単位[rad]
	fp_rad mgdArg;//単位[rad]
	fp_rad mArgErr;//単位[rad]

	fp_rad mEstTheta;//単位[rad]
	fp_rad mEstOmega;//単位[rad/s]

//	fp_rad mEstOmega_Observer;//単位[rad/s]
//	fp_rad mEstOmega_HF;//単位[rad/s]



};

#endif /* SRC_MOTORINFO_HPP_ */
