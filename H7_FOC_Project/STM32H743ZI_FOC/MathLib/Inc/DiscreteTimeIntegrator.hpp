/*
 * DiscreteTimeIntegrator.hpp
 *
 *  Created on: 2020/05/09
 *      Author: watashi
 *
 *
 *      離散積分ブロックのテンプレートクラス
 *
 */

#ifndef DISCRETETIMEINTEGRATOR_HPP_
#define DISCRETETIMEINTEGRATOR_HPP_

#include <cmath>

template <typename T> class DiscreteTimeIntegrator {
private:

	//Gain and SamplingTIme.
	T mGain;
	T mSamplingTime;

	//Old Value.
	// mInputZn1 = input * Z^-1
	// mOutputZn1 = output * Z^-1
	T mInputZn1;
	T mOutputZn1;

public:

	constexpr DiscreteTimeIntegrator(T pGain,T pSamplingTime) : mGain(pGain),mSamplingTime(pSamplingTime)
	{
		mOutputZn1 = (T)0;
		mInputZn1 = (T)0;
	};
	~DiscreteTimeIntegrator(){};

	constexpr T Integrate(const T &pInput) {
		T output = mOutputZn1 + ( mSamplingTime * mGain ) * (T)0.5 * (pInput + mInputZn1);
		//Update Value
		mInputZn1 = pInput;
		mOutputZn1 = output;
		return output;
	};

	constexpr T Integrate2PiMod(const T &pInput) {
		T output = mOutputZn1 + ( mSamplingTime * mGain ) * (T)0.5 * (pInput + mInputZn1);
		output = std::fmod(output, (T)2.0 * M_PI);//Modする
		//Update Value
		mInputZn1 = pInput;
		mOutputZn1 = output;
		return output;
	};

	constexpr void Reset() {
		mOutputZn1 = (T)0;
		mInputZn1 = (T)0;
	};

};

#endif /* DISCRETETIMEINTEGRATOR_HPP_ */
