/*
 * DInverse.hpp
 *
 *  Created on: 2020/05/09
 *      Author: watashi
 */

#ifndef INC_DINVERSE_HPP_
#define INC_DINVERSE_HPP_

#include <array>

template <typename T> class DInverse {
private:

	//SamplingTIme.
	T mSamplingTime;

	//input * Z^-1
	std::array<T, 2> mInputZn1;

	//output * Z^-1
	std::array<T, 2> mOutputZn1;

public:

	constexpr DInverse(T pSamplingTime)
	: mSamplingTime(pSamplingTime)
	{
		mInputZn1 = {(T)0, (T)0};
		mOutputZn1 = {(T)0, (T)0};
	};
	~DInverse(){};

	constexpr std::array<T, 2> Calculate(const T &pOmega, const std::array<T, 2> &pInput) {
		//ωT,ωT/2,(ωT/2)^2
		T omegaT = mSamplingTime * pOmega;
		T halfOmegaT = (T)0.5 * omegaT;
		T squareHalfOmegaT = halfOmegaT * halfOmegaT;

		//feedforward calc
		std::array<T, 2> add = { pInput[0] + mInputZn1[0]
										   , pInput[1] + mInputZn1[1]};

		std::array<T, 2>  gainMultiplied = { mSamplingTime * (T)0.5 * add[0]
															  , mSamplingTime * (T)0.5 * add[1] };

		std::array<T, 2> feedforwardDiff = { gainMultiplied[0] + halfOmegaT * gainMultiplied[1]
															  , gainMultiplied[1] - halfOmegaT * gainMultiplied[0] };

		//feedback calc
		std::array<T, 2> feedback = { ( (T)1.0-squareHalfOmegaT ) * mOutputZn1[0] + omegaT * mOutputZn1[1]
												    , ( (T)1.0-squareHalfOmegaT) * mOutputZn1[1] - omegaT * mOutputZn1[0] };

		std::array<T, 2> feedbackDiff = { feedforwardDiff[0] + feedback[0]
														 , feedforwardDiff[1] + feedback[1] };

		//output calc
		std::array<T, 2> output = { feedbackDiff[0] * (T)1.0/( (T)1.0+squareHalfOmegaT )
												, feedbackDiff[1] * (T)1.0/( (T)1.0+squareHalfOmegaT ) };

		//Update Value
		mInputZn1 = pInput;
		mOutputZn1 = output;
		return output;
	};

};


//template <typename T> class DInverse {
//private:
//
//	//Integrator
//	DiscreteTimeIntegrator<T> mIntegrator0;
//	DiscreteTimeIntegrator<T> mIntegrator1;
//
//	//output * Z^-1
//	std::array<T, 2> mOutputZn1;
//
//public:
//
//	constexpr DInverse(T pGain,T pSamplingTime)
//	: mIntegrator0(pGain,pSamplingTime)
//	, mIntegrator1(pGain,pSamplingTime)
//	{
//		mOutputZn1 = {(T)0, (T)0};
//	};
//	~DInverse(){};
//
//	constexpr std::array<T, 2> Calculate(const T &pOmega, const std::array<T, 2> &pInput) {
//
//		//feedback = ω * Jy
//		std::array<T, 2> feedback = { mOutputZn1[1] * -pOmega, mOutputZn1[0] * pOmega };
//
//		//diff = u - (ω * Jy)
//		std::array<T, 2> diff = { (pInput[0] - feedback[0]), (pInput[1] - feedback[1]) };
//
//		//output = Σdiff
//		std::array<T, 2> output = { mIntegrator0.Integrate(diff[0]), mIntegrator1.Integrate(diff[1]) };
//
//		//Update Value
//		mOutputZn1 = output;
//		return output;
//	};
//
//};


#endif /* INC_DINVERSE_HPP_ */
