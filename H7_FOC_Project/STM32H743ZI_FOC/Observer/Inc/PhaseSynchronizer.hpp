/*
 * PhaseSynchronizer.hpp
 *
 *  Created on: 2020/05/10
 *      Author: watashi
 */

#ifndef INC_PHASESYNCHRONIZER_HPP_
#define INC_PHASESYNCHRONIZER_HPP_

#include <cmath>
#include "DiscreteTimeIntegrator.hpp"

template <typename T> class PhaseSynchronizer {
private:


	DiscreteTimeIntegrator<T> mIntegratorforFilter;
	DiscreteTimeIntegrator<T> mIntegratorforAngle;

	T mCn0;
	T mCn1;

public:
	PhaseSynchronizer(T pGain, T pSamplingTime, T pCn0, T pCn1)
		:mIntegratorforFilter(pGain,pSamplingTime)
		,mIntegratorforAngle(pGain,pSamplingTime)
	{
		mCn0 = pCn0;
		mCn1 = pCn1;
	};
	~PhaseSynchronizer(){};

	constexpr void Calculate(T &pEstOmega, T &pEstTheta, const T &pInput) {

		T integrated = mIntegratorforFilter.Integrate(pInput);

		T omega = pInput * mCn1 + integrated * mCn0;

		T theta = mIntegratorforAngle.Integrate2PiMod(omega);

		//0 - 2PIに収める
		theta = theta + (T)2.0 * M_PI;
		theta = std::fmod(theta, (T)2.0 * M_PI);

		pEstOmega = omega;
		pEstTheta = theta;
	};



};

#endif /* INC_PHASESYNCHRONIZER_HPP_ */
