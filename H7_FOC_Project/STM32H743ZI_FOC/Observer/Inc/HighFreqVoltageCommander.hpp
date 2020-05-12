/*
 * HighFreqVoltageCommander.hpp
 *
 *  Created on: 2020/05/10
 *      Author: watashi
 */

#ifndef INC_HIGHFREQVOLTAGECOMMANDER_HPP_
#define INC_HIGHFREQVOLTAGECOMMANDER_HPP_

#include <array>
#include <cmath>
#include "DiscreteTimeIntegrator.hpp"

template <typename T> class HighFreqVoltageCommander {
private:
	DiscreteTimeIntegrator<T> mIntegrator;

public:
	constexpr HighFreqVoltageCommander(T pGain,T pSamplingTime)
	: mIntegrator(pGain, pSamplingTime)
	{};
	~HighFreqVoltageCommander(){};

	void ResetPhase(){
		mIntegrator.Reset();
	};

	constexpr std::array<T, 2> OutputWaves(T pOmega) {
		T theta = mIntegrator.Integrate2PiMod(pOmega);
		return {sin(theta),cos(theta)};
	};


};

#endif /* INC_HIGHFREQVOLTAGECOMMANDER_HPP_ */
