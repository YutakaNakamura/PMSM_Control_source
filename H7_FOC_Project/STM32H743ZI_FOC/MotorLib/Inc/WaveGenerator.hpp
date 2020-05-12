/*
 * WaveGenerator.hpp
 *
 *  Created on: Dec 13, 2019
 *      Author: watashi
 */

#ifndef INC_WAVEGENERATOR_HPP_
#define INC_WAVEGENERATOR_HPP_

#include "paramsetting.h"
#include "Trigonometric.hpp"
#include "MotorLibDefPack.hpp"
#include <array>

class WaveGenerator {
private:



	int mTargetHz;
	int mCount = 0;
	float mThetaPerStep;

	float mOmega;

public:
	WaveGenerator();
	virtual ~WaveGenerator();
	void InitFrequency(int pTargetHz);
	void ResetPhase();
	float OutputWaveform();
	std::array<float, 2> OutputWaves();
};

#endif /* INC_WAVEGENERATOR_HPP_ */
