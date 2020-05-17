/*
 * PID.hpp
 *
 *  Created on: Jul 12, 2019
 *      Author: watashi
 */

#ifndef PID_HPP_
#define PID_HPP_

#include <array>

class PID {
private:
	float mSampleTime;
	float mGain_p;
	float mGain_i;
	float mGain_d;

	std::array<float, 3> mError;
	float mOutput;
	float mOutValOfLast;
	float mDiff;

public:
	PID();
	PID(float pSampleTime, float pGain_p, float pGain_i, float pGain_d);
	virtual ~PID();
	void SetParam(float pGain_p, float pGain_i, float pGain_d);
	void ErrorUpdate(const float &pError);
	void SetSampleTime(float pSampleTime);
	float OutPut(void);
};

#endif /* PID_HPP_ */
