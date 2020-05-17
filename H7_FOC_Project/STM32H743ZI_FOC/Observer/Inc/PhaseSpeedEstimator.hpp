/*
 * PhaseSpeedEstimator.hpp
 *
 *  Created on: 2020/05/17
 *      Author: watashi
 *
 *      依存メモ
 *      "PhaseSpeedEstimator.hpp"
 *      |
 *      |--"MirrorPhaseEstimator.hpp"
 *      |  |--"DModuleFilter.hpp"
 *      |  |  |--"DInverse.hpp"
 *      |--"PhaseSynchronizer.hpp"
 *      |  |--"DiscreteTimeIntegrator.hpp"
 *      |--"HighFreqVoltageCommander.hpp"
 *      |  |--"DiscreteTimeIntegrator.hpp"
 *
 *
 */

#ifndef INC_PHASESPEEDESTIMATOR_HPP_
#define INC_PHASESPEEDESTIMATOR_HPP_

#include "MirrorPhaseEstimator.hpp"
#include "PhaseSynchronizer.hpp"
#include "HighFreqVoltageCommander.hpp"

template <typename T> class PhaseSpeedEstimator {
private:
	//オブザーバ

	//鏡相推定器
	MirrorPhaseEstimator<T> mMirrorPhaseEstimator;

	//PLL
	PhaseSynchronizer<T> mPhaseSynchronizer;

	//高周波生成器
	HighFreqVoltageCommander<T> mHFVC;

public:
	PhaseSpeedEstimator(T pControllerSamplingTime, T pIntegratorGain,
			T pMirrorPhaseEstimator_A0, T pMirrorPhaseEstimator_A1,
			T pMirrorPhaseEstimator_B0, T pMirrorPhaseEstimator_B1, T pMirrorPhaseEstimator_B2,
			T pPhaseSynchronizer_Cn0, T pPhaseSynchronizer_Cn1)
	: mMirrorPhaseEstimator( pControllerSamplingTime,
			pMirrorPhaseEstimator_A0, pMirrorPhaseEstimator_A1,
			pMirrorPhaseEstimator_B0, pMirrorPhaseEstimator_B1, pMirrorPhaseEstimator_B2 )
	, mPhaseSynchronizer(pIntegratorGain,
			pControllerSamplingTime,
			pPhaseSynchronizer_Cn0,
			pPhaseSynchronizer_Cn1 )
	, mHFVC(pIntegratorGain, pControllerSamplingTime)

	{

	}
	~PhaseSpeedEstimator(){};

	void Estimate(T &pEstOmega, T &pEstTheta, const T &pHighFreqOmega, const std::array<T, 2> &pIgd) {
		std::array<T, 2> CS2 = mMirrorPhaseEstimator.Calculate(pHighFreqOmega, pIgd);
		T deltaTheta = atan2(CS2[1], CS2[0])/2;
		T estOmega;
		T estTheta;
		mPhaseSynchronizer.Calculate(estOmega, estTheta, deltaTheta);
		pEstOmega = estOmega;
		pEstTheta = estTheta;
	};

	std::array<T, 2> OutputHFV(const T &pTargetHFOmega) {
		return mHFVC.OutputWaves(pTargetHFOmega);
	}

};

#endif /* INC_PHASESPEEDESTIMATOR_HPP_ */
