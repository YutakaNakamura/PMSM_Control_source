/*
 * MirrorPhaseEstimator.hpp
 *
 *  Created on: 2020/05/09
 *      Author: watashi
 */

#ifndef INC_MIRRORPHASEESTIMATOR_HPP_
#define INC_MIRRORPHASEESTIMATOR_HPP_

#include "DModuleFilter.hpp"

template <typename T> class MirrorPhaseEstimator {
private:
	DModuleFilter<T> mFilterP;
	DModuleFilter<T> mFilterN;

public:
	MirrorPhaseEstimator( T pSamplingTime, T pA0, T pA1, T pB0, T pB1, T pB2)
		: mFilterN( pSamplingTime, pA0, pA1, pB0, pB1, pB2 )
		, mFilterP( pSamplingTime, pA0, pA1, pB0, pB1, pB2 )
	{

	};

	~MirrorPhaseEstimator(){};

	constexpr std::array<T, 2> Calculate(const T &pOmega, const std::array<T, 2> &pInput) {


		std::array<T, 2> xp = mFilterN.Calculate( -pOmega, pInput );
		std::array<T, 2> xn = mFilterP.Calculate( pOmega, pInput );

		//Calculate [xp , Jxp] xn
		//		{
		//		   xp[0], -xp[1]
		//		   xp[1], xp[0]
		//		}
		//		*
		//		{
		//			xn[0],
		//			xn[1]
		//		}
		std::array<T, 2> CS2 = { xp[0] * xn[0] -  xp[1] * xn[1]
											, xp[1] * xn[0] + xp[0] * xn[1] };

		return CS2;
	};

};

#endif /* INC_MIRRORPHASEESTIMATOR_HPP_ */
