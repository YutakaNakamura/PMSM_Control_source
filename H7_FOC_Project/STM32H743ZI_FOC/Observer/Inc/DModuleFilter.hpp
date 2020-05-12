/*
 * DModuleFilter.hpp
 *
 *  Created on: 2020/05/09
 *      Author: watashi
 */

#ifndef INC_DMODULEFILTER_HPP_
#define INC_DMODULEFILTER_HPP_

#include <array>
#include "DInverse.hpp"

template <typename T> class DModuleFilter {
private:
	DInverse<T> DInverse1;
	DInverse<T> DInverse2;

	T mA0;
	T mA1;

	T mB0;
	T mB1;
	T mB2;

	//output * Z^-1
	std::array<T, 2> mOutputZn1;

public:
	DModuleFilter(T pSamplingTime, T pA0, T pA1, T pB0, T pB1, T pB2)
	: DInverse1(pSamplingTime)
	, DInverse2(pSamplingTime)
	, mA0(pA0), mA1(pA1)
	, mB0(pB0), mB1(pB1), mB2(pB2)
{
		mOutputZn1 = {(T)0, (T)0};
};
	~DModuleFilter(){};

	constexpr std::array<T, 2> Calculate(const T &pOmega, const std::array<T, 2> &pInput) {

		//input * B0 - output*Z^-1 *A0
		std::array<T, 2> diff0 = { pInput[0] * mB0 - mOutputZn1[0] * mA0,
												pInput[1] * mB0 - mOutputZn1[1] * mA0 };

		//diff0*DInv
		std::array<T, 2> dout0 = DInverse1.Calculate(pOmega, diff0);

		//input * B1 + dout0 - output*Z^-1 *A0
		std::array<T, 2> diff1 = { pInput[0] * mB1 + dout0[0] - mOutputZn1[0] * mA1,
												pInput[1] * mB1 + dout0[1] - mOutputZn1[1] * mA1 };

		//diff1*DInv
		std::array<T, 2> dout1 = DInverse2.Calculate(pOmega, diff1);

		//input * B2 + dout1
		std::array<T, 2> output = { pInput[0] * mB2 + dout1[0],
													pInput[1] * mB2 + dout1[1] };

		//Update Value
		mOutputZn1 = output;
		return output;
	};


	constexpr std::array<T, 2> TestCalculate(const T &pOmega, const std::array<T, 2> &pInput) {

		//input * B0 - output*Z^-1 *A0
		std::array<T, 2> diff0 = { pInput[0] * mB0 - mOutputZn1[0] * mA0,
												pInput[1] * mB0 - mOutputZn1[1] * mA0 };

		//diff0*DInv
		std::array<T, 2> dout0 = DInverse1.Calculate(pOmega, diff0);

//		//input * B1 + dout0 - output*Z^-1 *A0
//		std::array<T, 2> diff1 = { pInput[0] * mB1 + dout0[0] - mOutputZn1[0] * mA1,
//												pInput[1] * mB1 + dout0[1] - mOutputZn1[1] * mA1 };
//
//		//diff1*DInv
//		std::array<T, 2> dout1 = DInverse2.Calculate(pOmega, diff1);

		//input * B2 + dout1
		std::array<T, 2> output = { pInput[0] * mB1 + dout0[0],
													pInput[1] * mB1 + dout0[1] };

		//Update Value
		mOutputZn1 = output;
		return output;
	};



};

#endif /* INC_DMODULEFILTER_HPP_ */
