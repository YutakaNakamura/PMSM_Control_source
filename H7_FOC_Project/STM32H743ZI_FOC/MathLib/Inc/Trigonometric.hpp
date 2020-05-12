/*
 * Trigonometric.hpp
 *
 *  Created on: Aug 31, 2019
 *      Author: watashi
 *
 *
 *  三角関数演算クラス
 *　やっていることはWrapperにすぎない。
 *　別のマイコン・ライブラリへ変更したときに、このClassの演算を変えるだけで、
 *　全ての演算方法を変更できる。
 *
 */

#ifndef INC_TRIGONOMETRIC_HPP_
#define INC_TRIGONOMETRIC_HPP_


#ifdef __FPU_PRESENT
//FPU指定のみ(マイコン)

#include "math.h"
#include "DSPInc.hpp"
namespace Trigonometric {

	static inline float sin(const float &pVal) {
		return arm_sin_f32(pVal);
	};

	static inline float cos(const float &pVal) {
		return arm_cos_f32(pVal);
	}

	static inline float atan2(const float &pNumerator, const float &pDenominator) {
		return atan2f(pNumerator, pDenominator);
	}
};

#else
//それ以外

#include <cmath>
namespace Trigonometric {

	static inline float sin(const float &pVal) {
		return std::sinf(pVal);
	};

	static inline float cos(const float &pVal) {
		return std::cosf(pVal);
	}

	static inline float atan2(const float &pNumerator, const float &pDenominator) {
		return std::atan2f(pNumerator, pDenominator);
	}

	static inline double sin(const double &pVal) {
		return std::sin(pVal);
	};

	static inline double cos(const double &pVal) {
		return std::cos(pVal);
	}

	static inline double atan2(const double &pNumerator, const double &pDenominator) {
		return std::atan2(pNumerator, pDenominator);
	}

};

#endif



#endif /* INC_TRIGONOMETRIC_HPP_ */
