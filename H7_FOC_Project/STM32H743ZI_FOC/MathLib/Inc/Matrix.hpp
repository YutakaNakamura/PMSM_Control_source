/*
 * Matrix.hpp
 *
 *  Created on: Aug 31, 2019
 *      Author: watashi
 *
 *      宣言無しで使えるクラス。
 *      行列の和・積などは、内部に変数を持たないから、
 *      全てPublic Staticで返せばよいよね。
 */

#ifndef INC_MATRIX_HPP_
#define INC_MATRIX_HPP_

#include <array>
#include "math.h"

namespace Matrix {

//	static inline std::array<float, 2> MatrixMultiple2x2(const float &pa11, const float &pa12,
//								  const float &pa21, const float &pa22,
//								  const std::array<float, 2> &pVector);
//	static inline std::array<float, 2> VectorMultiple2x1(const float &pGain, const std::array<float, 2> &pVector);
//	static inline std::array<float, 2> VectorAdd2x1(const std::array<float, 2> &pVector1, const std::array<float, 2> &pVector2);//2個のベクトルの足し算
//	static inline std::array<float, 2> VectorAdd2x1(const std::array<float, 2> &pVector1, const std::array<float, 2> &pVector2, const std::array<float, 2> &pVector3);//3個の足し算
//	static inline std::array<float, 2> VectorSubtract2x1(const std::array<float, 2> &pVector1, const std::array<float, 2> &pVector2);//2個のベクトルの引き算,pVec1-pVec2
//
//
//	static inline float VectorMultiple1x2_2x1(const std::array<float, 2> &pVector1, const std::array<float, 2> &pVector2);

	//Inline function

	static inline std::array<float, 2> MatrixMultiple2x2(const float &pa11, const float &pa12,
		                    				const float &pa21, const float &pa22,
											const std::array<float, 2> &pVector) {

		return{ ( pa11 * pVector.at(0) + pa12 * pVector.at(1) ),
					 ( pa21 * pVector.at(0) + pa22 * pVector.at(1) ) };
	};

	static inline std::array<float, 2> VectorMultiple2x1(const float &pGain, const std::array<float, 2> &pVector) {

		return { ( pGain * pVector.at(0) ),
					  ( pGain * pVector.at(1) )  };
	};

	static inline std::array<float, 2> VectorAdd2x1(const std::array<float, 2> &pVector1, const std::array<float, 2> &pVector2) {

		return { ( pVector1.at(0) + pVector2.at(0) ),
					  ( pVector1.at(1) + pVector2.at(1) ) };

	};

	static inline std::array<float, 2> VectorAdd2x1(const std::array<float, 2> &pVector1, const std::array<float, 2> &pVector2, const std::array<float, 2> &pVector3) {

	return { ( pVector1.at(0) + pVector2.at(0) + pVector3.at(0) ),
				  ( pVector1.at(1) + pVector2.at(1) + pVector3.at(1) ) };
	};

	static inline std::array<float, 2> VectorSubtract2x1(const std::array<float, 2> &pVector1, const std::array<float, 2> &pVector2) {

		return { ( pVector1.at(0) - pVector2.at(0) ),
					  ( pVector1.at(1) - pVector2.at(1) ) };
	};

	static inline float VectorMultiple1x2_2x1(const std::array<float, 2> &pVector1, const std::array<float, 2> &pVector2) {

		return ( pVector1.at(0) * pVector2.at(0) + pVector1.at(1) * pVector2.at(1) );
	};




};

////Inline function
//
//inline std::array<float, 2> Matrix::MatrixMultiple2x2(const float &pa11, const float &pa12,
//	                    				const float &pa21, const float &pa22,
//										const std::array<float, 2> &pVector) {
//
//	return{ ( pa11 * pVector.at(0) + pa12 * pVector.at(1) ),
//				 ( pa21 * pVector.at(0) + pa22 * pVector.at(1) ) };
//}
//
//inline std::array<float, 2> Matrix::VectorMultiple2x1(const float &pGain, const std::array<float, 2> &pVector) {
//
//	return { ( pGain * pVector.at(0) ),
//				  ( pGain * pVector.at(1) )  };
//}
//
//inline std::array<float, 2> Matrix::VectorAdd2x1(const std::array<float, 2> &pVector1, const std::array<float, 2> &pVector2) {
//
//	return { ( pVector1.at(0) + pVector2.at(0) ),
//				  ( pVector1.at(1) + pVector2.at(1) ) };
//
//}
//
//inline std::array<float, 2> Matrix::VectorAdd2x1(const std::array<float, 2> &pVector1, const std::array<float, 2> &pVector2, const std::array<float, 2> &pVector3) {
//
//return { ( pVector1.at(0) + pVector2.at(0) + pVector3.at(0) ),
//			  ( pVector1.at(1) + pVector2.at(1) + pVector3.at(1) ) };
//}
//
//inline std::array<float, 2> Matrix::VectorSubtract2x1(const std::array<float, 2> &pVector1, const std::array<float, 2> &pVector2) {
//
//	return { ( pVector1.at(0) - pVector2.at(0) ),
//				  ( pVector1.at(1) - pVector2.at(1) ) };
//}
//
//inline float Matrix::VectorMultiple1x2_2x1(const std::array<float, 2> &pVector1, const std::array<float, 2> &pVector2) {
//
//	return ( pVector1.at(0) * pVector2.at(0) + pVector1.at(1) * pVector2.at(1) );
//}

#endif /* INC_MATRIX_HPP_ */
