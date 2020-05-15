/*
 * MotorMath.hpp
 *
 *  Created on: Sep 2, 2019
 *      Author: watashi
 */

#ifndef INC_MOTORMATH_HPP_
#define INC_MOTORMATH_HPP_

#include "MotorLibDefPack.hpp"
#include <array>
#include "Matrix.hpp"
#include "Trigonometric.hpp"

namespace MotorMath {

//static inline std::array<float, 2> clarkTransform (const std::array<float, 3> &pVector);
//	sinline std::array<float, 2> parkTransform( const fp_rad &pRadian,
//												const std::array<float, 2> &pVector);
//
//	static inline std::array<float, 2> InvparkTransform( const fp_rad &pRadian,
//												   const std::array<float, 2> &pVector);
//	static inline std::array<float, 3> InvclarkTransform (const std::array<float, 2> &pVector);
//	static inline std::array<float, 3> SVM(const std::array<float, 2> &pVector, const float &pVoltageVCC);
//
//	static inline fp_rad AngleDiff(const fp_rad &pAnglePlus, const fp_rad &pAngleMinus);
//};

//input [u,v,w]
//output [a,b]
template <typename T>
static inline std::array<T, 2> clarkTransform (const std::array<T, 3> &pVector) {
	//mIalpha = mIu - (mIv + mIw)/2;
	//mIbeta = (mIv - mIw)* 1.7320508f/2;
	return { ( 0.81649658 * ( pVector.at(0) - ((pVector.at(1) + pVector.at(2))/2)  ) ),
				  ( 0.81649658 * ( (pVector.at(1) - pVector.at(2)) * 1.7320508/2 ) ) };
};

template <typename T>
static inline std::array<T, 2> parkTransform(const T &pRadian, const std::array<T, 2> &pVector) {
	T sinVal = Trigonometric::sin(pRadian);
	T cosVal = Trigonometric::cos(pRadian);
	T Invsin = -1.0 * sinVal;

	return { ( cosVal * pVector.at(0) + sinVal * pVector.at(1) ),
				  ( Invsin * pVector.at(0) + cosVal * pVector.at(1) ) };

}

static inline std::array<float, 2> InvparkTransform(const fp_rad &pRadian, const std::array<float, 2> &pVector) {
	//mId =  mLib.getCosList().at(mArg) * mIalpha + mLib.getSinList().at(mArg) * mIbeta;
	//mIq = -mLib.getSinList().at(mArg) * mIalpha + mLib.getCosList().at(mArg) * mIbeta;
	float sinVal = Trigonometric::sin(pRadian);
	float cosVal = Trigonometric::cos(pRadian);
	float Invsin = -1.0f * sinVal;

	return { ( cosVal * pVector.at(0) + Invsin * pVector.at(1) ),
				  ( sinVal * pVector.at(0) + cosVal * pVector.at(1) ) };
}

//input [a,b]
//output [u,v,w]
static inline std::array<float, 3> InvclarkTransform (const std::array<float, 2> &pVector,  const float &pVoltageVCC) {
	//mVu = 0.75f * mValpha;
	//mVv = -0.75f * mValpha + mValpha / 3 + mVbeta / 1.7320508f;
	//mVw = - mValpha / 3 - mVbeta / 1.7320508f;

	float avec = pVector.at(0)/pVoltageVCC;
	float bvec = pVector.at(1)/pVoltageVCC;

	std::array<float ,3> uvw;
	uvw.at(0) = 0.75f * avec;
	uvw.at(1) = -0.75f * avec + avec / 3 + bvec / 1.7320508f;
	uvw.at(2) = - avec / 3 - bvec / 1.7320508f;
	uvw.at(0) = uvw.at(0) * 0.81649658f;//√2/3
	uvw.at(1) = uvw.at(1)  * 0.81649658f;//√2/3
	uvw.at(2) = uvw.at(2) * 0.81649658f;//√2/3
	return uvw;
};

//input [a,b],VCC
//output Duty[u,v,w]
static inline std::array<float, 3> SVM (const std::array<float, 2> &pVector, const float &pVoltageVCC) {
	float Va = pVector.at(0);
	float Vb = pVector.at(1);

	bool sector0 = ((Va*Va)>(Vb*Vb / 3)) && (Va >= 0) && (Vb >= 0);
	bool sector5 = ((Va*Va)>(Vb*Vb / 3)) && (Va>0) && (Vb<0);

	bool sector1 = ((Va*Va) <= (Vb*Vb / 3)) && (Vb >= 0);//Va=Vb=0
	bool sector4 = ((Va*Va) <= (Vb*Vb / 3)) && (Vb<0);

	bool sector2 = ((Va*Va)>(Vb*Vb / 3)) && (Va<0) && (Vb>0);
	bool sector3 = ((Va*Va)>(Vb*Vb / 3)) && (Va <= 0) && (Vb <= 0);

	int svmflag = ( sector0 | sector1<<1 | sector2 <<2 | sector3 <<3 | sector4 <<4 | sector5 <<5 );

	float cot60 = 1/sqrt(3);
	float cosec60 = 2/sqrt(3);
	float coefficient = sqrt(3.0f/2.0f);
	float Du, Dv, Dw;
	float D0, D1, D2, D3, D4, D5, D6, D7;
	switch (svmflag) {
	case 1://sector0
		D1 = coefficient * (Va - cot60 * Vb) / 12;
		D2 = coefficient * (cosec60 * Vb) / 12;
		D0 = (1 - (D1 + D2)) / 2;
		D7 = (1 - (D1 + D2)) / 2;

		Du = D1 + D2 + D7;
		Dv = D2 + D7;
		Dw = D7;
		break;
	case 2://sector1
		D3 = coefficient * (-Va + cot60 * Vb) / pVoltageVCC;
		D2 = coefficient * (Va + cot60 * Vb) / pVoltageVCC;
		D0 = (1 - (D2 + D3)) / 2;
		D7 = (1 - (D2 + D3)) / 2;
		Du = D2 + D7;
		Dv = D2 + D3 + D7;
		Dw = D7;
		break;
	case 4://sector2
		D3 = coefficient * (cosec60 * Vb) / pVoltageVCC;
		D4 = coefficient * (-Va - cot60 * Vb) / pVoltageVCC;
		D0 = (1 - (D3 + D4)) / 2;
		D7 = (1 - (D3 + D4)) / 2;
		Du = D7;
		Dv = D3 + D4 + D7;
		Dw = D4 + D7;
		break;
	case 8://sector3
		D5 = coefficient * (-cosec60 * Vb) / pVoltageVCC;
		D4 = coefficient * (-Va + cot60 * Vb) / pVoltageVCC;
		D0 = (1 - (D4 + D5)) / 2;
		D7 = (1 - (D4 + D5)) / 2;
		Du = D7;
		Dv = D4 + D7;
		Dw = D4 + D5 + D7;
		break;
	case 16://sector4
		D5 = coefficient * (-Va - cot60 * Vb) / pVoltageVCC;
		D6 = coefficient * (Va - cot60 * Vb) / pVoltageVCC;
		D0 = (1 - (D5 + D6)) / 2;
		D7 = (1 - (D5 + D6)) / 2;
		Du = D6 + D7;
		Dv = D7;
		Dw = D5 + D6 + D7;
		break;
	case 32://sector5
		D1 = coefficient * (Va + cot60 * Vb) / pVoltageVCC;
		D6 = coefficient * (-cosec60 * Vb) / pVoltageVCC;
		D0 = (1 - (D1 + D6)) / 2;
		D7 = (1 - (D1 + D6)) / 2;
		Du = D1 + D6 + D7;
		Dv = D7;
		Dw = D6 + D7;
		break;
	default:
		Du = 0.5f;
		Dv = 0.5f;
		Dw = 0.5f;
		break;
	}

	return {Du,Dv,Dw};
};


//input[Angle+, Angle-]
//output[Angle+ - Angle-]
//+10が出力の時
//Nominal 30 20
//Carry    5 355
//が存在する。
//Nominal:diff=10, diff_carry=390-20=370,diff_borrow=30-(380)=-350
//Carry  :diff=-350,diff_carry=365-355=10,diff_borrow=5-(715)=-710
//min(diff^2,diff_carry^2)が答えとなる。
//
//-20が出力の時
//Nominal 20 40
//Borrow 350 10
//Nominal:diff_carry=380-40=340 ,diff=-20,diff_borrow=20-(400)=-380
//Borrow:diff_carry=710-10=700 ,diff=340, diff_borrow=350-(370)=-20
static inline fp_rad AngleDiff(const fp_rad &pAnglePlus, const fp_rad &pAngleMinus) {
	fp_rad diff = pAnglePlus - pAngleMinus;
	fp_rad diff_carry = pAnglePlus + 2*M_PI - pAngleMinus;
	fp_rad diff_borrow = pAnglePlus - (pAngleMinus + 2*M_PI);

	float diff2 = diff * diff;
	float diff_carry2 = diff_carry * diff_carry;
	float diff_borrow2 = diff_borrow * diff_borrow;

	if(diff2 < diff_carry2) {//diffかborrow
		if(diff2 < diff_borrow2){
			return diff;
		}else{
			return diff_borrow;
		}
	}else{//carryかborrow
		if(diff_carry2 < diff_borrow2){
			return diff_carry;
		}else{
			return diff_borrow;
		}
	}

}

}
#endif /* INC_MOTORMATH_HPP_ */
