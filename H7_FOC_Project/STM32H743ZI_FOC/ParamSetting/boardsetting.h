/*
 * boardsetting.h
 *
 *  Created on: Jul 17, 2019
 *      Author: watashi
 */

#ifndef BOARDSETTING_H_
#define BOARDSETTING_H_

/*
 * ボードセッティング
 *
 * ここではマクロを利用して、動作モードの一括変更をする。
 *
 *注意　演算が入るものは（）をつけること。
 */

//ボードの I -> V 倍率
//(Rの配置及び計算式についてはPDF参照)
#define BOARD_OFFSET_VOLTAGE 1.65f
#define BOARD_OPAMP_R1 2700
#define BOARD_OPAMP_R2 33000
#define BOARD_OPAMP_R3 2700
#define BOARD_OPAMP_R4 33000
//#define BOARD_SHUNT_R 0.040f
#define BOARD_SHUNT_R 0.005f

#define BOARD_IV_RATIO ( ( 1.0f / (float)BOARD_SHUNT_R ) * ( (float)BOARD_OPAMP_R1 / ( (float)BOARD_OPAMP_R1 + (float)BOARD_OPAMP_R2 ) ) * ( ( (float)BOARD_OPAMP_R3 + (float)BOARD_OPAMP_R4 ) / (float)BOARD_OPAMP_R4 ) )
#define BOARD_IV_OFFSET ( -1.0f * ( 1.0f / (float)BOARD_SHUNT_R ) * ( (float)BOARD_OPAMP_R3 / (float)BOARD_OPAMP_R4 ) * (float)BOARD_OFFSET_VOLTAGE )

//VCC Voltage
#define VCC_VOLTAGE 12

//ADC Value -> Voltage倍率
#define ADC_VOLTAGE_RATIO ( 3.3f / (float)65535 )



#endif /* BOARDSETTING_H_ */
