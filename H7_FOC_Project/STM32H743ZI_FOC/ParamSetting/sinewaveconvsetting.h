/*
 * sinewaveconvsetting.h
 *
 *  Created on: Jul 17, 2019
 *      Author: watashi
 *
 *      ここでは正弦波重畳用のマクロを記述する。
 *
 *
 */

#ifndef SINEWAVECONVSETTING_H_
#define SINEWAVECONVSETTING_H_

#include "motorparamsetting.h"

/*
 * パラメータセッティング
 * ここではマクロを利用して、動作モードの一括変更をする。
 *
 *注意　演算が入るものは（）をつけること。
 */

//高周波重畳の推定制御器のパラメータ
#define HF_CONV_FREQ 400

//BPFパラメータ
//#define HF_HPF_BPF_CUTOFF_FREQ 220
//#define HF_HPF_BPF_SAMPLING_FREQ 20000
//#define HF_HPF_BPF_CUTOFF_TIME (1.0f /(2* M_PI * (float)HF_HPF_BPF_CUTOFF_FREQ) )
//#define HF_HPF_BPF_SAMPLING_TIME (1.0f/(float)HF_HPF_BPF_SAMPLING_FREQ )
//#define HF_HPF_BPF_DENOMINATOR (2*HF_HPF_BPF_CUTOFF_TIME + HF_HPF_BPF_SAMPLING_TIME)
//
//#define HF_HPF_BPF_A1 (HF_HPF_BPF_SAMPLING_TIME-2*HF_HPF_BPF_CUTOFF_TIME)/HF_HPF_BPF_DENOMINATOR
//#define HF_HPF_BPF_B0 2*HF_HPF_BPF_CUTOFF_TIME/HF_HPF_BPF_DENOMINATOR
//#define HF_HPF_BPF_B1 HF_HPF_BPF_B0
//
//#define HF_LPF_BPF_CUTOFF_FREQ 220
//#define HF_LPF_BPF_SAMPLING_FREQ 20000
//#define HF_LPF_BPF_CUTOFF_TIME (1.0f /(2* M_PI * (float)HF_LPF_BPF_CUTOFF_FREQ) )
//#define HF_LPF_BPF_SAMPLING_TIME (1.0f/(float)HF_LPF_BPF_SAMPLING_FREQ )
//#define HF_LPF_BPF_DENOMINATOR (2*HF_LPF_BPF_CUTOFF_TIME + HF_LPF_BPF_SAMPLING_TIME)
//
//#define HF_LPF_BPF_B0 HF_LPF_BPF_SAMPLING_TIME/HF_LPF_BPF_DENOMINATOR
//#define HF_LPF_BPF_B1 HF_LPF_BPF_B0
//#define HF_LPF_BPF_A1 (HF_LPF_BPF_SAMPLING_TIME - 2*HF_LPF_BPF_CUTOFF_TIME)/HF_LPF_BPF_DENOMINATOR




#endif
