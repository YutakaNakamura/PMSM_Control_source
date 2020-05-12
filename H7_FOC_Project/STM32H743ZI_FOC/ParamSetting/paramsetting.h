/*
 * paramsetting.h
 *
 *  Created on: Jul 17, 2019
 *      Author: watashi
 */

#ifndef PARAMSETTING_H_
#define PARAMSETTING_H_

/*
 * パラメータセッティング
 * 大元のインクルードファイル
 * マクロを利用して、動作モードの一括変更をする。
 *注意　演算が入るものは（）をつけること。
 */

//PWMセッティング
#include "pwmparamsetting.h"

//SPIセッティング
#define SPI_DATA_SIZE 16
#define SPI_TIMEOUT 100

//デバッグ設定
#define DEBUG_MODE 1 //デバッグモード これが0だとデバッグ出力なし

//ボード上のシャント抵抗・電流センサ用のセッティング
#include "boardsetting.h"

//モータのパラメータ設定
#include "motorparamsetting.h"


//制御器,サンプリング周波数
#define CONTROLLER_SAMPLING_FREQ PWM_FREQ_HZ
#define CONTROLLER_SAMPLING_TIME (1.0f/(float)CONTROLLER_SAMPLING_FREQ )

//LPFパラメータ
#define HF_LPF_CUTOFF_FREQ 44
#define HF_LPF_SAMPLING_FREQ PWM_FREQ_HZ
#define HF_LPF_CUTOFF_TIME (1.0f /(2* M_PI * (float)HF_LPF_CUTOFF_FREQ) )
#define HF_LPF_SAMPLING_TIME (1.0f/(float)HF_LPF_SAMPLING_FREQ )
#define HF_LPF_DENOMINATOR (2*HF_LPF_CUTOFF_TIME + HF_LPF_SAMPLING_TIME)

#define HF_LPF_B0 HF_LPF_SAMPLING_TIME/HF_LPF_DENOMINATOR
#define HF_LPF_B1 HF_LPF_B0
#define HF_LPF_A1 (HF_LPF_SAMPLING_TIME - 2*HF_LPF_CUTOFF_TIME)/HF_LPF_DENOMINATOR

//位相推定器パラメータ
#define HF_PII_GAIN_K1 3
#define HF_PII_GAIN_K2 (250 * OBSERVER_GAIN_ALPHA)
#define HF_PII_GAIN_K3 (OBSERVER_GAIN_ALPHA * OBSERVER_GAIN_ALPHA)

//強制的にdg軸を0と仮定し続ける。測定用モード
#define HF_ARG_ZERO_FIX 0

//電流PI制御器
#include "currentpisetting.h"


//速度PID制御器
#include "speedpidsetting.h"

//オブザーバ位相推定器セッティング
#include "observerparamsetting.h"

//高周波重畳パラメータ-正弦波バージョン
#define SINEWAVE_CONVOLUTION 1
#include "sinewaveconvsetting.h"


//OpenLoopから変更開始の加速度
#define OPENLOOP_END_OMEGA 1000
//OpenLoop から FOCに切り替えるまでのステップ数
#define OPEN_TO_FOC_TRANSITION_COUNT_STEP1 10000
#define OPEN_TO_FOC_TRANSITION_COUNT_STEP2 10000

//強制転流の設定
//#define FC_TARGET_RPM 1000
//#define FC_TARGET_ACCEL_OMEGA 0.3f
//#define FC_TARGET_ACCEL (float)FC_TARGET_ACCEL_OMEGA/CONTROL_FREQ_HZ

#define OPENLOOP_TARGET_RPS 7*10 //秒間200周
#define OPENLOOP_TARGET_ACCEL 0.1f/50000


#define ENCODER_ABZ 1
#define ENCODER_PERIOD (2000 - 1)


//測定
#define MEASURE_HIGH_FREQ_RESOLUTION 1000

#endif /* PARAMSETTING_H_ */
