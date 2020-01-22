/*
* paramsetting.h
*
*  Created on: Jul 17, 2019
*      Author: watashi
*/

#ifndef PARAMSETTING_H_
#define PARAMSETTING_H_

/*
* �p�����[�^�Z�b�e�B���O
* �����ł̓}�N���𗘗p���āA���샂�[�h�̈ꊇ�ύX������B
*
*���Ӂ@���Z��������̂́i�j�����邱�ƁB
*/


/*
* �n�[�h�E�F�A�Z�b�e�B���O
*/
//PWM�̐ݒ�
#define PWM_FREQ_HZ 20000
#define TIM_CLOCK_DIVIDER 1
#define TIM_CLK_MHz (480/TIM_CLOCK_DIVIDER)
#define PWM_PERIOD_CYCLES (uint16_t)(TIM_CLK_MHz*(unsigned long long)1000000u/(uint16_t)(PWM_FREQ_HZ))
//#define PWM_PERIOD_COUNT PWM_PERIOD_CYCLES/2�@//�Б���PWM���Ƃ���
#define PWM_PERIOD_COUNT (PWM_PERIOD_CYCLES/4) //���[��PWM���Ƃ���
#define PWM_DEADTIME_COUNT 20

#define SPI_DATA_SIZE 16
#define SPI_TIMEOUT 100

/*
* �\�t�g�E�F�A�Z�b�e�B���O
*/
//�f�o�b�O�ݒ�
#define DEBUG_MODE 1 //�f�o�b�O���[�h
#define DEBUG_COUNT 360 //�f�o�b�O���̃J�E���g
//#define DEBUG_COUNT 720 //�f�o�b�O���̃J�E���g
//#define DEBUG_COUNT 7 //�f�o�b�O���̃J�E���g
#define DEBUG_ADC_TRIG_PWM_OUT 1//ON��TIM1 ch4���o�� ���[�^�͓��삵�Ȃ�


//�{�[�h�� I -> V �{��
//(R�̔z�u�y�ьv�Z���ɂ��Ă�PDF�Q��)
#define BOARD_OFFSET_VOLTAGE 1.65f
#define BOARD_OPAMP_R1 2700
#define BOARD_OPAMP_R2 33000
#define BOARD_OPAMP_R3 2700
#define BOARD_OPAMP_R4 33000
#define BOARD_SHUNT_R 0.040f

#define BOARD_IV_RATIO ( ( 1.0f / (float)BOARD_SHUNT_R ) * ( (float)BOARD_OPAMP_R1 / ( (float)BOARD_OPAMP_R1 + (float)BOARD_OPAMP_R2 ) ) * ( ( (float)BOARD_OPAMP_R3 + (float)BOARD_OPAMP_R4 ) / (float)BOARD_OPAMP_R4 ) )
#define BOARD_IV_OFFSET ( -1.0f * ( 1.0f / (float)BOARD_SHUNT_R ) * ( (float)BOARD_OPAMP_R3 / (float)BOARD_OPAMP_R4 ) * (float)BOARD_OFFSET_VOLTAGE )

//VCC Voltage
#define VCC_VOLTAGE 12

//ADC Value -> Voltage�{��
#define ADC_VOLTAGE_RATIO ( 3.3f / (float)65535 )


//���[�^�̃p�����[�^�ݒ�
//R  = 0.0782;
//Ld = 0.035e-3;
//Lq = 0.07e-3;
//�������� = 6.61798e-3
//�ɑΐ� = 7
#define M_PARAM_LD 0.000035f
#define M_PARAM_LQ 0.00007f
#define M_PARAM_PHY 0.0008f
#define M_PARAM_R 0.006617f

#define OBSERVER_CYCLE_TIME PWM_PERIOD_SEC
#define OBSERVER_GAIN_ALPHA 200
#define OBSERVER_GAIN_K1 2
#define OBSERVER_GAIN_K2 (3 * OBSERVER_GAIN_ALPHA)
#define OBSERVER_GAIN_K3 (OBSERVER_GAIN_ALPHA * OBSERVER_GAIN_ALPHA)

//�����g�d��̐��萧���̃p�����[�^
#define HF_CONV_FREQ 220

#define HF_BPF_CUTOFF_FREQ 220
//#define HF_BPF_SAMPLING_FREQ 20000
#define HF_BPF_SAMPLING_FREQ 100000
#define HF_BPF_CUTOFF_TIME (1.0f /(2* M_PI * (double)HF_BPF_CUTOFF_FREQ) )
#define HF_BPF_CUTOFF_TIME_SQUARED (HF_BPF_CUTOFF_TIME*HF_BPF_CUTOFF_TIME)
#define HF_BPF_SAMPLING_TIME (1.0/(double)HF_BPF_SAMPLING_FREQ )
#define HF_BPF_SAMPLING_TIME_SQUARED (HF_BPF_SAMPLING_TIME*HF_BPF_SAMPLING_TIME)
#define HF_BPF_DENOMINATOR ((2*HF_BPF_CUTOFF_TIME + HF_BPF_SAMPLING_TIME) * (2*HF_BPF_CUTOFF_TIME + HF_BPF_SAMPLING_TIME))

#define HF_BPF_B0 (2 * HF_BPF_CUTOFF_TIME * HF_BPF_SAMPLING_TIME) / HF_BPF_DENOMINATOR
#define HF_BPF_B2 HF_BPF_B0
#define HF_BPF_A1 (2*(HF_BPF_SAMPLING_TIME_SQUARED - 4*HF_BPF_CUTOFF_TIME_SQUARED))/HF_BPF_DENOMINATOR
#define HF_BPF_A2 ((2*HF_BPF_CUTOFF_TIME - HF_BPF_SAMPLING_TIME) * (2*HF_BPF_CUTOFF_TIME - HF_BPF_SAMPLING_TIME))/HF_BPF_DENOMINATOR


//#define HF_HPF_BPF_CUTOFF_FREQ 1000
//#define HF_HPF_BPF_SAMPLING_FREQ 100000

#define HF_HPF_BPF_CUTOFF_FREQ 220
#define HF_HPF_BPF_SAMPLING_FREQ 20000
#define HF_HPF_BPF_CUTOFF_TIME (1.0f /(2* M_PI * (float)HF_HPF_BPF_CUTOFF_FREQ) )
#define HF_HPF_BPF_SAMPLING_TIME (1.0f/(float)HF_HPF_BPF_SAMPLING_FREQ )
#define HF_HPF_BPF_DENOMINATOR (2*HF_HPF_BPF_CUTOFF_TIME + HF_HPF_BPF_SAMPLING_TIME)

#define HF_HPF_BPF_A1 (HF_HPF_BPF_SAMPLING_TIME-2*HF_HPF_BPF_CUTOFF_TIME)/HF_HPF_BPF_DENOMINATOR
#define HF_HPF_BPF_B0 2*HF_HPF_BPF_CUTOFF_TIME/HF_HPF_BPF_DENOMINATOR
#define HF_HPF_BPF_B1 HF_HPF_BPF_B0

//#define HF_LPF_BPF_CUTOFF_FREQ 3000//�Ւf���g����3�{���x�ɂ���Ƃ悢�B10�{���Ɩ�����
//#define HF_LPF_BPF_SAMPLING_FREQ 100000
#define HF_LPF_BPF_CUTOFF_FREQ 220
#define HF_LPF_BPF_SAMPLING_FREQ 20000
#define HF_LPF_BPF_CUTOFF_TIME (1.0f /(2* M_PI * (float)HF_LPF_BPF_CUTOFF_FREQ) )
#define HF_LPF_BPF_SAMPLING_TIME (1.0f/(float)HF_LPF_BPF_SAMPLING_FREQ )
#define HF_LPF_BPF_DENOMINATOR (2*HF_LPF_BPF_CUTOFF_TIME + HF_LPF_BPF_SAMPLING_TIME)

#define HF_LPF_BPF_B0 HF_LPF_BPF_SAMPLING_TIME/HF_LPF_BPF_DENOMINATOR
#define HF_LPF_BPF_B1 HF_LPF_BPF_B0
#define HF_LPF_BPF_A1 (HF_LPF_BPF_SAMPLING_TIME - 2*HF_LPF_BPF_CUTOFF_TIME)/HF_LPF_BPF_DENOMINATOR



#define HF_HETERODYNE_PHASE_OFFSET (float)(0*2*M_PI/360.0f)

#define HF_LPF_CUTOFF_FREQ 44
#define HF_LPF_SAMPLING_FREQ 20000
#define HF_LPF_CUTOFF_TIME (1.0f /(2* M_PI * (float)HF_LPF_CUTOFF_FREQ) )
#define HF_LPF_SAMPLING_TIME (1.0f/(float)HF_LPF_SAMPLING_FREQ )
#define HF_LPF_DENOMINATOR (2*HF_LPF_CUTOFF_TIME + HF_LPF_SAMPLING_TIME)

#define HF_LPF_B0 HF_LPF_SAMPLING_TIME/HF_LPF_DENOMINATOR
#define HF_LPF_B1 HF_LPF_B0
#define HF_LPF_A1 (HF_LPF_SAMPLING_TIME - 2*HF_LPF_CUTOFF_TIME)/HF_LPF_DENOMINATOR

#define HF_PII_GAIN_K1 0
#define HF_PII_GAIN_K2 (3 * OBSERVER_GAIN_ALPHA)
#define HF_PII_GAIN_K3 (OBSERVER_GAIN_ALPHA * OBSERVER_GAIN_ALPHA)

//�����I��dg����0�Ɖ��肵������B����p���[�h
#define HF_ARG_ZERO_FIX 0

//����p����
#define PWM_PERIOD_SEC 1.0f/(float)PWM_FREQ_HZ
//#define CONTROL_FREQ_HZ PWM_FREQ_HZ
#define CONTROL_FREQ_HZ 20000

//PID�����ݒ�

//�������[s]
#define PID_CYCLE_TIME PWM_PERIOD_SEC
#define PID_TIME_CONSTANT_G 1.0f/((float)175*2*3.1415926f)
#define PID_TIME_CONSTANT_D 1.0f/((float)175*2*3.1415926f)

//PI�d������������Ȃ�
#define PI_NOCONTROL_DEBUG 0
////PIdq�����p�����[�^�ݒ�
//#define PID_GAIN_ID_P 0.1
//#define PID_GAIN_ID_I 0.1
//#define PID_GAIN_ID_D 0.1
//#define PID_ID_MAX_VOLTAGE 10
//
//#define PID_GAIN_IQ_P 0.1
//#define PID_GAIN_IQ_I 0.1
//#define PID_GAIN_IQ_D 0.1
//#define PID_IQ_MAX_VOLTAGE 10

//PIgd�����p�����[�^�ݒ�
#define PID_GAIN_IGANMA_P (M_PARAM_LD)/(PID_TIME_CONSTANT_G)
#define PID_GAIN_IGANMA_I (M_PARAM_LD)/(M_PARAM_R)
#define PID_GAIN_IGANMA_D 0
#define PID_IGANMA_MAX_VOLTAGE 9.0f
#define PID_IGANMA_MIN_VOLTAGE -9.0f

#define PID_GAIN_IDELTA_P (M_PARAM_LQ)/(PID_TIME_CONSTANT_D)
#define PID_GAIN_IDELTA_I (M_PARAM_LQ)/(M_PARAM_R)
#define PID_GAIN_IDELTA_D 0
#define PID_IDELTA_MAX_VOLTAGE 9.0f
#define PID_IDELTA_MIN_VOLTAGE -9.0f

//���x�����PID
#define VEL_CONTROL 0
#define PID_GAIN_VEL_P 0.001f
#define PID_GAIN_VEL_I 0.01f
#define PID_GAIN_VEL_D 0.01f
#define PID_CYCLE_TIME_VEL PWM_PERIOD_SEC


//OpenLoop����ύX�J�n�̉����x
#define OPENLOOP_END_OMEGA 1000
//OpenLoop ���� FOC�ɐ؂�ւ���܂ł̃X�e�b�v��
#define OPEN_TO_FOC_TRANSITION_COUNT_STEP1 10000
#define OPEN_TO_FOC_TRANSITION_COUNT_STEP2 10000

//�����]���̐ݒ�
//#define FC_TARGET_RPM 1000
//#define FC_TARGET_ACCEL_OMEGA 0.3f
//#define FC_TARGET_ACCEL (float)FC_TARGET_ACCEL_OMEGA/CONTROL_FREQ_HZ

#define OPENLOOP_TARGET_RPS 7*10 //�b��200��
#define OPENLOOP_TARGET_ACCEL 0.1f/50000


#define ENCODER_ABZ 1
#define ENCODER_PERIOD (2000 - 1)

//�����g�d��
#define FOC_CONVOLUTION 1
//����
#define MEASURE_HIGH_FREQ_RESOLUTION 1000

#endif /* PARAMSETTING_H_ */