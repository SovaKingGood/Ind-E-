/*
 * Copyright (c) 2020 Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef BOARD_H
#define BOARD_H

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//
// Included Files
//

#include "driverlib.h"
#include "device.h"

//*****************************************************************************
//
// PinMux Configurations
//
//*****************************************************************************

//
// EPWM1 -> PhaseA Pinmux
//
//
// EPWM1A - GPIO Settings
//
#define GPIO_PIN_EPWM1A 0
#define PhaseA_EPWMA_GPIO 0
#define PhaseA_EPWMA_PIN_CONFIG GPIO_0_EPWM1A
//
// EPWM1B - GPIO Settings
//
#define GPIO_PIN_EPWM1B 1
#define PhaseA_EPWMB_GPIO 1
#define PhaseA_EPWMB_PIN_CONFIG GPIO_1_EPWM1B

//
// EPWM2 -> PhaseB Pinmux
//
//
// EPWM2A - GPIO Settings
//
#define GPIO_PIN_EPWM2A 2
#define PhaseB_EPWMA_GPIO 2
#define PhaseB_EPWMA_PIN_CONFIG GPIO_2_EPWM2A
//
// EPWM2B - GPIO Settings
//
#define GPIO_PIN_EPWM2B 3
#define PhaseB_EPWMB_GPIO 3
#define PhaseB_EPWMB_PIN_CONFIG GPIO_3_EPWM2B

//
// EPWM3 -> PhaseC Pinmux
//
//
// EPWM3A - GPIO Settings
//
#define GPIO_PIN_EPWM3A 4
#define PhaseC_EPWMA_GPIO 4
#define PhaseC_EPWMA_PIN_CONFIG GPIO_4_EPWM3A
//
// EPWM3B - GPIO Settings
//
#define GPIO_PIN_EPWM3B 5
#define PhaseC_EPWMB_GPIO 5
#define PhaseC_EPWMB_PIN_CONFIG GPIO_5_EPWM3B

//
// EPWM4 -> rpmInterrupt Pinmux
//
//
// EPWM4A - GPIO Settings
//
#define GPIO_PIN_EPWM4A 6
#define rpmInterrupt_EPWMA_GPIO 6
#define rpmInterrupt_EPWMA_PIN_CONFIG GPIO_6_EPWM4A
//
// EPWM4B - GPIO Settings
//
#define GPIO_PIN_EPWM4B 7
#define rpmInterrupt_EPWMB_GPIO 7
#define rpmInterrupt_EPWMB_PIN_CONFIG GPIO_7_EPWM4B

//
// EPWM6 -> Blue_Led Pinmux
//
//
// EPWM6A - GPIO Settings
//
#define GPIO_PIN_EPWM6A 10
#define Blue_Led_EPWMA_GPIO 10
#define Blue_Led_EPWMA_PIN_CONFIG GPIO_10_EPWM6A
//
// EPWM6B - GPIO Settings
//
#define GPIO_PIN_EPWM6B 11
#define Blue_Led_EPWMB_GPIO 11
#define Blue_Led_EPWMB_PIN_CONFIG GPIO_11_EPWM6B

//
// EQEP2 -> RPM Pinmux
//
//
// EQEP2A - GPIO Settings
//
#define GPIO_PIN_EQEP2A 54
#define RPM_EQEPA_GPIO 54
#define RPM_EQEPA_PIN_CONFIG GPIO_54_EQEP2A
//
// EQEP2B - GPIO Settings
//
#define GPIO_PIN_EQEP2B 55
#define RPM_EQEPB_GPIO 55
#define RPM_EQEPB_PIN_CONFIG GPIO_55_EQEP2B
//
// EQEP2I - GPIO Settings
//
#define GPIO_PIN_EQEP2I 57
#define RPM_EQEPINDEX_GPIO 57
#define RPM_EQEPINDEX_PIN_CONFIG GPIO_57_EQEP2I
//
// GPIO32 - GPIO Settings
//
#define Brake_GPIO_PIN_CONFIG GPIO_32_GPIO32
//
// GPIO29 - GPIO Settings
//
#define Reverse_GPIO_PIN_CONFIG GPIO_29_GPIO29
//
// GPIO67 - GPIO Settings
//
#define HALL_A_GPIO_PIN_CONFIG GPIO_67_GPIO67
//
// GPIO22 - GPIO Settings
//
#define HALL_B_GPIO_PIN_CONFIG GPIO_22_GPIO22
//
// GPIO94 - GPIO Settings
//
#define HALL_C_GPIO_PIN_CONFIG GPIO_94_GPIO94
//
// GPIO122 - GPIO Settings
//
#define ESP32PWM_GPIO_PIN_CONFIG GPIO_122_GPIO122

//*****************************************************************************
//
// ADC Configurations
//
//*****************************************************************************
#define myADC0_BASE ADCA_BASE
#define myADC0_RESULT_BASE ADCARESULT_BASE
#define myADC0_SOC0 ADC_SOC_NUMBER0
#define myADC0_FORCE_SOC0 ADC_FORCE_SOC0
#define myADC0_SAMPLE_WINDOW_SOC0 100
#define myADC0_TRIGGER_SOURCE_SOC0 ADC_TRIGGER_EPWM1_SOCA
#define myADC0_CHANNEL_SOC0 ADC_CH_ADCIN0
#define myADC0_SOC1 ADC_SOC_NUMBER1
#define myADC0_FORCE_SOC1 ADC_FORCE_SOC1
#define myADC0_SAMPLE_WINDOW_SOC1 100
#define myADC0_TRIGGER_SOURCE_SOC1 ADC_TRIGGER_EPWM1_SOCA
#define myADC0_CHANNEL_SOC1 ADC_CH_ADCIN1
#define myADC0_A_HIGH ADC_SOC_NUMBER2
#define myADC0_FORCE_A_HIGH ADC_FORCE_SOC2
#define myADC0_SAMPLE_WINDOW_A_HIGH 100
#define myADC0_TRIGGER_SOURCE_A_HIGH ADC_TRIGGER_EPWM1_SOCA
#define myADC0_CHANNEL_A_HIGH ADC_CH_ADCIN2
#define myADC0_A_LOW ADC_SOC_NUMBER3
#define myADC0_FORCE_A_LOW ADC_FORCE_SOC3
#define myADC0_SAMPLE_WINDOW_A_LOW 100
#define myADC0_TRIGGER_SOURCE_A_LOW ADC_TRIGGER_EPWM1_SOCA
#define myADC0_CHANNEL_A_LOW ADC_CH_ADCIN3
void myADC0_init();

#define myADCB_BASE ADCB_BASE
#define myADCB_RESULT_BASE ADCBRESULT_BASE
#define myADCB_SOC0 ADC_SOC_NUMBER0
#define myADCB_FORCE_SOC0 ADC_FORCE_SOC0
#define myADCB_SAMPLE_WINDOW_SOC0 100
#define myADCB_TRIGGER_SOURCE_SOC0 ADC_TRIGGER_SW_ONLY
#define myADCB_CHANNEL_SOC0 ADC_CH_ADCIN0
void myADCB_init();

#define myADCC_BASE ADCC_BASE
#define myADCC_RESULT_BASE ADCCRESULT_BASE
#define myADCC_SOC0 ADC_SOC_NUMBER0
#define myADCC_FORCE_SOC0 ADC_FORCE_SOC0
#define myADCC_SAMPLE_WINDOW_SOC0 100
#define myADCC_TRIGGER_SOURCE_SOC0 ADC_TRIGGER_EPWM1_SOCA
#define myADCC_CHANNEL_SOC0 ADC_CH_ADCIN0
#define myADCC_B_Low ADC_SOC_NUMBER2
#define myADCC_FORCE_B_Low ADC_FORCE_SOC2
#define myADCC_SAMPLE_WINDOW_B_Low 100
#define myADCC_TRIGGER_SOURCE_B_Low ADC_TRIGGER_EPWM1_SOCA
#define myADCC_CHANNEL_B_Low ADC_CH_ADCIN2
void myADCC_init();


//*****************************************************************************
//
// ECAP Configurations
//
//*****************************************************************************
#define esp32pwm_BASE ECAP2_BASE
#define esp32pwm_SIGNAL_MUNIT_BASE ECAP2SIGNALMONITORING_BASE
void esp32pwm_init();

//*****************************************************************************
//
// EPWM Configurations
//
//*****************************************************************************
#define PhaseA_BASE EPWM1_BASE
#define PhaseA_TBPRD 4000
#define PhaseA_COUNTER_MODE EPWM_COUNTER_MODE_UP
#define PhaseA_TBPHS 0
#define PhaseA_CMPA 5
#define PhaseA_CMPB 5
#define PhaseA_CMPC 5
#define PhaseA_CMPD 10
#define PhaseA_DBRED 50
#define PhaseA_DBFED 50
#define PhaseA_TZA_ACTION EPWM_TZ_ACTION_HIGH_Z
#define PhaseA_TZB_ACTION EPWM_TZ_ACTION_HIGH_Z
#define PhaseA_INTERRUPT_SOURCE EPWM_INT_TBCTR_U_CMPC
void PhaseA_init();
#define PhaseB_BASE EPWM2_BASE
#define PhaseB_TBPRD 4000
#define PhaseB_COUNTER_MODE EPWM_COUNTER_MODE_UP
#define PhaseB_TBPHS 2
#define PhaseB_CMPA 2
#define PhaseB_CMPB 1
#define PhaseB_CMPC 0
#define PhaseB_CMPD 0
#define PhaseB_DBRED 50
#define PhaseB_DBFED 50
#define PhaseB_TZA_ACTION EPWM_TZ_ACTION_HIGH_Z
#define PhaseB_TZB_ACTION EPWM_TZ_ACTION_HIGH_Z
#define PhaseB_INTERRUPT_SOURCE EPWM_INT_TBCTR_DISABLED
void PhaseB_init();
#define PhaseC_BASE EPWM3_BASE
#define PhaseC_TBPRD 4000
#define PhaseC_COUNTER_MODE EPWM_COUNTER_MODE_UP
#define PhaseC_TBPHS 2
#define PhaseC_CMPA 5
#define PhaseC_CMPB 5
#define PhaseC_CMPC 0
#define PhaseC_CMPD 0
#define PhaseC_DBRED 50
#define PhaseC_DBFED 50
#define PhaseC_TZA_ACTION EPWM_TZ_ACTION_HIGH_Z
#define PhaseC_TZB_ACTION EPWM_TZ_ACTION_HIGH_Z
#define PhaseC_INTERRUPT_SOURCE EPWM_INT_TBCTR_DISABLED
void PhaseC_init();
#define rpmInterrupt_BASE EPWM4_BASE
#define rpmInterrupt_TBPRD 60000
#define rpmInterrupt_COUNTER_MODE EPWM_COUNTER_MODE_UP
#define rpmInterrupt_TBPHS 0
#define rpmInterrupt_CMPA 30000
#define rpmInterrupt_CMPB 0
#define rpmInterrupt_CMPC 0
#define rpmInterrupt_CMPD 0
#define rpmInterrupt_DBRED 0
#define rpmInterrupt_DBFED 0
#define rpmInterrupt_TZA_ACTION EPWM_TZ_ACTION_HIGH_Z
#define rpmInterrupt_TZB_ACTION EPWM_TZ_ACTION_HIGH_Z
#define rpmInterrupt_INTERRUPT_SOURCE EPWM_INT_TBCTR_U_CMPA
void rpmInterrupt_init();
#define Blue_Led_BASE EPWM6_BASE
#define Blue_Led_TBPRD 30000
#define Blue_Led_COUNTER_MODE EPWM_COUNTER_MODE_UP
#define Blue_Led_TBPHS 0
#define Blue_Led_CMPA 5000
#define Blue_Led_CMPB 0
#define Blue_Led_CMPC 0
#define Blue_Led_CMPD 0
#define Blue_Led_DBRED 0
#define Blue_Led_DBFED 0
#define Blue_Led_TZA_ACTION EPWM_TZ_ACTION_HIGH_Z
#define Blue_Led_TZB_ACTION EPWM_TZ_ACTION_HIGH_Z
#define Blue_Led_INTERRUPT_SOURCE EPWM_INT_TBCTR_DISABLED
void Blue_Led_init();

//*****************************************************************************
//
// EQEP Configurations
//
//*****************************************************************************
#define RPM_BASE EQEP2_BASE
void RPM_init();

//*****************************************************************************
//
// GPIO Configurations
//
//*****************************************************************************
#define Brake 32
void Brake_init();
#define Reverse 29
void Reverse_init();
#define HALL_A 67
void HALL_A_init();
#define HALL_B 22
void HALL_B_init();
#define HALL_C 94
void HALL_C_init();
#define ESP32PWM 122
void ESP32PWM_init();

//*****************************************************************************
//
// INPUTXBAR Configurations
//
//*****************************************************************************
#define myINPUTXBARINPUT0_SOURCE 0
#define myINPUTXBARINPUT0_INPUT XBAR_INPUT1
void myINPUTXBARINPUT0_init();
#define ESP32XBAR_SOURCE 122
#define ESP32XBAR_INPUT XBAR_INPUT8
void ESP32XBAR_init();

//*****************************************************************************
//
// INTERRUPT Configurations
//
//*****************************************************************************

// Interrupt Settings for INT_esp32pwm
// ISR need to be defined for the registered interrupts
#define INT_esp32pwm INT_ECAP2
#define INT_esp32pwm_INTERRUPT_ACK_GROUP INTERRUPT_ACK_GROUP4
extern __interrupt void INT_esp32pwm_ISR(void);

// Interrupt Settings for INT_PhaseA
// ISR need to be defined for the registered interrupts
#define INT_PhaseA INT_EPWM1
#define INT_PhaseA_INTERRUPT_ACK_GROUP INTERRUPT_ACK_GROUP3
extern __interrupt void INT_PhaseA_ISR(void);

// Interrupt Settings for INT_rpmInterrupt
// ISR need to be defined for the registered interrupts
#define INT_rpmInterrupt INT_EPWM4
#define INT_rpmInterrupt_INTERRUPT_ACK_GROUP INTERRUPT_ACK_GROUP3
extern __interrupt void INT_rpmInterrupt_ISR(void);

//*****************************************************************************
//
// SYNC Scheme Configurations
//
//*****************************************************************************

//*****************************************************************************
//
// Board Configurations
//
//*****************************************************************************
void	Board_init();
void	ADC_init();
void	ECAP_init();
void	EPWM_init();
void	EQEP_init();
void	GPIO_init();
void	INPUTXBAR_init();
void	INTERRUPT_init();
void	SYNC_init();
void	PinMux_init();

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif  // end of BOARD_H definition
