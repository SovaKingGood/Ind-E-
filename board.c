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

#include "board.h"

//*****************************************************************************
//
// Board Configurations
// Initializes the rest of the modules. 
// Call this function in your application if you wish to do all module 
// initialization.
// If you wish to not use some of the initializations, instead of the 
// Board_init use the individual Module_inits
//
//*****************************************************************************
void Board_init()
{
	EALLOW;

	PinMux_init();
	INPUTXBAR_init();
	SYNC_init();
	ADC_init();
	ECAP_init();
	EPWM_init();
	EQEP_init();
	GPIO_init();
	INTERRUPT_init();

	EDIS;
}

//*****************************************************************************
//
// PINMUX Configurations
//
//*****************************************************************************
void PinMux_init()
{
	//
	// PinMux for modules assigned to CPU1
	//
	
	//
	// EPWM1 -> PhaseA Pinmux
	//
	GPIO_setPinConfig(PhaseA_EPWMA_PIN_CONFIG);
	GPIO_setPadConfig(PhaseA_EPWMA_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(PhaseA_EPWMA_GPIO, GPIO_QUAL_SYNC);

	GPIO_setPinConfig(PhaseA_EPWMB_PIN_CONFIG);
	GPIO_setPadConfig(PhaseA_EPWMB_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(PhaseA_EPWMB_GPIO, GPIO_QUAL_SYNC);

	//
	// EPWM2 -> PhaseB Pinmux
	//
	GPIO_setPinConfig(PhaseB_EPWMA_PIN_CONFIG);
	GPIO_setPadConfig(PhaseB_EPWMA_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(PhaseB_EPWMA_GPIO, GPIO_QUAL_SYNC);

	GPIO_setPinConfig(PhaseB_EPWMB_PIN_CONFIG);
	GPIO_setPadConfig(PhaseB_EPWMB_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(PhaseB_EPWMB_GPIO, GPIO_QUAL_SYNC);

	//
	// EPWM3 -> PhaseC Pinmux
	//
	GPIO_setPinConfig(PhaseC_EPWMA_PIN_CONFIG);
	GPIO_setPadConfig(PhaseC_EPWMA_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(PhaseC_EPWMA_GPIO, GPIO_QUAL_SYNC);

	GPIO_setPinConfig(PhaseC_EPWMB_PIN_CONFIG);
	GPIO_setPadConfig(PhaseC_EPWMB_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(PhaseC_EPWMB_GPIO, GPIO_QUAL_SYNC);

	//
	// EPWM4 -> rpmInterrupt Pinmux
	//
	GPIO_setPinConfig(rpmInterrupt_EPWMA_PIN_CONFIG);
	GPIO_setPadConfig(rpmInterrupt_EPWMA_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(rpmInterrupt_EPWMA_GPIO, GPIO_QUAL_SYNC);

	GPIO_setPinConfig(rpmInterrupt_EPWMB_PIN_CONFIG);
	GPIO_setPadConfig(rpmInterrupt_EPWMB_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(rpmInterrupt_EPWMB_GPIO, GPIO_QUAL_SYNC);

	//
	// EPWM6 -> Blue_Led Pinmux
	//
	GPIO_setPinConfig(Blue_Led_EPWMA_PIN_CONFIG);
	GPIO_setPadConfig(Blue_Led_EPWMA_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(Blue_Led_EPWMA_GPIO, GPIO_QUAL_SYNC);

	GPIO_setPinConfig(Blue_Led_EPWMB_PIN_CONFIG);
	GPIO_setPadConfig(Blue_Led_EPWMB_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(Blue_Led_EPWMB_GPIO, GPIO_QUAL_SYNC);

	//
	// EQEP2 -> RPM Pinmux
	//
	GPIO_setPinConfig(RPM_EQEPA_PIN_CONFIG);
	GPIO_setPadConfig(RPM_EQEPA_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(RPM_EQEPA_GPIO, GPIO_QUAL_SYNC);

	GPIO_setPinConfig(RPM_EQEPB_PIN_CONFIG);
	GPIO_setPadConfig(RPM_EQEPB_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(RPM_EQEPB_GPIO, GPIO_QUAL_SYNC);

	GPIO_setPinConfig(RPM_EQEPINDEX_PIN_CONFIG);
	GPIO_setPadConfig(RPM_EQEPINDEX_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(RPM_EQEPINDEX_GPIO, GPIO_QUAL_SYNC);

	// GPIO32 -> Brake Pinmux
	GPIO_setPinConfig(GPIO_32_GPIO32);
	// GPIO29 -> Reverse Pinmux
	GPIO_setPinConfig(GPIO_29_GPIO29);
	// GPIO67 -> HALL_A Pinmux
	GPIO_setPinConfig(GPIO_67_GPIO67);
	// GPIO22 -> HALL_B Pinmux
	GPIO_setPinConfig(GPIO_22_GPIO22);
	// GPIO94 -> HALL_C Pinmux
	GPIO_setPinConfig(GPIO_94_GPIO94);
	// GPIO122 -> ESP32PWM Pinmux
	GPIO_setPinConfig(GPIO_122_GPIO122);

}

//*****************************************************************************
//
// ADC Configurations
//
//*****************************************************************************
void ADC_init(){
	myADC0_init();
	myADCB_init();
	myADCC_init();
}

void myADC0_init(){
	//
	// Configures the analog-to-digital converter module prescaler.
	//
	ADC_setPrescaler(myADC0_BASE, ADC_CLK_DIV_4_0);
	//
	// Configures the analog-to-digital converter resolution and signal mode.
	//
	ADC_setMode(myADC0_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
	//
	// Sets the timing of the end-of-conversion pulse
	//
	ADC_setInterruptPulseMode(myADC0_BASE, ADC_PULSE_END_OF_CONV);
	//
	// Powers up the analog-to-digital converter core.
	//
	ADC_enableConverter(myADC0_BASE);
	//
	// Delay for 1ms to allow ADC time to power up
	//
	DEVICE_DELAY_US(500);
	//
	// SOC Configuration: Setup ADC EPWM channel and trigger settings
	//
	// Disables SOC burst mode.
	//
	ADC_disableBurstMode(myADC0_BASE);
	//
	// Sets the priority mode of the SOCs.
	//
	ADC_setSOCPriority(myADC0_BASE, ADC_PRI_ALL_ROUND_ROBIN);
	//
	// Start of Conversion 0 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 0
	//	  	Trigger			: ADC_TRIGGER_EPWM1_SOCA
	//	  	Channel			: ADC_CH_ADCIN0
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(myADC0_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN0, 20U);
	ADC_setInterruptSOCTrigger(myADC0_BASE, ADC_SOC_NUMBER0, ADC_INT_SOC_TRIGGER_NONE);
	//
	// Start of Conversion 1 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 1
	//	  	Trigger			: ADC_TRIGGER_EPWM1_SOCA
	//	  	Channel			: ADC_CH_ADCIN1
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(myADC0_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN1, 20U);
	ADC_setInterruptSOCTrigger(myADC0_BASE, ADC_SOC_NUMBER1, ADC_INT_SOC_TRIGGER_NONE);
	//
	// Start of Conversion 2 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 2
	//	  	Trigger			: ADC_TRIGGER_EPWM1_SOCA
	//	  	Channel			: ADC_CH_ADCIN2
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(myADC0_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN2, 20U);
	ADC_setInterruptSOCTrigger(myADC0_BASE, ADC_SOC_NUMBER2, ADC_INT_SOC_TRIGGER_NONE);
	//
	// Start of Conversion 3 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 3
	//	  	Trigger			: ADC_TRIGGER_EPWM1_SOCA
	//	  	Channel			: ADC_CH_ADCIN3
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(myADC0_BASE, ADC_SOC_NUMBER3, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN3, 20U);
	ADC_setInterruptSOCTrigger(myADC0_BASE, ADC_SOC_NUMBER3, ADC_INT_SOC_TRIGGER_NONE);
	//
	// ADC Interrupt 2 Configuration
	// 		Source	: ADC_SOC_NUMBER1
	// 		Interrupt Source: enabled
	// 		Continuous Mode	: disabled
	//
	//
	ADC_setInterruptSource(myADC0_BASE, ADC_INT_NUMBER2, ADC_SOC_NUMBER1);
	ADC_clearInterruptStatus(myADC0_BASE, ADC_INT_NUMBER2);
	ADC_disableContinuousMode(myADC0_BASE, ADC_INT_NUMBER2);
	ADC_enableInterrupt(myADC0_BASE, ADC_INT_NUMBER2);
}

void myADCB_init(){
	//
	// Configures the analog-to-digital converter module prescaler.
	//
	ADC_setPrescaler(myADCB_BASE, ADC_CLK_DIV_4_0);
	//
	// Configures the analog-to-digital converter resolution and signal mode.
	//
	ADC_setMode(myADCB_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
	//
	// Sets the timing of the end-of-conversion pulse
	//
	ADC_setInterruptPulseMode(myADCB_BASE, ADC_PULSE_END_OF_ACQ_WIN);
	//
	// Powers up the analog-to-digital converter core.
	//
	ADC_enableConverter(myADCB_BASE);
	//
	// Delay for 1ms to allow ADC time to power up
	//
	DEVICE_DELAY_US(500);
	//
	// SOC Configuration: Setup ADC EPWM channel and trigger settings
	//
	// Disables SOC burst mode.
	//
	ADC_disableBurstMode(myADCB_BASE);
	//
	// Sets the priority mode of the SOCs.
	//
	ADC_setSOCPriority(myADCB_BASE, ADC_PRI_ALL_ROUND_ROBIN);
	//
	// Start of Conversion 0 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 0
	//	  	Trigger			: ADC_TRIGGER_SW_ONLY
	//	  	Channel			: ADC_CH_ADCIN0
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(myADCB_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 20U);
	ADC_setInterruptSOCTrigger(myADCB_BASE, ADC_SOC_NUMBER0, ADC_INT_SOC_TRIGGER_NONE);
}

void myADCC_init(){
	//
	// Configures the analog-to-digital converter module prescaler.
	//
	ADC_setPrescaler(myADCC_BASE, ADC_CLK_DIV_4_0);
	//
	// Configures the analog-to-digital converter resolution and signal mode.
	//
	ADC_setMode(myADCC_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
	//
	// Sets the timing of the end-of-conversion pulse
	//
	ADC_setInterruptPulseMode(myADCC_BASE, ADC_PULSE_END_OF_ACQ_WIN);
	//
	// Powers up the analog-to-digital converter core.
	//
	ADC_enableConverter(myADCC_BASE);
	//
	// Delay for 1ms to allow ADC time to power up
	//
	DEVICE_DELAY_US(500);
	//
	// SOC Configuration: Setup ADC EPWM channel and trigger settings
	//
	// Disables SOC burst mode.
	//
	ADC_disableBurstMode(myADCC_BASE);
	//
	// Sets the priority mode of the SOCs.
	//
	ADC_setSOCPriority(myADCC_BASE, ADC_PRI_ALL_ROUND_ROBIN);
	//
	// Start of Conversion 0 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 0
	//	  	Trigger			: ADC_TRIGGER_EPWM1_SOCA
	//	  	Channel			: ADC_CH_ADCIN0
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(myADCC_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN0, 20U);
	ADC_setInterruptSOCTrigger(myADCC_BASE, ADC_SOC_NUMBER0, ADC_INT_SOC_TRIGGER_NONE);
	//
	// Start of Conversion 2 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 2
	//	  	Trigger			: ADC_TRIGGER_EPWM1_SOCA
	//	  	Channel			: ADC_CH_ADCIN2
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(myADCC_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN2, 20U);
	ADC_setInterruptSOCTrigger(myADCC_BASE, ADC_SOC_NUMBER2, ADC_INT_SOC_TRIGGER_NONE);
}


//*****************************************************************************
//
// ECAP Configurations
//
//*****************************************************************************
void ECAP_init(){
	esp32pwm_init();
}

void esp32pwm_init(){
	//
	// Disable ,clear all capture flags and interrupts
	//
	ECAP_disableInterrupt(esp32pwm_BASE,
		(ECAP_ISR_SOURCE_CAPTURE_EVENT_1  |
		ECAP_ISR_SOURCE_CAPTURE_EVENT_2  |
		ECAP_ISR_SOURCE_CAPTURE_EVENT_3  |
		ECAP_ISR_SOURCE_CAPTURE_EVENT_4  |
		ECAP_ISR_SOURCE_COUNTER_OVERFLOW |
		ECAP_ISR_SOURCE_COUNTER_PERIOD   |
		ECAP_ISR_SOURCE_COUNTER_COMPARE));
	ECAP_clearInterrupt(esp32pwm_BASE,
		(ECAP_ISR_SOURCE_CAPTURE_EVENT_1  |
		ECAP_ISR_SOURCE_CAPTURE_EVENT_2  |
		ECAP_ISR_SOURCE_CAPTURE_EVENT_3  |
		ECAP_ISR_SOURCE_CAPTURE_EVENT_4  |
		ECAP_ISR_SOURCE_COUNTER_OVERFLOW |
		ECAP_ISR_SOURCE_COUNTER_PERIOD   |
		ECAP_ISR_SOURCE_COUNTER_COMPARE));
	//
	// Disables time stamp capture.
	//
	ECAP_disableTimeStampCapture(esp32pwm_BASE);
	//
	// Stops Time stamp counter.
	//
	ECAP_stopCounter(esp32pwm_BASE);
	//
	// Sets eCAP in Capture mode.
	//
	ECAP_enableCaptureMode(esp32pwm_BASE);
	//
	// Sets the capture mode.
	//
	ECAP_setCaptureMode(esp32pwm_BASE,ECAP_CONTINUOUS_CAPTURE_MODE,ECAP_EVENT_4);
	//
	// Sets the Capture event prescaler.
	//
	ECAP_setEventPrescaler(esp32pwm_BASE, 0U);
	//
	// Sets the Capture event polarity.
	//
	ECAP_setEventPolarity(esp32pwm_BASE,ECAP_EVENT_1,ECAP_EVNT_RISING_EDGE);
	ECAP_setEventPolarity(esp32pwm_BASE,ECAP_EVENT_2,ECAP_EVNT_FALLING_EDGE);
	ECAP_setEventPolarity(esp32pwm_BASE,ECAP_EVENT_3,ECAP_EVNT_RISING_EDGE);
	ECAP_setEventPolarity(esp32pwm_BASE,ECAP_EVENT_4,ECAP_EVNT_FALLING_EDGE);
	//
	// Configure counter reset on events
	//
	ECAP_disableCounterResetOnEvent(esp32pwm_BASE,ECAP_EVENT_1);
	ECAP_disableCounterResetOnEvent(esp32pwm_BASE,ECAP_EVENT_2);
	ECAP_disableCounterResetOnEvent(esp32pwm_BASE,ECAP_EVENT_3);
	ECAP_disableCounterResetOnEvent(esp32pwm_BASE,ECAP_EVENT_4);
	//
	// Sets a phase shift value count.
	//
	ECAP_setPhaseShiftCount(esp32pwm_BASE,0U);
	//
	// Disable counter loading with phase shift value.
	//
	ECAP_disableLoadCounter(esp32pwm_BASE);
	//
	// Configures Sync out signal mode.
	//
	ECAP_setSyncOutMode(esp32pwm_BASE,ECAP_SYNC_OUT_DISABLED);
	//
	// Configures emulation mode.
	//
	ECAP_setEmulationMode(esp32pwm_BASE,ECAP_EMULATION_STOP);
	//
	// Starts Time stamp counter for esp32pwm.
	//
	ECAP_startCounter(esp32pwm_BASE);
	//
	// Enables time stamp capture for esp32pwm.
	//
	ECAP_enableTimeStampCapture(esp32pwm_BASE);
	//
	// Re-arms the eCAP module for esp32pwm.
	//
	ECAP_reArm(esp32pwm_BASE);
	//
	// Enables interrupt source for esp32pwm.
	//
	ECAP_enableInterrupt(esp32pwm_BASE,(ECAP_ISR_SOURCE_CAPTURE_EVENT_4));

    //-----------------Signal Monitoring--------------------//
}

//*****************************************************************************
//
// EPWM Configurations
//
//*****************************************************************************
void EPWM_init(){
	PhaseA_init();
	PhaseB_init();
	PhaseC_init();
	rpmInterrupt_init();
	Blue_Led_init();
}
void PhaseA_init(){
    EPWM_setClockPrescaler(PhaseA_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);	
    EPWM_setPeriodLoadMode(PhaseA_BASE, EPWM_PERIOD_DIRECT_LOAD);	
    EPWM_setTimeBasePeriod(PhaseA_BASE, 4000);	
    EPWM_enableGlobalLoadRegisters(PhaseA_BASE, EPWM_GL_REGISTER_TBPRD_TBPRDHR);	
    EPWM_setTimeBaseCounter(PhaseA_BASE, 0);	
    EPWM_setTimeBaseCounterMode(PhaseA_BASE, EPWM_COUNTER_MODE_UP);	
    EPWM_disablePhaseShiftLoad(PhaseA_BASE);	
    EPWM_setPhaseShift(PhaseA_BASE, 0);	
    EPWM_setSyncOutPulseMode(PhaseA_BASE, EPWM_SYNC_OUT_PULSE_ON_COUNTER_ZERO);	
    EPWM_setSyncPulseSource(PhaseA_BASE, HRPWM_PWMSYNC_SOURCE_ZERO);	
    EPWM_setCounterCompareValue(PhaseA_BASE, EPWM_COUNTER_COMPARE_A, 5);	
    EPWM_setCounterCompareShadowLoadMode(PhaseA_BASE, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_PERIOD);	
    EPWM_setCounterCompareValue(PhaseA_BASE, EPWM_COUNTER_COMPARE_B, 5);	
    EPWM_setCounterCompareShadowLoadMode(PhaseA_BASE, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_PERIOD);	
    EPWM_setCounterCompareValue(PhaseA_BASE, EPWM_COUNTER_COMPARE_C, 5);	
    EPWM_setCounterCompareValue(PhaseA_BASE, EPWM_COUNTER_COMPARE_D, 10);	
    EPWM_enableGlobalLoadRegisters(PhaseA_BASE, EPWM_GL_REGISTER_CMPD);	
    EPWM_setActionQualifierT2TriggerSource(PhaseA_BASE, EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_2);	
    EPWM_setActionQualifierShadowLoadMode(PhaseA_BASE, EPWM_ACTION_QUALIFIER_A, EPWM_AQ_LOAD_ON_CNTR_PERIOD);	
    EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
    EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	
    EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
    EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);	
    EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
    EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);	
    EPWM_setActionQualifierShadowLoadMode(PhaseA_BASE, EPWM_ACTION_QUALIFIER_B, EPWM_AQ_LOAD_ON_CNTR_PERIOD);	
    EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
    EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	
    EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
    EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);	
    EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
    EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);	
    EPWM_setDeadBandDelayPolarity(PhaseA_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);	
    EPWM_setDeadBandDelayMode(PhaseA_BASE, EPWM_DB_RED, true);	
    EPWM_setRisingEdgeDelayCountShadowLoadMode(PhaseA_BASE, EPWM_RED_LOAD_ON_CNTR_ZERO);	
    EPWM_setRisingEdgeDelayCount(PhaseA_BASE, 50);	
    EPWM_setDeadBandDelayMode(PhaseA_BASE, EPWM_DB_FED, true);	
    EPWM_setFallingEdgeDelayCountShadowLoadMode(PhaseA_BASE, EPWM_FED_LOAD_ON_CNTR_ZERO);	
    EPWM_setFallingEdgeDelayCount(PhaseA_BASE, 50);	
    EPWM_setDeadBandCounterClock(PhaseA_BASE, EPWM_DB_COUNTER_CLOCK_HALF_CYCLE);	
    EPWM_enableInterrupt(PhaseA_BASE);	
    EPWM_setInterruptSource(PhaseA_BASE, EPWM_INT_TBCTR_U_CMPC);	
    EPWM_setInterruptEventCount(PhaseA_BASE, 1);	
    EPWM_enableADCTrigger(PhaseA_BASE, EPWM_SOC_A);	
    EPWM_setADCTriggerSource(PhaseA_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_U_CMPD);	
    EPWM_setADCTriggerEventPrescale(PhaseA_BASE, EPWM_SOC_A, 10);	
}
void PhaseB_init(){
    EPWM_setClockPrescaler(PhaseB_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);	
    EPWM_setPeriodLoadMode(PhaseB_BASE, EPWM_PERIOD_DIRECT_LOAD);	
    EPWM_setTimeBasePeriod(PhaseB_BASE, 4000);	
    EPWM_setupEPWMLinks(PhaseB_BASE, EPWM_LINK_WITH_EPWM_1, EPWM_LINK_TBPRD);	
    EPWM_enableGlobalLoadRegisters(PhaseB_BASE, EPWM_GL_REGISTER_TBPRD_TBPRDHR);	
    EPWM_setTimeBaseCounter(PhaseB_BASE, 0);	
    EPWM_setTimeBaseCounterMode(PhaseB_BASE, EPWM_COUNTER_MODE_UP);	
    EPWM_enablePhaseShiftLoad(PhaseB_BASE);	
    EPWM_setPhaseShift(PhaseB_BASE, 2);	
    EPWM_setSyncOutPulseMode(PhaseB_BASE, EPWM_SYNC_OUT_PULSE_ON_EPWMxSYNCIN);	
    EPWM_setSyncPulseSource(PhaseB_BASE, HRPWM_PWMSYNC_SOURCE_ZERO);	
    EPWM_setCounterCompareValue(PhaseB_BASE, EPWM_COUNTER_COMPARE_A, 2);	
    EPWM_disableCounterCompareShadowLoadMode(PhaseB_BASE, EPWM_COUNTER_COMPARE_A);	
    EPWM_setCounterCompareShadowLoadMode(PhaseB_BASE, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);	
    EPWM_setupEPWMLinks(PhaseB_BASE, EPWM_LINK_WITH_EPWM_1, EPWM_LINK_COMP_A);	
    EPWM_setCounterCompareValue(PhaseB_BASE, EPWM_COUNTER_COMPARE_B, 1);	
    EPWM_disableCounterCompareShadowLoadMode(PhaseB_BASE, EPWM_COUNTER_COMPARE_B);	
    EPWM_setCounterCompareShadowLoadMode(PhaseB_BASE, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);	
    EPWM_setupEPWMLinks(PhaseB_BASE, EPWM_LINK_WITH_EPWM_1, EPWM_LINK_COMP_B);	
    EPWM_disableCounterCompareShadowLoadMode(PhaseB_BASE, EPWM_COUNTER_COMPARE_C);	
    EPWM_disableCounterCompareShadowLoadMode(PhaseB_BASE, EPWM_COUNTER_COMPARE_D);	
    EPWM_setActionQualifierT2TriggerSource(PhaseB_BASE, EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_2);	
    EPWM_setActionQualifierShadowLoadMode(PhaseB_BASE, EPWM_ACTION_QUALIFIER_A, EPWM_AQ_LOAD_ON_CNTR_PERIOD);	
    EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
    EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	
    EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
    EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);	
    EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
    EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);	
    EPWM_setActionQualifierShadowLoadMode(PhaseB_BASE, EPWM_ACTION_QUALIFIER_B, EPWM_AQ_LOAD_ON_CNTR_PERIOD);	
    EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
    EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	
    EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
    EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);	
    EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
    EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);	
    EPWM_setDeadBandDelayPolarity(PhaseB_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);	
    EPWM_setDeadBandDelayMode(PhaseB_BASE, EPWM_DB_RED, true);	
    EPWM_setRisingEdgeDelayCountShadowLoadMode(PhaseB_BASE, EPWM_RED_LOAD_ON_CNTR_ZERO);	
    EPWM_setRisingEdgeDelayCount(PhaseB_BASE, 50);	
    EPWM_setDeadBandDelayMode(PhaseB_BASE, EPWM_DB_FED, true);	
    EPWM_setFallingEdgeDelayCountShadowLoadMode(PhaseB_BASE, EPWM_FED_LOAD_ON_CNTR_ZERO);	
    EPWM_setFallingEdgeDelayCount(PhaseB_BASE, 50);	
    EPWM_setDeadBandCounterClock(PhaseB_BASE, EPWM_DB_COUNTER_CLOCK_HALF_CYCLE);	
}
void PhaseC_init(){
    EPWM_setClockPrescaler(PhaseC_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);	
    EPWM_setPeriodLoadMode(PhaseC_BASE, EPWM_PERIOD_DIRECT_LOAD);	
    EPWM_setTimeBasePeriod(PhaseC_BASE, 4000);	
    EPWM_setupEPWMLinks(PhaseC_BASE, EPWM_LINK_WITH_EPWM_1, EPWM_LINK_TBPRD);	
    EPWM_enableGlobalLoadRegisters(PhaseC_BASE, EPWM_GL_REGISTER_TBPRD_TBPRDHR);	
    EPWM_setTimeBaseCounter(PhaseC_BASE, 0);	
    EPWM_setTimeBaseCounterMode(PhaseC_BASE, EPWM_COUNTER_MODE_UP);	
    EPWM_enablePhaseShiftLoad(PhaseC_BASE);	
    EPWM_setPhaseShift(PhaseC_BASE, 2);	
    EPWM_setSyncOutPulseMode(PhaseC_BASE, EPWM_SYNC_OUT_PULSE_ON_EPWMxSYNCIN);	
    EPWM_setSyncPulseSource(PhaseC_BASE, HRPWM_PWMSYNC_SOURCE_ZERO);	
    EPWM_setCounterCompareValue(PhaseC_BASE, EPWM_COUNTER_COMPARE_A, 5);	
    EPWM_disableCounterCompareShadowLoadMode(PhaseC_BASE, EPWM_COUNTER_COMPARE_A);	
    EPWM_setCounterCompareShadowLoadMode(PhaseC_BASE, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);	
    EPWM_setupEPWMLinks(PhaseC_BASE, EPWM_LINK_WITH_EPWM_1, EPWM_LINK_COMP_A);	
    EPWM_setCounterCompareValue(PhaseC_BASE, EPWM_COUNTER_COMPARE_B, 5);	
    EPWM_disableCounterCompareShadowLoadMode(PhaseC_BASE, EPWM_COUNTER_COMPARE_B);	
    EPWM_setCounterCompareShadowLoadMode(PhaseC_BASE, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);	
    EPWM_setupEPWMLinks(PhaseC_BASE, EPWM_LINK_WITH_EPWM_1, EPWM_LINK_COMP_B);	
    EPWM_setCounterCompareShadowLoadMode(PhaseC_BASE, EPWM_COUNTER_COMPARE_C, EPWM_COMP_LOAD_ON_CNTR_PERIOD);	
    EPWM_disableCounterCompareShadowLoadMode(PhaseC_BASE, EPWM_COUNTER_COMPARE_D);	
    EPWM_setActionQualifierT2TriggerSource(PhaseC_BASE, EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_2);	
    EPWM_setActionQualifierShadowLoadMode(PhaseC_BASE, EPWM_ACTION_QUALIFIER_A, EPWM_AQ_LOAD_ON_CNTR_PERIOD);	
    EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
    EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	
    EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
    EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);	
    EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
    EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);	
    EPWM_setActionQualifierShadowLoadMode(PhaseC_BASE, EPWM_ACTION_QUALIFIER_B, EPWM_AQ_LOAD_ON_CNTR_PERIOD);	
    EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
    EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	
    EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
    EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);	
    EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
    EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);	
    EPWM_setDeadBandDelayPolarity(PhaseC_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);	
    EPWM_setDeadBandDelayMode(PhaseC_BASE, EPWM_DB_RED, true);	
    EPWM_setRisingEdgeDelayCountShadowLoadMode(PhaseC_BASE, EPWM_RED_LOAD_ON_CNTR_ZERO);	
    EPWM_setRisingEdgeDelayCount(PhaseC_BASE, 50);	
    EPWM_setDeadBandDelayMode(PhaseC_BASE, EPWM_DB_FED, true);	
    EPWM_setFallingEdgeDelayCountShadowLoadMode(PhaseC_BASE, EPWM_FED_LOAD_ON_CNTR_ZERO);	
    EPWM_setFallingEdgeDelayCount(PhaseC_BASE, 50);	
    EPWM_setDeadBandCounterClock(PhaseC_BASE, EPWM_DB_COUNTER_CLOCK_HALF_CYCLE);	
}
void rpmInterrupt_init(){
    EPWM_setClockPrescaler(rpmInterrupt_BASE, EPWM_CLOCK_DIVIDER_128, EPWM_HSCLOCK_DIVIDER_2);	
    EPWM_setClockPrescaler(rpmInterrupt_BASE, EPWM_CLOCK_DIVIDER_128, EPWM_HSCLOCK_DIVIDER_2);	
    EPWM_setTimeBasePeriod(rpmInterrupt_BASE, 60000);	
    EPWM_setTimeBaseCounter(rpmInterrupt_BASE, 0);	
    EPWM_setTimeBaseCounterMode(rpmInterrupt_BASE, EPWM_COUNTER_MODE_UP);	
    EPWM_disablePhaseShiftLoad(rpmInterrupt_BASE);	
    EPWM_setPhaseShift(rpmInterrupt_BASE, 0);	
    EPWM_setSyncOutPulseMode(rpmInterrupt_BASE, EPWM_SYNC_OUT_PULSE_DISABLED);	
    EPWM_setCounterCompareValue(rpmInterrupt_BASE, EPWM_COUNTER_COMPARE_A, 30000);	
    EPWM_disableCounterCompareShadowLoadMode(rpmInterrupt_BASE, EPWM_COUNTER_COMPARE_A);	
    EPWM_setCounterCompareShadowLoadMode(rpmInterrupt_BASE, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);	
    EPWM_setCounterCompareValue(rpmInterrupt_BASE, EPWM_COUNTER_COMPARE_B, 0);	
    EPWM_setCounterCompareShadowLoadMode(rpmInterrupt_BASE, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);	
    EPWM_setActionQualifierAction(rpmInterrupt_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
    EPWM_setActionQualifierAction(rpmInterrupt_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	
    EPWM_setActionQualifierAction(rpmInterrupt_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
    EPWM_setActionQualifierAction(rpmInterrupt_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);	
    EPWM_setActionQualifierAction(rpmInterrupt_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
    EPWM_setActionQualifierAction(rpmInterrupt_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);	
    EPWM_setActionQualifierAction(rpmInterrupt_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
    EPWM_setActionQualifierAction(rpmInterrupt_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	
    EPWM_setActionQualifierAction(rpmInterrupt_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
    EPWM_setActionQualifierAction(rpmInterrupt_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);	
    EPWM_setActionQualifierAction(rpmInterrupt_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
    EPWM_setActionQualifierAction(rpmInterrupt_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);	
    EPWM_setRisingEdgeDelayCountShadowLoadMode(rpmInterrupt_BASE, EPWM_RED_LOAD_ON_CNTR_ZERO);	
    EPWM_disableRisingEdgeDelayCountShadowLoadMode(rpmInterrupt_BASE);	
    EPWM_setFallingEdgeDelayCountShadowLoadMode(rpmInterrupt_BASE, EPWM_FED_LOAD_ON_CNTR_ZERO);	
    EPWM_disableFallingEdgeDelayCountShadowLoadMode(rpmInterrupt_BASE);	
    EPWM_enableInterrupt(rpmInterrupt_BASE);	
    EPWM_setInterruptSource(rpmInterrupt_BASE, EPWM_INT_TBCTR_U_CMPA);	
    EPWM_setInterruptEventCount(rpmInterrupt_BASE, 1);	
}
void Blue_Led_init(){
    EPWM_setEmulationMode(Blue_Led_BASE, EPWM_EMULATION_FREE_RUN);	
    EPWM_setClockPrescaler(Blue_Led_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_12);	
    EPWM_setTimeBasePeriod(Blue_Led_BASE, 30000);	
    EPWM_setTimeBaseCounter(Blue_Led_BASE, 0);	
    EPWM_setTimeBaseCounterMode(Blue_Led_BASE, EPWM_COUNTER_MODE_UP);	
    EPWM_disablePhaseShiftLoad(Blue_Led_BASE);	
    EPWM_setPhaseShift(Blue_Led_BASE, 0);	
    EPWM_setCounterCompareValue(Blue_Led_BASE, EPWM_COUNTER_COMPARE_A, 5000);	
    EPWM_setCounterCompareShadowLoadMode(Blue_Led_BASE, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);	
    EPWM_setCounterCompareValue(Blue_Led_BASE, EPWM_COUNTER_COMPARE_B, 0);	
    EPWM_setCounterCompareShadowLoadMode(Blue_Led_BASE, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);	
    EPWM_setActionQualifierAction(Blue_Led_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
    EPWM_setActionQualifierAction(Blue_Led_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	
    EPWM_setActionQualifierAction(Blue_Led_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
    EPWM_setActionQualifierAction(Blue_Led_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);	
    EPWM_setActionQualifierAction(Blue_Led_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
    EPWM_setActionQualifierAction(Blue_Led_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);	
    EPWM_setActionQualifierSWAction(Blue_Led_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_TOGGLE);	
    EPWM_setActionQualifierAction(Blue_Led_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
    EPWM_setActionQualifierAction(Blue_Led_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	
    EPWM_setActionQualifierAction(Blue_Led_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
    EPWM_setActionQualifierAction(Blue_Led_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);	
    EPWM_setActionQualifierAction(Blue_Led_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
    EPWM_setActionQualifierAction(Blue_Led_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);	
    EPWM_setRisingEdgeDelayCountShadowLoadMode(Blue_Led_BASE, EPWM_RED_LOAD_ON_CNTR_ZERO);	
    EPWM_disableRisingEdgeDelayCountShadowLoadMode(Blue_Led_BASE);	
    EPWM_setFallingEdgeDelayCountShadowLoadMode(Blue_Led_BASE, EPWM_FED_LOAD_ON_CNTR_ZERO);	
    EPWM_disableFallingEdgeDelayCountShadowLoadMode(Blue_Led_BASE);	
}

//*****************************************************************************
//
// EQEP Configurations
//
//*****************************************************************************
void EQEP_init(){
	RPM_init();
}

void RPM_init(){
	//
	// Sets the polarity of the eQEP module's input signals.
	//
	EQEP_setInputPolarity(RPM_BASE,false,false,false,false);
	//
	// Configures eQEP module's quadrature decoder unit.
	//
	EQEP_setDecoderConfig(RPM_BASE, (EQEP_CONFIG_QUADRATURE | EQEP_CONFIG_2X_RESOLUTION | EQEP_CONFIG_NO_SWAP | EQEP_CONFIG_IGATE_DISABLE));
	//
	// Set the emulation mode of the eQEP module.
	//
	EQEP_setEmulationMode(RPM_BASE,EQEP_EMULATIONMODE_STOPIMMEDIATELY);
	//
	// Configures eQEP module position counter unit.
	//
	EQEP_setPositionCounterConfig(RPM_BASE,EQEP_POSITION_RESET_MAX_POS,2000U);
	//
	// Sets the current encoder position.
	//
	EQEP_setPosition(RPM_BASE,1000U);
	//
	// Disables the eQEP module unit timer.
	//
	EQEP_disableUnitTimer(RPM_BASE);
	//
	// Disables the eQEP module watchdog timer.
	//
	EQEP_disableWatchdog(RPM_BASE);
	//
	// Configures the quadrature modes in which the position count can be latched.
	//
	EQEP_setLatchMode(RPM_BASE,(EQEP_LATCH_CNT_READ_BY_CPU|EQEP_LATCH_RISING_STROBE|EQEP_LATCH_RISING_INDEX));
	//
	// Configures the mode in which the position counter is initialized.
	//
	EQEP_setPositionInitMode(RPM_BASE,(EQEP_INIT_EDGE_DIR_STROBE));
	//
	// Sets the software initialization of the encoder position counter.
	//
	EQEP_setSWPositionInit(RPM_BASE,false);
	//
	// Sets the init value for the encoder position counter.
	//
	EQEP_setInitialPosition(RPM_BASE,1000U);
	//
	// Enables the eQEP module.
	//
	EQEP_enableModule(RPM_BASE);
}

//*****************************************************************************
//
// GPIO Configurations
//
//*****************************************************************************
void GPIO_init(){
	Brake_init();
	Reverse_init();
	HALL_A_init();
	HALL_B_init();
	HALL_C_init();
	ESP32PWM_init();
}

void Brake_init(){
	GPIO_setPadConfig(Brake, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(Brake, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(Brake, GPIO_DIR_MODE_IN);
	GPIO_setControllerCore(Brake, GPIO_CORE_CPU1);
}
void Reverse_init(){
	GPIO_setPadConfig(Reverse, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(Reverse, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(Reverse, GPIO_DIR_MODE_IN);
	GPIO_setControllerCore(Reverse, GPIO_CORE_CPU1);
}
void HALL_A_init(){
	GPIO_setPadConfig(HALL_A, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(HALL_A, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(HALL_A, GPIO_DIR_MODE_IN);
	GPIO_setControllerCore(HALL_A, GPIO_CORE_CPU1);
}
void HALL_B_init(){
	GPIO_setPadConfig(HALL_B, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(HALL_B, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(HALL_B, GPIO_DIR_MODE_IN);
	GPIO_setControllerCore(HALL_B, GPIO_CORE_CPU1);
}
void HALL_C_init(){
	GPIO_setPadConfig(HALL_C, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(HALL_C, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(HALL_C, GPIO_DIR_MODE_IN);
	GPIO_setControllerCore(HALL_C, GPIO_CORE_CPU1);
}
void ESP32PWM_init(){
	GPIO_setPadConfig(ESP32PWM, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(ESP32PWM, GPIO_QUAL_ASYNC);
	GPIO_setDirectionMode(ESP32PWM, GPIO_DIR_MODE_IN);
	GPIO_setControllerCore(ESP32PWM, GPIO_CORE_CPU1);
}

//*****************************************************************************
//
// INPUTXBAR Configurations
//
//*****************************************************************************
void INPUTXBAR_init(){
	myINPUTXBARINPUT0_init();
	ESP32XBAR_init();
}

void myINPUTXBARINPUT0_init(){
	XBAR_setInputPin(myINPUTXBARINPUT0_INPUT, myINPUTXBARINPUT0_SOURCE);
}
void ESP32XBAR_init(){
	XBAR_setInputPin(ESP32XBAR_INPUT, ESP32XBAR_SOURCE);
}

//*****************************************************************************
//
// INTERRUPT Configurations
//
//*****************************************************************************
void INTERRUPT_init(){
	
	// Interrupt Settings for INT_esp32pwm
	// ISR need to be defined for the registered interrupts
	Interrupt_register(INT_esp32pwm, &INT_esp32pwm_ISR);
	Interrupt_enable(INT_esp32pwm);
	
	// Interrupt Settings for INT_PhaseA
	// ISR need to be defined for the registered interrupts
	Interrupt_register(INT_PhaseA, &INT_PhaseA_ISR);
	Interrupt_enable(INT_PhaseA);
	
	// Interrupt Settings for INT_rpmInterrupt
	// ISR need to be defined for the registered interrupts
	Interrupt_register(INT_rpmInterrupt, &INT_rpmInterrupt_ISR);
	Interrupt_enable(INT_rpmInterrupt);
}
//*****************************************************************************
//
// SYNC Scheme Configurations
//
//*****************************************************************************
void SYNC_init(){
	SysCtl_setSyncOutputConfig(SYSCTL_SYNC_OUT_SRC_EPWM1SYNCOUT);
	//
	// For EPWM1, the sync input is: SYSCTL_SYNC_IN_SRC_EXTSYNCIN1
	//
	SysCtl_setSyncInputConfig(SYSCTL_SYNC_IN_EPWM4, SYSCTL_SYNC_IN_SRC_EPWM1SYNCOUT);
	SysCtl_setSyncInputConfig(SYSCTL_SYNC_IN_EPWM7, SYSCTL_SYNC_IN_SRC_EPWM1SYNCOUT);
	SysCtl_setSyncInputConfig(SYSCTL_SYNC_IN_EPWM10, SYSCTL_SYNC_IN_SRC_EPWM1SYNCOUT);
	SysCtl_setSyncInputConfig(SYSCTL_SYNC_IN_ECAP1, SYSCTL_SYNC_IN_SRC_EPWM1SYNCOUT);
	SysCtl_setSyncInputConfig(SYSCTL_SYNC_IN_ECAP4, SYSCTL_SYNC_IN_SRC_EPWM1SYNCOUT);
	//
	// SOCA
	//
	SysCtl_enableExtADCSOCSource(0);
	//
	// SOCB
	//
	SysCtl_enableExtADCSOCSource(0);
}
