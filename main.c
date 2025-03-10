// Ind-E Motor Controller
// Created by Brennan Pinette
// Feb 17 2025
//
// Included Files
//
#include "adc.h"
#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "epwm.h"
#include "stdio.h"
#include "math.h"



__interrupt void INT_PhaseA_ISR(void);


uint16_t adcResult = 2048;

volatile float angle = 0.0f; // in degrees
float frequency = 5.0f;
volatile bool SVPWM_UP = 1;
volatile float duty_cycle = 50.0f; // in percent 
volatile uint16_t CMPA;
volatile uint16_t CMPB;

volatile float V_Alpha;
volatile float V_Beta;
volatile float angle_calculated;

volatile float Va; //
volatile float Vb; 
volatile float Vc;

volatile float Vmax;
volatile float Vmin;
volatile float V_offset;

volatile float T1;
volatile float T2;
volatile float T0;

volatile float modulation_index = 0.5f; // duty_cycle
volatile float V_DC = 30.0f; 
volatile float V_ref;

float pot;






#define PWM_Period 200.0f
#define PWM_ClockDivider 20.0f

#define Duty_Cycle_Scalar 30.0f
#define frequency_rated 60.0f
#define boostV 1.6f






void main(void)
{
    Device_init();               // Initialize the device and clock
    Interrupt_initModule();      // Initialize PIE and clear PIE registers. Disables CPU interrups
    Interrupt_initVectorTable(); // Initialize the PIE vector table with pointers to the shell Interrupt
    Device_initGPIO();
    Board_init();                // Initializes all peripherals configured in SysConfig
    EINT;
    ERTM;


    // Configure GPIO34 and GPIO31 as output pins for LEDs (D1 and D9/D10)
    GPIO_setDirectionMode(34, GPIO_DIR_MODE_OUT);  // D1 - Red LED
    GPIO_setDirectionMode(31, GPIO_DIR_MODE_OUT);  // D9/D10 - Green and Blue LEDs

    // Set initial state for LEDs to off
    GPIO_writePin(34, 0);  // Turn off Red LED (D1)
    GPIO_writePin(31, 0);  // Turn off Green and Blue LEDs (D9/D10)

    
    while(1)
    {
       
        ADC_forceSOC(myADC0_BASE, myADC0_FORCE_SOC0);

        while(ADC_getInterruptStatus(myADC0_BASE,  ADC_INT_NUMBER1) == false){};

        ADC_clearInterruptStatus(myADC0_BASE,  ADC_INT_NUMBER1);

        adcResult = ADC_readResult(myADC0_RESULT_BASE,  myADC0_SOC0); // 0 - 4095

        //printf("%d\r\n", adcResult);
        pot = roundf(adcResult * frequency_rated/4095.0f);

     



        


        GPIO_togglePin(34);  // Toggle Red LED (D1)
        // DEVICE_DELAY_US(500000);  // Delay 500ms

        // // Toggle Green and Blue LEDs (D9/D10)
        // GPIO_togglePin(31);  // Toggle Green and Blue LEDs (D9/D10)
        DEVICE_DELAY_US(50000);  // Delay 50ms
    }
}

__interrupt void INT_PhaseA_ISR(void){

    // Clear the interrupt flag (specific to your interrupt source, e.g., TBCTR_ZERO)
    EALLOW;
    
    SVPWM_UP = !SVPWM_UP; // switch between V0 V1 V2 V7 -> V7 V2 V1 V0 pattern
    
    GPIO_togglePin(myGPIO0);

    frequency = pot;

    //increment angle
    angle = angle + 2.0f * M_PI * frequency * ((PWM_Period * PWM_ClockDivider) / 200000000.0f);

    if(angle >= 2.0f * M_PI){
        angle = angle - 2.0f * M_PI;
    }

    //calculate space vector modulation calculations



    V_ref = boostV * (frequency/frequency_rated) * V_DC/sqrtf(3.0f);
    //V_ref = 17.0f;

    V_Alpha = V_ref * cosf(angle);
    V_Beta = V_ref * sinf(angle);

    // Inverse Clarke

    Va = V_Alpha;
    Vb = ((-1.0f/2.0f) * V_Alpha) + ((sqrtf(3.0f)/2.0f) * V_Beta);
    Vc = ((-1.0f/2.0f) * V_Alpha) - ((sqrtf(3.0f)/2.0f) * V_Beta);

    //min-max injection
    Vmax = fmaxf(Va,Vb);
    Vmax = fmaxf(Vmax,Vc);

    Vmin = fminf(Va,Vb);
    Vmin = fminf(Vmin,Vc);


    V_offset = (Vmax + Vmin)/2.0f;

    Va = Va - V_offset;
    Vb = Vb - V_offset;
    Vc = Vc - V_offset;

    //Clarke transform

    V_Alpha = Va;
    V_Beta = (sqrtf(3.0f))/2.0f * (Vb - Vc);

    angle_calculated = atan2f(V_Beta,V_Alpha);

    if(angle_calculated < 0.0f){
        angle_calculated = angle_calculated + 2*M_PI;
    }





    if(SVPWM_UP){





        if(angle_calculated < 60.0f*M_PI/180.0f){ // V1 V2 V7 //60 deg

    

            T1 = (sqrtf(3.0f)*PWM_Period*V_ref*sinf(60.0f*M_PI/180.0f - angle_calculated))/V_DC; //V1
            T2 = (sqrtf(3.0f)*PWM_Period*V_ref*sinf(angle_calculated - 0.0f*M_PI/180.0f ))/V_DC; //V2
            T0 = (PWM_Period - T1 - T2);

            CMPA = roundf(T0);
            CMPB = roundf(T1+T0);   


            EPWM_setCounterCompareValue(PhaseA_BASE, EPWM_COUNTER_COMPARE_A, CMPA);	
            EPWM_setCounterCompareValue(PhaseA_BASE, EPWM_COUNTER_COMPARE_B, CMPB);

            EPWM_setCounterCompareValue(PhaseB_BASE, EPWM_COUNTER_COMPARE_A, CMPA);	
            EPWM_setCounterCompareValue(PhaseB_BASE, EPWM_COUNTER_COMPARE_B, CMPB);

            EPWM_setCounterCompareValue(PhaseC_BASE, EPWM_COUNTER_COMPARE_A, CMPA);	
            EPWM_setCounterCompareValue(PhaseC_BASE, EPWM_COUNTER_COMPARE_B, CMPB);	

            //PHASE A 1 1 1 
            EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
            EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
            EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	



            //PHASE B 0 1 1
            EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
            EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
            EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	



            //PHASE C 0 0 1
            EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
            EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
            EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
        }
        else if (angle_calculated >= 60.0f*M_PI/180.0f && angle_calculated < 120.0f*M_PI/180.0f){ //V2 V3 V7


            T1 = (sqrtf(3.0f)*PWM_Period*V_ref*sinf(120.0f*M_PI/180.0f - angle_calculated))/V_DC; //V1
            T2 = (sqrtf(3.0f)*PWM_Period*V_ref*sinf(angle_calculated - 60.0f*M_PI/180.0f ))/V_DC; //V2
            T0 = (PWM_Period - T1 - T2);

            CMPA = roundf(T0);
            CMPB = roundf(T1+T0);   


            EPWM_setCounterCompareValue(PhaseA_BASE, EPWM_COUNTER_COMPARE_A, CMPA);	
            EPWM_setCounterCompareValue(PhaseA_BASE, EPWM_COUNTER_COMPARE_B, CMPB);

            EPWM_setCounterCompareValue(PhaseB_BASE, EPWM_COUNTER_COMPARE_A, CMPA);	
            EPWM_setCounterCompareValue(PhaseB_BASE, EPWM_COUNTER_COMPARE_B, CMPB);

            EPWM_setCounterCompareValue(PhaseC_BASE, EPWM_COUNTER_COMPARE_A, CMPA);	
            EPWM_setCounterCompareValue(PhaseC_BASE, EPWM_COUNTER_COMPARE_B, CMPB);	

            //PHASE A 1 0 1 
            EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
            EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
            EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	



            //PHASE B 1 1 1
            EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
            EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
            EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	



            //PHASE C 0 0 1
            EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
            EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
            EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);

        } 
        else if (angle_calculated >= 120.0f*M_PI/180.0f && angle_calculated < 180.0f*M_PI/180.0f){ //V3 V4 V7


            T1 = (sqrtf(3.0f)*PWM_Period*V_ref*sinf(180.0f*M_PI/180.0f - angle_calculated))/V_DC; //V1
            T2 = (sqrtf(3.0f)*PWM_Period*V_ref*sinf(angle_calculated - 120.0f*M_PI/180.0f ))/V_DC; //V2
            T0 = (PWM_Period - T1 - T2);

            CMPA = roundf(T0);
            CMPB = roundf(T1+T0);   


            EPWM_setCounterCompareValue(PhaseA_BASE, EPWM_COUNTER_COMPARE_A, CMPA);	
            EPWM_setCounterCompareValue(PhaseA_BASE, EPWM_COUNTER_COMPARE_B, CMPB);

            EPWM_setCounterCompareValue(PhaseB_BASE, EPWM_COUNTER_COMPARE_A, CMPA);	
            EPWM_setCounterCompareValue(PhaseB_BASE, EPWM_COUNTER_COMPARE_B, CMPB);

            EPWM_setCounterCompareValue(PhaseC_BASE, EPWM_COUNTER_COMPARE_A, CMPA);	
            EPWM_setCounterCompareValue(PhaseC_BASE, EPWM_COUNTER_COMPARE_B, CMPB);	

            //PHASE A 0 0 1 
            EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
            EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
            EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	



            //PHASE B 1 1 1
            EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
            EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
            EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	



            //PHASE C 0 1 1
            EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
            EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
            EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);

        }
        else if (angle_calculated >= 180.0f*M_PI/180.0f && angle_calculated < 240.0f*M_PI/180.0f){ //V4 V5 V7

            T1 = (sqrtf(3.0f)*PWM_Period*V_ref*sinf(240.0f*M_PI/180.0f - angle_calculated))/V_DC; //V1
            T2 = (sqrtf(3.0f)*PWM_Period*V_ref*sinf(angle_calculated - 180.0f*M_PI/180.0f ))/V_DC; //V2
            T0 = (PWM_Period - T1 - T2);

            CMPA = roundf(T0);
            CMPB = roundf(T1+T0);   


            EPWM_setCounterCompareValue(PhaseA_BASE, EPWM_COUNTER_COMPARE_A, CMPA);	
            EPWM_setCounterCompareValue(PhaseA_BASE, EPWM_COUNTER_COMPARE_B, CMPB);

            EPWM_setCounterCompareValue(PhaseB_BASE, EPWM_COUNTER_COMPARE_A, CMPA);	
            EPWM_setCounterCompareValue(PhaseB_BASE, EPWM_COUNTER_COMPARE_B, CMPB);

            EPWM_setCounterCompareValue(PhaseC_BASE, EPWM_COUNTER_COMPARE_A, CMPA);	
            EPWM_setCounterCompareValue(PhaseC_BASE, EPWM_COUNTER_COMPARE_B, CMPB);	

            //PHASE A 0 0 1 
            EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
            EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
            EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	



            //PHASE B 1 0 1
            EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
            EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
            EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	


            
            //PHASE C 1 1 1
            EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
            EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
            EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);

        }
        else if (angle_calculated >= 240.0f*M_PI/180.0f && angle_calculated < 300.0f*M_PI/180.0f){ //V5 V6 V7

            T1 = (sqrtf(3.0f)*PWM_Period*V_ref*sinf(300.0f*M_PI/180.0f - angle_calculated))/V_DC; //V1
            T2 = (sqrtf(3.0f)*PWM_Period*V_ref*sinf(angle_calculated - 240.0f*M_PI/180.0f ))/V_DC; //V2
            T0 = (PWM_Period - T1 - T2);

            CMPA = roundf(T0);
            CMPB = roundf(T1+T0);   


            EPWM_setCounterCompareValue(PhaseA_BASE, EPWM_COUNTER_COMPARE_A, CMPA);	
            EPWM_setCounterCompareValue(PhaseA_BASE, EPWM_COUNTER_COMPARE_B, CMPB);

            EPWM_setCounterCompareValue(PhaseB_BASE, EPWM_COUNTER_COMPARE_A, CMPA);	
            EPWM_setCounterCompareValue(PhaseB_BASE, EPWM_COUNTER_COMPARE_B, CMPB);

            EPWM_setCounterCompareValue(PhaseC_BASE, EPWM_COUNTER_COMPARE_A, CMPA);	
            EPWM_setCounterCompareValue(PhaseC_BASE, EPWM_COUNTER_COMPARE_B, CMPB);	

            //PHASE A 0 1 1 
            EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
            EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
            EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	



            //PHASE B 0 0 1
            EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
            EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
            EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	



            //PHASE C 1 1 1
            EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
            EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
            EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);

        }
        else if (angle_calculated >= 300.0f*M_PI/180.0f && angle_calculated < 360.0f*M_PI/180.0f){ //V6 V1 V7

            T1 = (sqrtf(3.0f)*PWM_Period*V_ref*sinf(360.0f*M_PI/180.0f - angle_calculated))/V_DC; //V1
            T2 = (sqrtf(3.0f)*PWM_Period*V_ref*sinf(angle_calculated - 300.0f*M_PI/180.0f ))/V_DC; //V2
            T0 = (PWM_Period - T1 - T2);

            CMPA = roundf(T0);
            CMPB = roundf(T1+T0);   


            EPWM_setCounterCompareValue(PhaseA_BASE, EPWM_COUNTER_COMPARE_A, CMPA);	
            EPWM_setCounterCompareValue(PhaseA_BASE, EPWM_COUNTER_COMPARE_B, CMPB);

            EPWM_setCounterCompareValue(PhaseB_BASE, EPWM_COUNTER_COMPARE_A, CMPA);	
            EPWM_setCounterCompareValue(PhaseB_BASE, EPWM_COUNTER_COMPARE_B, CMPB);

            EPWM_setCounterCompareValue(PhaseC_BASE, EPWM_COUNTER_COMPARE_A, CMPA);	
            EPWM_setCounterCompareValue(PhaseC_BASE, EPWM_COUNTER_COMPARE_B, CMPB);	

            //PHASE A 1 1 1 
            EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
            EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
            EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	



            //PHASE B 0 0 1
            EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
            EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
            EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	



            //PHASE C 1 0 1
            EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
            EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
            EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);

        }
    }
    else { // counting down


        

        if(angle_calculated < 60.0f*M_PI/180.0f){ // V2 V1 V0

            T1 = (sqrtf(3.0f)*PWM_Period*V_ref*sinf(60.0f*M_PI/180.0f - angle_calculated))/V_DC; //V1
            T2 = (sqrtf(3.0f)*PWM_Period*V_ref*sinf(angle_calculated - 0.0f*M_PI/180.0f))/V_DC; //V2
            T0 = (PWM_Period - T1 - T2);

            CMPA = roundf(T0);
            CMPB = roundf(T2+T0);   
            

            EPWM_setCounterCompareValue(PhaseC_BASE, EPWM_COUNTER_COMPARE_A, CMPA);	
            EPWM_setCounterCompareValue(PhaseC_BASE, EPWM_COUNTER_COMPARE_B, CMPB);	
            EPWM_setCounterCompareValue(PhaseB_BASE, EPWM_COUNTER_COMPARE_A, CMPA);	
            EPWM_setCounterCompareValue(PhaseB_BASE, EPWM_COUNTER_COMPARE_B, CMPB);
            EPWM_setCounterCompareValue(PhaseA_BASE, EPWM_COUNTER_COMPARE_A, CMPA);	
            EPWM_setCounterCompareValue(PhaseA_BASE, EPWM_COUNTER_COMPARE_B, CMPB);


    
            //PHASE A 1 1 0
            EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
            EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
            EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	


            //PHASE B 1 0 0
            EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
            EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
            EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	


            //PHASE C 0 0 0
            EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
            EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
            EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);


        }
        else if (angle_calculated >= 60.0f*M_PI/180.0f && angle_calculated < 120.0f*M_PI/180.0f){ //V3 V2 V0

            T1 = (sqrtf(3.0f)*PWM_Period*V_ref*sinf(120.0f*M_PI/180.0f - angle_calculated))/V_DC; //V1
            T2 = (sqrtf(3.0f)*PWM_Period*V_ref*sinf(angle_calculated - 60.0f*M_PI/180.0f))/V_DC; //V2
            T0 = (PWM_Period - T1 - T2);

            CMPA = roundf(T0);
            CMPB = roundf(T2+T0);   
            

            EPWM_setCounterCompareValue(PhaseC_BASE, EPWM_COUNTER_COMPARE_A, CMPA);	
            EPWM_setCounterCompareValue(PhaseC_BASE, EPWM_COUNTER_COMPARE_B, CMPB);	
            EPWM_setCounterCompareValue(PhaseB_BASE, EPWM_COUNTER_COMPARE_A, CMPA);	
            EPWM_setCounterCompareValue(PhaseB_BASE, EPWM_COUNTER_COMPARE_B, CMPB);
            EPWM_setCounterCompareValue(PhaseA_BASE, EPWM_COUNTER_COMPARE_A, CMPA);	
            EPWM_setCounterCompareValue(PhaseA_BASE, EPWM_COUNTER_COMPARE_B, CMPB);

            //PHASE A 0 1 0 
            EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
            EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
            EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	



            //PHASE B 1 1 0
            EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
            EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
            EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	



            //PHASE C 0 0 0
            EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
            EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
            EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);


        } 
        else if (angle_calculated >= 120.0f*M_PI/180.0f && angle_calculated < 180.0f*M_PI/180.0f){ //V4 V3 V0

            T1 = (sqrtf(3.0f)*PWM_Period*V_ref*sinf(180.0f*M_PI/180.0f - angle_calculated))/V_DC; //V1
            T2 = (sqrtf(3.0f)*PWM_Period*V_ref*sinf(angle_calculated - 120.0f*M_PI/180.0f))/V_DC; //V2
            T0 = (PWM_Period - T1 - T2);

            CMPA = roundf(T0);
            CMPB = roundf(T2+T0);   
            

            EPWM_setCounterCompareValue(PhaseC_BASE, EPWM_COUNTER_COMPARE_A, CMPA);	
            EPWM_setCounterCompareValue(PhaseC_BASE, EPWM_COUNTER_COMPARE_B, CMPB);	
            EPWM_setCounterCompareValue(PhaseB_BASE, EPWM_COUNTER_COMPARE_A, CMPA);	
            EPWM_setCounterCompareValue(PhaseB_BASE, EPWM_COUNTER_COMPARE_B, CMPB);
            EPWM_setCounterCompareValue(PhaseA_BASE, EPWM_COUNTER_COMPARE_A, CMPA);	
            EPWM_setCounterCompareValue(PhaseA_BASE, EPWM_COUNTER_COMPARE_B, CMPB);

            //PHASE A 0 0 0
            EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
            EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
            EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	



            //PHASE B 1 1 0
            EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
            EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
            EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	



            //PHASE C 1 0 0
            EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
            EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
            EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);


        }
        else if (angle_calculated >= 180.0f*M_PI/180.0f && angle_calculated < 240.0f*M_PI/180.0f){ //V5 V4 V0

            T1 = (sqrtf(3.0f)*PWM_Period*V_ref*sinf(240.0f*M_PI/180.0f - angle_calculated))/V_DC; //V1
            T2 = (sqrtf(3.0f)*PWM_Period*V_ref*sinf(angle_calculated - 180.0f*M_PI/180.0f))/V_DC; //V2
            T0 = (PWM_Period - T1 - T2);

            CMPA = roundf(T0);
            CMPB = roundf(T2+T0);   
            

            EPWM_setCounterCompareValue(PhaseC_BASE, EPWM_COUNTER_COMPARE_A, CMPA);	
            EPWM_setCounterCompareValue(PhaseC_BASE, EPWM_COUNTER_COMPARE_B, CMPB);	
            EPWM_setCounterCompareValue(PhaseB_BASE, EPWM_COUNTER_COMPARE_A, CMPA);	
            EPWM_setCounterCompareValue(PhaseB_BASE, EPWM_COUNTER_COMPARE_B, CMPB);
            EPWM_setCounterCompareValue(PhaseA_BASE, EPWM_COUNTER_COMPARE_A, CMPA);	
            EPWM_setCounterCompareValue(PhaseA_BASE, EPWM_COUNTER_COMPARE_B, CMPB);

            //PHASE A 0 0 0
            EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
            EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
            EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	



            //PHASE B 0 1 0
            EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
            EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
            EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	



            //PHASE C 1 1 0
            EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
            EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
            EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);


        }
        else if (angle_calculated >= 240.0f*M_PI/180.0f && angle_calculated < 300.0f*M_PI/180.0f){ //V6 V5 V0

            T1 = (sqrtf(3.0f)*PWM_Period*V_ref*sinf(300.0f*M_PI/180.0f - angle_calculated))/V_DC; //V1
            T2 = (sqrtf(3.0f)*PWM_Period*V_ref*sinf(angle_calculated - 240.0f*M_PI/180.0f))/V_DC; //V2
            T0 = (PWM_Period - T1 - T2);

            CMPA = roundf(T0);
            CMPB = roundf(T2+T0);   
            

            EPWM_setCounterCompareValue(PhaseC_BASE, EPWM_COUNTER_COMPARE_A, CMPA);	
            EPWM_setCounterCompareValue(PhaseC_BASE, EPWM_COUNTER_COMPARE_B, CMPB);	
            EPWM_setCounterCompareValue(PhaseB_BASE, EPWM_COUNTER_COMPARE_A, CMPA);	
            EPWM_setCounterCompareValue(PhaseB_BASE, EPWM_COUNTER_COMPARE_B, CMPB);
            EPWM_setCounterCompareValue(PhaseA_BASE, EPWM_COUNTER_COMPARE_A, CMPA);	
            EPWM_setCounterCompareValue(PhaseA_BASE, EPWM_COUNTER_COMPARE_B, CMPB);

            //PHASE A 1 0 0
            EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
            EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
            EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	



            //PHASE B 0 0 0
            EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
            EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
            EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	



            //PHASE C 1 1 0
            EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
            EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
            EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);


        }
        else if (angle_calculated >= 300.0f*M_PI/180.0f && angle_calculated < 360.0f*M_PI/180.0f){ //V1 V6 V0

            T1 = (sqrtf(3.0f)*PWM_Period*V_ref*sinf(360.0f*M_PI/180.0f - angle_calculated))/V_DC; //V1
            T2 = (sqrtf(3.0f)*PWM_Period*V_ref*sinf(angle_calculated - 300.0f*M_PI/180.0f))/V_DC; //V2
            T0 = (PWM_Period - T1 - T2);

            CMPA = roundf(T0);
            CMPB = roundf(T2+T0);   
            

            EPWM_setCounterCompareValue(PhaseC_BASE, EPWM_COUNTER_COMPARE_A, CMPA);	
            EPWM_setCounterCompareValue(PhaseC_BASE, EPWM_COUNTER_COMPARE_B, CMPB);	
            EPWM_setCounterCompareValue(PhaseB_BASE, EPWM_COUNTER_COMPARE_A, CMPA);	
            EPWM_setCounterCompareValue(PhaseB_BASE, EPWM_COUNTER_COMPARE_B, CMPB);
            EPWM_setCounterCompareValue(PhaseA_BASE, EPWM_COUNTER_COMPARE_A, CMPA);	
            EPWM_setCounterCompareValue(PhaseA_BASE, EPWM_COUNTER_COMPARE_B, CMPB);

            //PHASE A 1 1 0
            EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
            EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
            EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	



            //PHASE B 0 0 0
            EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
            EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
            EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	



            //PHASE C 0 1 0
            EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
            EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
            EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);


        }


    }



    EPWM_clearEventTriggerInterruptFlag(PhaseA_BASE);
    // Acknowledge the interrupt in PIE (this is crucial for clearing the interrupt in PIE)
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
    
    EDIS;
}

//
// End of File
//
