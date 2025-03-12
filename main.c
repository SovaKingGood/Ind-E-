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
#include "CAN_setup.h"


//SovaKingGOod
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
volatile uint8_t A_CMPA_UP, A_CMPB_UP, B_CMPA_UP, B_CMPB_UP, C_CMPA_UP, C_CMPB_UP;

volatile float T1;
volatile float T2;
volatile float T0;


volatile float V_DC = 12.0f; 
volatile float V_ref;
volatile float boostV = 0.3f;

union {
    float f;
    uint8_t words[4];
} floatUnion;

volatile union {
    float f;
    uint8_t words[4];
} pot1;


volatile union {
    float f;
    uint8_t words[4];
} pot2;


//float pot[1];
//float pot2[2];
volatile float PWM_Period = 7000.0f;
#define PWM_ClockDivider 1 // 128 * 8
#define frequency_rated 60.0f // Max Frequency
//#define deadband 4

volatile uint16_t deadband = 96;


#define EPWM_CLOCK 100000000 // System Clock divide by 2: 200MHz / 2 = 100MHz


__interrupt void INT_PhaseA_ISR(void);
void changeEPWMcompare();


void main(void)
{
    Device_init();               // Initialize the device and clock
    Interrupt_initModule();      // Initialize PIE and clear PIE registers. Disables CPU interrups
    Interrupt_initVectorTable(); // Initialize the PIE vector table with pointers to the shell Interrupt
    Device_initGPIO();
    Board_init();                // Initializes all peripherals configured in SysConfig
    EINT;
    ERTM;
    GPIO_setDirectionMode(34, GPIO_DIR_MODE_OUT);  // D1 - Red LED
    GPIO_writePin(34, 0);  // Turn off Red LED (D1)
    CAN_send_setup();


    


    //sync epwm
    // Manually configure EPWM2 and EPWM3 to sync to EPWM1's SYNCOUT

    while(1)
    {
       



        GPIO_togglePin(34);  // Toggle Blue LED (D1)

        //send frequency
        //CAN_sendMessage(CANB_BASE, TX_MSG_OBJ_ID5,4,pot1.words);
        //poll for reception of remote frame
        //while(((HWREGH(CANB_BASE + CAN_O_ES) & CAN_ES_TXOK)) !=  CAN_ES_TXOK){}

        

        //DEVICE_DELAY_US(50000);  // Delay 500ms

        //send
        //CAN_sendMessage(CANB_BASE, TX_MSG_OBJ_ID6,4,pot2.words);
        //poll for reception of remote frame
        //while(((HWREGH(CANB_BASE + CAN_O_ES) & CAN_ES_TXOK)) !=  CAN_ES_TXOK){}
        //adjustable delays
 
        DEVICE_DELAY_US(50000);  // Delay 500ms
    }
}


__interrupt void INT_PhaseA_ISR(void){

    //EPWM_setTimeBaseCounter(PhaseC_BASE, 5);	
    //EPWM_setTimeBaseCounter(PhaseB_BASE, 5);	
    //EPWM_forceSyncPulse(PhaseA_BASE);



    EALLOW;
    //EPWM_setGlobalLoadOneShotLatch(PhaseA_BASE);
    


    
    adcResult = ADC_readResult(myADC0_RESULT_BASE,  myADC0_SOC0); // 0 - 4095

    //printf("%d\r\n", adcResult);
    pot1.f = adcResult * frequency_rated/4095.0f;


    adcResult = ADC_readResult(myADC0_RESULT_BASE,  myADC0_SOC1); // 0 - 4095

    pot2.f = adcResult * 0.9f/4095.0f;


    //deadband = roundf(pot2.f); //amplutide set by pot2

    EPWM_setRisingEdgeDelayCount(PhaseA_BASE, deadband);
    EPWM_setRisingEdgeDelayCount(PhaseB_BASE, deadband);	
    EPWM_setRisingEdgeDelayCount(PhaseC_BASE, deadband);

    EPWM_setFallingEdgeDelayCount(PhaseA_BASE, deadband);	
    EPWM_setFallingEdgeDelayCount(PhaseB_BASE, deadband);	
    EPWM_setFallingEdgeDelayCount(PhaseC_BASE, deadband);	

    SVPWM_UP = !SVPWM_UP; // switch between V0 V1 V2 -> V7 V2 V1 pattern
    
    GPIO_togglePin(myGPIO0); // verify interrupt is going through

    frequency = pot1.f; //frequency set by pot
    boostV = pot2.f;
    //deadband = pot2;
    //boostV = 0.3f;
    angle = angle + 2.0f * M_PI * frequency * ((PWM_Period * PWM_ClockDivider) / EPWM_CLOCK);   //increment angle based on time past

    if(angle >= 2.0f * M_PI){ //If angle is greater than 2pi subtract by 2pi
        angle = angle - 2.0f * M_PI;
    }

    //minmax injection calculations
    
    V_ref = boostV * V_DC/sqrtf(3.0f);

    if(V_ref >= V_DC/sqrtf(3.0f)){ //ensure no overmodulatioon
        V_ref = V_DC/sqrtf(3.0f);
    }
    //V_ref = 17.0f;

    // V_Alpha = V_ref * cosf(angle);
    // V_Beta = V_ref * sinf(angle);

    // // Inverse Clarke

    // Va = V_Alpha;
    // Vb = ((-1.0f/2.0f) * V_Alpha) + ((sqrtf(3.0f)/2.0f) * V_Beta);
    // Vc = ((-1.0f/2.0f) * V_Alpha) - ((sqrtf(3.0f)/2.0f) * V_Beta);

    // //min-max injection
    // Vmax = fmaxf(Va,Vb);
    // Vmax = fmaxf(Vmax,Vc);

    // Vmin = fminf(Va,Vb);
    // Vmin = fminf(Vmin,Vc);


    // V_offset = (Vmax + Vmin)/2.0f;

    // Va = Va - V_offset;
    // Vb = Vb - V_offset;
    // Vc = Vc - V_offset;

    // //Clarke transform

    // V_Alpha = Va;
    // V_Beta = (sqrtf(3.0f))/2.0f * (Vb - Vc);

    // angle_calculated = atan2f(V_Beta,V_Alpha);

    // V_ref = sqrtf(V_Alpha * V_Alpha + V_Beta * V_Beta);

    angle_calculated = angle;

    // if(angle_calculated < 0.0f){
    //     angle_calculated = angle_calculated + 2*M_PI;
    // }

    if(angle_calculated < 60.0f*M_PI/180.0f){ // If in sector 1    0-60deg
        T1 = (sqrtf(3.0f)*PWM_Period*V_ref*sinf(60.0f*M_PI/180.0f - angle_calculated))/V_DC; //V1
        T2 = (sqrtf(3.0f)*PWM_Period*V_ref*sinf(angle_calculated - 0.0f*M_PI/180.0f ))/V_DC; //V2
        T0 = (PWM_Period - T1 - T2);
        CMPA = roundf(T0);
        // set AQ states
        if(SVPWM_UP){ //V1 V2 V7
            CMPB = roundf(T1+T0);
            //V1 1 0 0
            A_CMPA_UP = EPWM_AQ_OUTPUT_HIGH;
            B_CMPA_UP = EPWM_AQ_OUTPUT_LOW;
            C_CMPA_UP = EPWM_AQ_OUTPUT_LOW;
            //V2 1 1 0
            A_CMPB_UP = EPWM_AQ_OUTPUT_HIGH;
            B_CMPB_UP = EPWM_AQ_OUTPUT_HIGH;
            C_CMPB_UP = EPWM_AQ_OUTPUT_LOW;
        }
        else{ //V2 V1 V0
            CMPB = roundf(T2+T0);
            //V2 1 1 0
            A_CMPA_UP = EPWM_AQ_OUTPUT_HIGH;
            B_CMPA_UP = EPWM_AQ_OUTPUT_HIGH;
            C_CMPA_UP = EPWM_AQ_OUTPUT_LOW;
            //V1 1 0 0
            A_CMPB_UP = EPWM_AQ_OUTPUT_HIGH;
            B_CMPB_UP = EPWM_AQ_OUTPUT_LOW;
            C_CMPB_UP = EPWM_AQ_OUTPUT_LOW;
        }  
    }
    else if (angle_calculated >= 60.0f*M_PI/180.0f && angle_calculated < 120.0f*M_PI/180.0f){ //V2 V3 V7
        T1 = (sqrtf(3.0f)*PWM_Period*V_ref*sinf(120.0f*M_PI/180.0f - angle_calculated))/V_DC; //V1
        T2 = (sqrtf(3.0f)*PWM_Period*V_ref*sinf(angle_calculated - 60.0f*M_PI/180.0f ))/V_DC; //V2
        T0 = (PWM_Period - T1 - T2);
        CMPA = roundf(T0);
        if(SVPWM_UP){ //V2 V3 V7
            CMPB = roundf(T1+T0);
            //V2 1 1 0
            A_CMPA_UP = EPWM_AQ_OUTPUT_HIGH;
            B_CMPA_UP = EPWM_AQ_OUTPUT_HIGH;
            C_CMPA_UP = EPWM_AQ_OUTPUT_LOW;
            //V3 0 1 0
            A_CMPB_UP = EPWM_AQ_OUTPUT_LOW;
            B_CMPB_UP = EPWM_AQ_OUTPUT_HIGH;
            C_CMPB_UP = EPWM_AQ_OUTPUT_LOW;
        }
        else{ //V2 V1 V0
            CMPB = roundf(T2+T0);
            //V3 0 1 0
            A_CMPA_UP = EPWM_AQ_OUTPUT_LOW;
            B_CMPA_UP = EPWM_AQ_OUTPUT_HIGH;
            C_CMPA_UP = EPWM_AQ_OUTPUT_LOW;
            //V2 1 1 0
            A_CMPB_UP = EPWM_AQ_OUTPUT_HIGH;
            B_CMPB_UP = EPWM_AQ_OUTPUT_HIGH;
            C_CMPB_UP = EPWM_AQ_OUTPUT_LOW;
        }
    } 
    else if (angle_calculated >= 120.0f*M_PI/180.0f && angle_calculated < 180.0f*M_PI/180.0f){ //V3 V4 V7
        T1 = (sqrtf(3.0f)*PWM_Period*V_ref*sinf(180.0f*M_PI/180.0f - angle_calculated))/V_DC; //V1
        T2 = (sqrtf(3.0f)*PWM_Period*V_ref*sinf(angle_calculated - 120.0f*M_PI/180.0f ))/V_DC; //V2
        T0 = (PWM_Period - T1 - T2);
        CMPA = roundf(T0);
        if(SVPWM_UP){ //V3 V4 V7
            CMPB = roundf(T1+T0);
            //V3 0 1 0
            A_CMPA_UP = EPWM_AQ_OUTPUT_LOW;
            B_CMPA_UP = EPWM_AQ_OUTPUT_HIGH;
            C_CMPA_UP = EPWM_AQ_OUTPUT_LOW;
            //V4 0 1 1
            A_CMPB_UP = EPWM_AQ_OUTPUT_LOW;
            B_CMPB_UP = EPWM_AQ_OUTPUT_HIGH;
            C_CMPB_UP = EPWM_AQ_OUTPUT_HIGH;
        }
        else{ //V2 V1 V0
            CMPB = roundf(T2+T0);
            //V4 0 1 1
            A_CMPA_UP = EPWM_AQ_OUTPUT_LOW;
            B_CMPA_UP = EPWM_AQ_OUTPUT_HIGH;
            C_CMPA_UP = EPWM_AQ_OUTPUT_HIGH;
            //V3 0 1 0
            A_CMPB_UP = EPWM_AQ_OUTPUT_LOW;
            B_CMPB_UP = EPWM_AQ_OUTPUT_HIGH;
            C_CMPB_UP = EPWM_AQ_OUTPUT_LOW;
        }
    }
    else if (angle_calculated >= 180.0f*M_PI/180.0f && angle_calculated < 240.0f*M_PI/180.0f){ //V4 V5 V7
        T1 = (sqrtf(3.0f)*PWM_Period*V_ref*sinf(240.0f*M_PI/180.0f - angle_calculated))/V_DC; //V1
        T2 = (sqrtf(3.0f)*PWM_Period*V_ref*sinf(angle_calculated - 180.0f*M_PI/180.0f ))/V_DC; //V2
        T0 = (PWM_Period - T1 - T2);
        CMPA = roundf(T0);
        if(SVPWM_UP){ //V4 V5 V7
            CMPB = roundf(T1+T0);
            //V4 0 1 1
            A_CMPA_UP = EPWM_AQ_OUTPUT_LOW;
            B_CMPA_UP = EPWM_AQ_OUTPUT_HIGH;
            C_CMPA_UP = EPWM_AQ_OUTPUT_HIGH;
            //V5 0 0 1
            A_CMPB_UP = EPWM_AQ_OUTPUT_LOW;
            B_CMPB_UP = EPWM_AQ_OUTPUT_LOW;
            C_CMPB_UP = EPWM_AQ_OUTPUT_HIGH;
        }
        else{ //V5 V4 V0
            CMPB = roundf(T2+T0);
            //V5 0 0 1
            A_CMPA_UP = EPWM_AQ_OUTPUT_LOW;
            B_CMPA_UP = EPWM_AQ_OUTPUT_LOW;
            C_CMPA_UP = EPWM_AQ_OUTPUT_HIGH;
            //V4 0 1 1
            A_CMPB_UP = EPWM_AQ_OUTPUT_LOW;
            B_CMPB_UP = EPWM_AQ_OUTPUT_HIGH;
            C_CMPB_UP = EPWM_AQ_OUTPUT_HIGH;
        }
    }
    else if (angle_calculated >= 240.0f*M_PI/180.0f && angle_calculated < 300.0f*M_PI/180.0f){ //V5 V6 V7
        T1 = (sqrtf(3.0f)*PWM_Period*V_ref*sinf(300.0f*M_PI/180.0f - angle_calculated))/V_DC; //V1
        T2 = (sqrtf(3.0f)*PWM_Period*V_ref*sinf(angle_calculated - 240.0f*M_PI/180.0f ))/V_DC; //V2
        T0 = (PWM_Period - T1 - T2);
        CMPA = roundf(T0);
        if(SVPWM_UP){ //V5 V6 V7
            CMPB = roundf(T1+T0);
            //V5 0 0 1
            A_CMPA_UP = EPWM_AQ_OUTPUT_LOW;
            B_CMPA_UP = EPWM_AQ_OUTPUT_LOW;
            C_CMPA_UP = EPWM_AQ_OUTPUT_HIGH;
            //V6 1 0 1
            A_CMPB_UP = EPWM_AQ_OUTPUT_HIGH;
            B_CMPB_UP = EPWM_AQ_OUTPUT_LOW;
            C_CMPB_UP = EPWM_AQ_OUTPUT_HIGH;
        }
        else{ //V5 V4 V0
            CMPB = roundf(T2+T0);
            //V6 1 0 1
            A_CMPA_UP = EPWM_AQ_OUTPUT_HIGH;
            B_CMPA_UP = EPWM_AQ_OUTPUT_LOW;
            C_CMPA_UP = EPWM_AQ_OUTPUT_HIGH;
            //V5 0 0 1
            A_CMPB_UP = EPWM_AQ_OUTPUT_LOW;
            B_CMPB_UP = EPWM_AQ_OUTPUT_LOW;
            C_CMPB_UP = EPWM_AQ_OUTPUT_HIGH;
        }
    }
    else if (angle_calculated >= 300.0f*M_PI/180.0f && angle_calculated < 360.0f*M_PI/180.0f){ //V6 V1 V7
        T1 = (sqrtf(3.0f)*PWM_Period*V_ref*sinf(360.0f*M_PI/180.0f - angle_calculated))/V_DC; //V1
        T2 = (sqrtf(3.0f)*PWM_Period*V_ref*sinf(angle_calculated - 300.0f*M_PI/180.0f ))/V_DC; //V2
        T0 = (PWM_Period - T1 - T2);
        CMPA = roundf(T0);
        if(SVPWM_UP){ //V6 V1 V7
            CMPB = roundf(T1+T0);
            //V6 1 0 1
            A_CMPA_UP = EPWM_AQ_OUTPUT_HIGH;
            B_CMPA_UP = EPWM_AQ_OUTPUT_LOW;
            C_CMPA_UP = EPWM_AQ_OUTPUT_HIGH;
            //V1 1 0 0
            A_CMPB_UP = EPWM_AQ_OUTPUT_HIGH;
            B_CMPB_UP = EPWM_AQ_OUTPUT_LOW;
            C_CMPB_UP = EPWM_AQ_OUTPUT_LOW;
        }
        else{ //V5 V4 V0
            CMPB = roundf(T2+T0);
            //V1 1 0 0
            A_CMPA_UP = EPWM_AQ_OUTPUT_HIGH;
            B_CMPA_UP = EPWM_AQ_OUTPUT_LOW;
            C_CMPA_UP = EPWM_AQ_OUTPUT_LOW;
            //V6 1 0 1
            A_CMPB_UP = EPWM_AQ_OUTPUT_HIGH;
            B_CMPB_UP = EPWM_AQ_OUTPUT_LOW;
            C_CMPB_UP = EPWM_AQ_OUTPUT_HIGH;
        }
    }

    changeEPWMcompare(); //Update EPWM compare values
    EPWM_clearEventTriggerInterruptFlag(PhaseA_BASE);
    // Acknowledge the interrupt in PIE (this is crucial for clearing the interrupt in PIE)
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
    
    EDIS;
}


void changeEPWMcompare(){
    //Fill EPWM registers compare values with new calculated values

    EPWM_setCounterCompareValue(PhaseA_BASE, EPWM_COUNTER_COMPARE_A, CMPA);	
    EPWM_setCounterCompareValue(PhaseA_BASE, EPWM_COUNTER_COMPARE_B, CMPB);

    EPWM_setCounterCompareValue(PhaseB_BASE, EPWM_COUNTER_COMPARE_A, CMPA);	
    EPWM_setCounterCompareValue(PhaseB_BASE, EPWM_COUNTER_COMPARE_B, CMPB);

    EPWM_setCounterCompareValue(PhaseC_BASE, EPWM_COUNTER_COMPARE_A, CMPA);	
    EPWM_setCounterCompareValue(PhaseC_BASE, EPWM_COUNTER_COMPARE_B, CMPB);	
    //EPWM_AQ_OUTPUT_NO_CHANGE = 0,  //!< No change in the output pins
    //EPWM_AQ_OUTPUT_LOW       = 1,  //!< Set output pins to low
    //EPWM_AQ_OUTPUT_HIGH      = 2,  //!< Set output pins to High
    //EPWM_AQ_OUTPUT_TOGGLE    = 3   //!< Toggle the output pins

    //Phase A
    EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, A_CMPA_UP, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, A_CMPB_UP, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
    //Phase B
    EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, B_CMPA_UP, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
    EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, B_CMPB_UP, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
    //Phase C
    EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, C_CMPA_UP, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
    EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, C_CMPB_UP, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	

    //Null states
    if( CMPA == 0 || CMPB == 0){ //If max duty cycle no need for null states

        EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	
        EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
        
        EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	
        EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

        EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	
        EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    }
    else if(SVPWM_UP){ //period will go to state V7
        EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
        //EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

        EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
        //EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

        EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
        //EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    }
    else { //period will go to state V0
        EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
        //EPWM_setActionQualifierAction(PhaseA_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

        EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
        //EPWM_setActionQualifierAction(PhaseB_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

        EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
        //EPWM_setActionQualifierAction(PhaseC_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

    }
}


//
// End of File
//
