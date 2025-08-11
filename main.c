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
#include "eqep.h"
#include "gpio.h"
#include "inc/hw_memmap.h"
#include "sdfm.h"
#include "stdio.h"
#include "math.h"

#include "CAN_setup.h"




///// moving average 

#define MA_WINDOW 8

float freq_history[MA_WINDOW] = {0};
float boost_history[MA_WINDOW] = {0};

uint8_t ma_index = 0;
float freq_sum = 0;
float boost_sum = 0;



/* ---------- tunables ---------- */
#define KP_SLIP            0.15f      /* proportional  (Hz / RPM)      */
#define KI_SLIP             3.0f      /* integral      (Hz / RPM·s)    */
#define CTRL_TS             0.001f    /* speed-loop period [s] (1 kHz) */

#define SLIP_RATIO_MAX      0.05f     /* ≤5 % of synchronous freq      */
#define SLIP_START_HZ_MIN   1.0f      /* floor for stall pull-away     */

/* ---------- tiny PI helper ---------- */
typedef struct {
    float Kp, Ki, Ts;
    float integ;                      /* ∑ error·Ts */
} PI_t;

static PI_t pi_slip = { KP_SLIP, KI_SLIP, CTRL_TS, 0.0f };

static inline float pi_update(PI_t *c,
                              float err,
                              float out_min,
                              float out_max)
{
    float p = c->Kp * err;

    /* integrate with anti-wind-up */
    c->integ += c->Ki * err * c->Ts;
    if (c->integ > out_max) c->integ = out_max;
    if (c->integ < out_min) c->integ = out_min;

    float u = p + c->integ;

    /* final clamp */
    if (u > out_max) u = out_max;
    if (u < out_min) u = out_min;
    return u;
}


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
volatile float encoder_counter = 0;
volatile bool direction = 0;
volatile float rpm = 0;
volatile float f_elec = 0;
volatile float targetRPM = 0;
volatile float rpmError = 0;
volatile float slipFactor = 0;

volatile float A_High_Temp = 0;
volatile float A_Low_Temp = 0;
volatile float B_High_Temp = 0;
volatile float B_Low_Temp = 0;
volatile float C_High_Temp = 0;
volatile float C_Low_Temp = 0;

volatile bool shutdown_requested = 1;

volatile uint8_t HALL = 0;

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
volatile float PWM_Period = 4000.0f; //20000 = 5kHz?
#define PWM_ClockDivider 1 // 128 * 8
#define frequency_rated 2000.0f // Max Frequency
//#define deadband 4

volatile uint16_t deadband = 500;


#define EPWM_CLOCK 100000000 // System Clock divide by 2: 200MHz / 2 = 100MHz
#define pole_pairs  6 // 12 pole motor ( 6 poles per phase/pair)


__interrupt void INT_PhaseA_ISR(void);
__interrupt void INT_rpmInterrupt_ISR(void);
__interrupt void INT_esp32pwm_ISR(void);
void changeEPWMcompare();
float measureThermistors(uint16_t adcVal);
void spaceVectorModulation();


//temp 

volatile uint32_t raw1, raw2, raw3;
volatile float voltage;
volatile float current;

#define SDM_FULL_SCALE_V 0.05f
#define SDM_SHIFT_BITS   10
#define SDM_MAX_VALUE    8388608.0f  // 2^23 (because you right-shifted by 10)

volatile int brightness = 20000;

volatile uint8_t prev_hall_state = 0xFF;
volatile uint16_t hall_transition_count = 0;
uint8_t cycle_count = 0;
static uint8_t last_hall = 0xFF;










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

        
        // measure thermistors
        //
        // A_High = A2
        // A_Low = B2
        // B_High = C2
        // B_Low = A3
        // C_High = B3
        // C_Low = C3
        //
        // A = myADC0
        // B = myADCB
        // C = myADCC
        //


        adcResult = ADC_readResult(myADC0_RESULT_BASE, myADC0_A_HIGH);
        A_High_Temp = measureThermistors(adcResult);



        adcResult = ADC_readResult(myADC0_RESULT_BASE, myADC0_A_LOW);

        A_Low_Temp = measureThermistors(adcResult);



        adcResult = ADC_readResult(myADCC_RESULT_BASE, myADCC_B_Low);
        B_High_Temp = measureThermistors(adcResult);

        //send frequency
        //CAN_sendMessage(CANB_BASE, TX_MSG_OBJ_ID5,4,pot1.words);
        //poll for reception of remote frame
        //while(((HWREGH(CANB_BASE + CAN_O_ES) & CAN_ES_TXOK)) !=  CAN_ES_TXOK){}

        // raw = SDFM_getFilterData(Current_BASE, SDFM_FILTER_3);

        // raw >>= 10;
        // voltage = (float)raw / SDM_MAX_VALUE * SDM_FULL_SCALE_V;
        // current = voltage / 0.0005f;

        // raw1 = SDFM_getFilterData(Current_BASE, SDFM_FILTER_3);

        // raw1 >>= 10;
        // voltage = (float)raw1 / 1024.0f;

        //raw2 = SDFM_getFilterData(Voltage_BASE, SDFM_FILTER_2);
        //raw3 = SDFM_getFilterData(Voltage_BASE, SDFM_FILTER_3);
        //DEVICE_DELAY_US(50000);  // Delay 500ms

        //send

        cycle_count++;

        // 48 pole motor ebike
        if (cycle_count >= 60)  // 10 × 100ms = 1 second
        {
            float rpm = (hall_transition_count * 60.0f) / 144.0f;
            uint16_t rpm_u16 = (uint16_t)rpm;

            uint8_t rpm_bytes[2];
            rpm_bytes[0] = (rpm_u16 >> 8) & 0xFF;
            rpm_bytes[1] = rpm_u16 & 0xFF;

            CAN_sendMessage(CANB_BASE, 12, 2, rpm_bytes);
            while (HWREGH(CANB_BASE + CAN_O_TXRQ_X) & (1U << (12 - 1))) {}


            hall_transition_count = 0;
            cycle_count = 0;
        }


 
        uint8_t throttle_int = (uint8_t)pot1.f;  
        uint8_t temp = (uint8_t)A_Low_Temp;
        uint8_t hall_states = HALL;
        uint8_t angle_bytes[2];
        uint8_t duty_cycle_pot = (uint8_t)(pot2.f * 100);
        uint16_t angle_u16 = (uint16_t)(angle * (180.0 / M_PI));
        angle_bytes[0] = (uint8_t)(angle_u16 >> 8);    // MSB
        angle_bytes[1] = (uint8_t)(angle_u16 & 0xFF);  // LSB

        CAN_sendMessage(CANB_BASE, 13, 1, &hall_states);
        while (HWREGH(CANB_BASE + CAN_O_TXRQ_X) & (1U << (13 - 1))) {}

        CAN_sendMessage(CANB_BASE, 11, 1, &throttle_int);
        while (HWREGH(CANB_BASE + CAN_O_TXRQ_X) & (1U << (11 - 1))) {}

        CAN_sendMessage(CANB_BASE, 14, 1, &temp);
        while (HWREGH(CANB_BASE + CAN_O_TXRQ_X) & (1U << (14 - 1))) {}

        CAN_sendMessage(CANB_BASE, 15, 2, angle_bytes);
        while (HWREGH(CANB_BASE + CAN_O_TXRQ_X) & (1U << (15 - 1))) {} // 0x300

        CAN_sendMessage(CANB_BASE, 16, 1, &duty_cycle_pot);
        while (HWREGH(CANB_BASE + CAN_O_TXRQ_X) & (1U << (16 - 1))) {} // 0x300




        //CAN_sendMessage(CANB_BASE, 11, 1, &throttle_int); // ID 0x115


        
        //poll for reception of remote frame
        //while(((HWREGH(CANB_BASE + CAN_O_ES) & CAN_ES_TXOK)) !=  CAN_ES_TXOK){}
        //adjustable delays

        EPWM_setCounterCompareValue(Blue_Led_BASE, EPWM_COUNTER_COMPARE_A, brightness);	
 
        DEVICE_DELAY_US(100000);  
        
    }
}


float measureThermistors(uint16_t adcVal){
    float v_ref = 3.3f;
    float r_fixed = 10000.0f;
    float adc_max = 4095.0f;

    float v_adc = ((float)adcVal / adc_max) * v_ref;
    float r_ntc = ((v_ref - v_adc) * r_fixed) / v_adc;  // Pull-down config

    float R0 = 10000.0f;         // NTC nominal resistance at 25°C
    float B = 4450.0f;           // Beta constant
    float T0 = 298.15f;          // 25°C in Kelvin

    float inv_T = (1.0f / T0) + (1.0f / B) * logf(r_ntc / R0);
    float temp_K = 1.0f / inv_T;
    return temp_K - 273.15f;     // °C
}

// globals (one definition)
volatile float pwm_freq_hz = 0.0f;
volatile float pwm_duty    = 0.0f;

__interrupt void INT_esp32pwm_ISR(void)
{
    EALLOW;
    uint32_t t1 = ECAP_getEventTimeStamp(esp32pwm_BASE, ECAP_EVENT_1); // rising
    uint32_t t2 = ECAP_getEventTimeStamp(esp32pwm_BASE, ECAP_EVENT_2); // falling
    uint32_t t3 = ECAP_getEventTimeStamp(esp32pwm_BASE, ECAP_EVENT_3); // rising
    uint32_t t4 = ECAP_getEventTimeStamp(esp32pwm_BASE, ECAP_EVENT_4); // falling (not used)

   

    uint32_t t_high = t2 - t1;   // HIGH time
    uint32_t t_low  = t3 - t2;   // LOW time  
    uint32_t t_per  = t3 - t1;   // PERIOD    


    pwm_duty = (float)t_high / (float)t_per;



    ECAP_clearInterrupt(esp32pwm_BASE, ECAP_ISR_SOURCE_CAPTURE_EVENT_4 | ECAP_ISR_SOURCE_COUNTER_OVERFLOW); // 1) source flags

    ECAP_clearGlobalInterrupt(esp32pwm_BASE);
    ECAP_reArm(esp32pwm_BASE);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP4);
    EDIS;
}



__interrupt void INT_rpmInterrupt_ISR(void){ // every 38.4ms

    EALLOW;
    EPWM_clearEventTriggerInterruptFlag(rpmInterrupt_BASE);
    
    // Acknowledge the interrupt in PIE (this is crucial for clearing the interrupt in PIE)
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);

    direction = EQEP_getDirection(RPM_BASE);
    encoder_counter = EQEP_getPosition(RPM_BASE); // 0 - 2000. 1000 is the reset value
    if (encoder_counter == 1000){ // not rotating
        rpm = 0;
    }
    else {
        if (direction == 1) { //forward     
            rpm = -1 * (((encoder_counter - 1000) / 256.0f) * 60 / ((60000.0f * 2* 128.0f) / EPWM_CLOCK));
        }
        else {
            rpm = (((encoder_counter - 1000) / 256.0f) * 60 / ((60000.0f * 64.0f) / EPWM_CLOCK));
        }

        EQEP_setPosition(RPM_BASE, 1000); // reset back to 1000
    }

 
    EDIS;
    

}

// Slowly ease current frequency toward target
float easing = 0.011f;  // Smaller = slower; try 0.005f or 0.001f for very smooth ramp
float easing2 = 0.011f;


__interrupt void INT_PhaseA_ISR(void){

    //EPWM_setTimeBaseCounter(PhaseC_BASE, 5);	
    //EPWM_setTimeBaseCounter(PhaseB_BASE, 5);	
    //EPWM_forceSyncPulse(PhaseA_BASE);



    EALLOW;
    EPWM_clearEventTriggerInterruptFlag(PhaseA_BASE);
    // Acknowledge the interrupt in PIE (this is crucial for clearing the interrupt in PIE)
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);

    shutdown_requested = GPIO_readPin(Brake);

    //HALL = GPIO_readPin(HALL_A) << 0 | GPIO_readPin(HALL_B) << 1  | GPIO_readPin(HALL_C) << 2;
    uint8_t current_hall = GPIO_readPin(HALL_A) << 0 |
                       GPIO_readPin(HALL_B) << 1 |
                       GPIO_readPin(HALL_C) << 2;

    HALL = current_hall; // Keep this if you still use HALL elsewhere

    if (current_hall != last_hall) {
        hall_transition_count++;
        last_hall = current_hall;
    }


    if (shutdown_requested) {
        // Smoothly ramp boostV to 0
        if (boostV > 0.01f) {
            boostV -= 0.01f; // decrease per interrupt cycle (adjust as needed)
            spaceVectorModulation();
        } else {
            boostV = 0.0f;

           
            A_CMPA_UP = EPWM_AQ_OUTPUT_LOW;
            A_CMPB_UP = EPWM_AQ_OUTPUT_LOW;
            B_CMPA_UP = EPWM_AQ_OUTPUT_LOW;
            B_CMPB_UP = EPWM_AQ_OUTPUT_LOW;
            C_CMPA_UP = EPWM_AQ_OUTPUT_LOW;
            C_CMPB_UP = EPWM_AQ_OUTPUT_LOW;
            

            
            V_ref = 0.0f;
            changeEPWMcompare();
        }
    }
    else{
        
        if (1) {                /* ───────── MANUAL POT MODE ───────── */
                // Read ADC and compute raw values
                // adcResult = ADC_readResult(myADC0_RESULT_BASE, myADC0_SOC0);
                // pot1.f = adcResult * frequency_rated / 4095.0f;

                adcResult = ADC_readResult(myADC0_RESULT_BASE, myADC0_SOC0);
                float target_freq = adcResult * frequency_rated / 4095.0f;

                
                pot1.f += (target_freq - pot1.f) * easing;

                adcResult = ADC_readResult(myADC0_RESULT_BASE, myADC0_SOC1);
                float target_v = adcResult * 0.9f / 4095.0f;

                
                pot2.f += (target_v - pot2.f) * easing2;

                // Moving average update for frequency
                freq_sum -= freq_history[ma_index];
                freq_history[ma_index] = pot1.f;
                freq_sum += pot1.f;

                // Moving average update for boost
                boost_sum -= boost_history[ma_index];
                boost_history[ma_index] = pot2.f;
                boost_sum += pot2.f;

                // Store smoothed values
                frequency = freq_sum / MA_WINDOW;
                boostV = boost_sum / MA_WINDOW;

                // Advance index
                ma_index = (ma_index + 1) % MA_WINDOW;

            }
            else {                   /* ───────── AUTO: PI-SLIP MODE ───── */

                /* 1) throttle pot → target RPM */
                adcResult  = ADC_readResult(myADC0_RESULT_BASE, myADC0_SOC0);
                targetRPM  = (adcResult * 1000.0f) / 4095.0f;      /* 0-1000 rpm */
                if (targetRPM < 10.0f) targetRPM = 0.0f;           /* dead-band  */

                /* 2) mech → electrical freq (present) */
                float f_mech = rpm * pole_pairs / 60.0f;           /* Hz rotor   */

                /* 3) dynamic slip ceiling (≤5 % of sync freq) */
                float slip_hz_max = SLIP_RATIO_MAX * f_mech;       /* 5 % rule   */
                if (slip_hz_max < SLIP_START_HZ_MIN)
                    slip_hz_max = SLIP_START_HZ_MIN;               /* low-speed pull */

                /* 4) PI → slip command (Hz, ≥0) */
                float speedErr = targetRPM - rpm;                  /* RPM error  */
                float slip_hz  = pi_update(&pi_slip,
                                        speedErr,
                                        0.0f,                  /* no regen   */
                                        slip_hz_max);

                /* 5) final synchronous frequency */
                frequency = f_mech + slip_hz;                      /* Hz stator  */
                if (frequency < 0.5f) {                            /* near stall */
                    frequency     = 1.0f;
                    pi_slip.integ = 0.0f;                          /* clear I    */
                }

                /* 6) simple V/f feed-forward for flux */
                boostV = fminf(frequency / frequency_rated, 1.0f); /* 0-1 duty   */
            }
            
        spaceVectorModulation();
    }

    



    
  
    
    changeEPWMcompare(); //Update EPWM compare values
    

    EDIS;
}

void spaceVectorModulation(){
      
    EPWM_setRisingEdgeDelayCount(PhaseA_BASE, deadband);
    EPWM_setRisingEdgeDelayCount(PhaseB_BASE, deadband);	
    EPWM_setRisingEdgeDelayCount(PhaseC_BASE, deadband);

    EPWM_setFallingEdgeDelayCount(PhaseA_BASE, deadband);	
    EPWM_setFallingEdgeDelayCount(PhaseB_BASE, deadband);	
    EPWM_setFallingEdgeDelayCount(PhaseC_BASE, deadband);	
    
    //EPWM_setGlobalLoadOneShotLatch(PhaseA_BASE);
    


    


    //printf("%d\r\n", adcResult);


    //deadband = roundf(pot2.f);


    //deadband = roundf(pot2.f); //amplutide set by pot2


    SVPWM_UP = !SVPWM_UP; // switch between V0 V1 V2 -> V7 V2 V1 pattern
    
    //GPIO_togglePin(myGPIO0); // verify interrupt is going through


    //deadband = pot2;
    //boostV = 0.2f;
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
