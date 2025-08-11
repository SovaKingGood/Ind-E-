
//
// Included Files
//

//#include <stdlib.h>  // For rand() function
//#include <time.h>    // For seeding the random number generator

//
// Defines
//
#define TXCOUNT  100000
#define MSG_DATA_LENGTH1 1
#define MSG_DATA_LENGTH2 2
#define MSG_DATA_LENGTH4 4
#define TX_MSG_OBJ_ID1 1
#define TX_MSG_OBJ_ID2 2
#define TX_MSG_OBJ_ID3 3
#define TX_MSG_OBJ_ID4 4
#define TX_MSG_OBJ_ID5 5 //freq
#define TX_MSG_OBJ_ID6 6 // amp
#define TX_MSG_OBJ_ID7 7 // test
#define BUTTON_GPIO 105  // GPIO 105 as button input

//============CAN bus ID priority tags=================//
#define SPEED_ID 0x101
#define ACC_ID 0x102
#define MTR_CTL_TEMP_1 0x103
//
// space for motor controller temps up to #6
//
#define MTR_TEMP 0x109
#define PHA_I 0x110
#define PHB_I 0x111
#define PHC_I 0x112
#define THRTL_POS 0x113
#define MTR_TRQ 0x114
#define MTR_RPM 0x115
#define DRV_MODE 0x116 //PARK / NEUTRAL / REVERSE
#define frequency_can 0x117
#define amplitude_can 0x118
#define test 0x69

#define BRAKE_STATUS 0x117 //ON or OFF
#define WINDSHIELD_WIPER_STATUS 0x118 //!!!!!!!!!!!noooooooo wayyyyyyy
//====================================================//

//
// Globals
//
uint16_t button_last_state = 1;
uint16_t button_current_state  = 1;
volatile unsigned long i;
volatile uint32_t txMsgCount = 0;
uint16_t txMsgData[2];

uint16_t speed[1];          // Random speed (0-200 km/h)
uint16_t acc[1];             // Random acceleration (0-9 m/s²)
uint16_t mtr_ctl_temp[1];  // Motor controller temp (20-120°C)
uint16_t mtr_temp[1];      // Motor temp (20-120°C)
uint16_t pha_i[1];          // Phase A current (0-100 A)
uint16_t phb_i[1];          // Phase B current (0-100 A)
uint16_t phc_i[1];          // Phase C current (0-100 A)
uint16_t thrtl_pos[1];      // Throttle position (0-100%)
uint16_t mtr_trq[1];        // Motor torque (0-500 Nm)
uint16_t mtr_rpm[1];      // Motor RPM (0-10,000 RPM)
uint16_t drv_mode[1];         // 0 = Park, 1 = Neutral, 2 = Reverse
uint16_t brake_status[1];     // 0 = OFF, 1 = ON
uint16_t wiper_status[1];     // 0 = OFF, 1 = ON

//int txMsgData = 0; //temporary for test




void CAN_send_setup(void)
{
    GPIO_setPinConfig(DEVICE_GPIO_CFG_CANRXA);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_CANTXA);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_CANRXB);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_CANTXB);

    //Board_init();
    //
    // Initialize the CAN controllers
    //
    CAN_initModule(CANA_BASE);
    CAN_initModule(CANB_BASE);

    //
    // Set up the CAN bus bit rate to 500kHz for each module
    // Refer to the Driver Library User Guide for information on how to set
    // tighter timing control. Additionally, consult the device data sheet
    // for more information about the CAN module clocking.
    //

    CAN_setBitRate(CANA_BASE, DEVICE_SYSCLK_FREQ, 250000, 16);
    CAN_setBitRate(CANB_BASE, DEVICE_SYSCLK_FREQ, 250000, 16);

    //
    // Initialize the transmit message object used for sending CAN messages.
    // Message Object Parameters:
    //      CAN Module: B
    //      Message Object ID Number: 1
    //      Message Identifier: 0x01
    //      Message Frame: Standard
    //      Message Type: Transmit
    //      Message ID Mask: 0x0
    //      Message Object Flags: None
    //      Message Data Length: 4 Bytes
    //

    CAN_setupMessageObject(CANB_BASE, TX_MSG_OBJ_ID1, SPEED_ID,
                           CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0,
                           CAN_MSG_OBJ_NO_FLAGS, MSG_DATA_LENGTH1); //set up CANB to respond to remote frames

    CAN_setupMessageObject(CANB_BASE, TX_MSG_OBJ_ID2, THRTL_POS,
                           CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0,
                           CAN_MSG_OBJ_NO_FLAGS, MSG_DATA_LENGTH2); //set up CANB to respond to remote frames

    CAN_setupMessageObject(CANB_BASE, TX_MSG_OBJ_ID3, MTR_RPM,
                           CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0,
                           CAN_MSG_OBJ_NO_FLAGS, MSG_DATA_LENGTH2); //set up CANB to respond to remote frames

    CAN_setupMessageObject(CANB_BASE, TX_MSG_OBJ_ID4, DRV_MODE,
                           CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0,
                           CAN_MSG_OBJ_NO_FLAGS, MSG_DATA_LENGTH2); //set up CANB to respond to remote frames

    CAN_setupMessageObject(CANB_BASE, TX_MSG_OBJ_ID5, frequency_can,
                           CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0,
                           CAN_MSG_OBJ_NO_FLAGS, MSG_DATA_LENGTH4); //set up CANB to respond to remote frames

    CAN_setupMessageObject(CANB_BASE, TX_MSG_OBJ_ID6, amplitude_can,
                           CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0,
                           CAN_MSG_OBJ_NO_FLAGS, MSG_DATA_LENGTH4); //set up CANB to respond to remote frames

    CAN_setupMessageObject(CANB_BASE, TX_MSG_OBJ_ID7, test,
                           CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0,
                           CAN_MSG_OBJ_NO_FLAGS, MSG_DATA_LENGTH1); //set up CANB to respond to remote frames

    CAN_setupMessageObject(CANB_BASE, 10, 0x123,
                        CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0,
                        CAN_MSG_OBJ_NO_FLAGS, 3); //set up CANB to respond to remote frames

    CAN_setupMessageObject(CANB_BASE, 11, 0x113,
                        CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0,
                        CAN_MSG_OBJ_NO_FLAGS, 1); // THROTTLE
                                    
    CAN_setupMessageObject(CANB_BASE, 12, 0x115,
                        CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0,
                        CAN_MSG_OBJ_NO_FLAGS, 2); // RPM

    CAN_setupMessageObject(CANB_BASE, 13, 0x200,
                        CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0,
                        CAN_MSG_OBJ_NO_FLAGS, 1); // HALL


    CAN_setupMessageObject(CANB_BASE, 14, 0x103,
                        CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0,
                        CAN_MSG_OBJ_NO_FLAGS, 1); // Temp

    CAN_setupMessageObject(CANB_BASE, 15, 0x300,
                        CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0,
                        CAN_MSG_OBJ_NO_FLAGS, 2); // Angle
                                    

    CAN_setupMessageObject(CANB_BASE, 16, 0x116,
                        CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0,
                        CAN_MSG_OBJ_NO_FLAGS, 1); // Duty

                        
    CAN_setupMessageObject(CANB_BASE, 17, 0x117893900,
                        CAN_MSG_FRAME_EXT, CAN_MSG_OBJ_TYPE_TX, 0,
                        CAN_MSG_OBJ_NO_FLAGS, 8); // DEMO SHITS       
                                    
    //
    //
    // Initialize the transmit message object data buffer to be sent
    //
    //txMsgData[0] = 0x00;
    //txMsgData[1] = 0x00;
    //txMsgData[2] = 0x45;
    //txMsgData[3] = 0x67;
    //txMsgData[4] = 0x89;
    //txMsgData[5] = 0xAB;
    //txMsgData[6] = 0xCD;
    //txMsgData[7] = 0xEF;
    //txMsgData = 0;
	
    //
    // Start CAN module A operations
    //
    CAN_startModule(CANA_BASE);
    CAN_startModule(CANB_BASE);

}



