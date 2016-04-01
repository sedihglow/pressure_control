/***********************************************************
*JAMES ROSS
*6 FEBRUARY 2016
*PROJECT: PSI pressure value w/ stepper motor.
*
*ABSTRACT: Header file defining pin assignments and 
*common values used througout the program.
***********************************************************/

//************** Debug print macros ********************************************
#ifdef DEBUG
    #define DEBUG_STATE_PRINT(state, target, sens){                            \
        Serial.print("Pressure " state " , Target: ");                         \
        Serial.print(target);                                                  \
        Serial.print(" Sample: ");                                             \
        Serial.println(sens);                                                  \
    } //end debug_state_print

    #define DEBUG_PRINT(toPrint)   Serial.print(toPrint);
    #define DEBUG_PRINTNL(toPrint) Serial.println(toPrint);
#endif

#ifndef DEBUG // The macros do nothing.
    #define DEBUG_STATE_PRINT(state, target, sens)
    #define DEBUG_PRINT(toPrint)
    #define DEBUG_PRINTNL(toPrint)
#endif

// Arduino values
#define SERIAL_V     115200  // speed of serial connection

// Motor values
#define STEP_REV     200  // steps per revolution on the motor
#define STEP_CH      2    // associated stepper channel for the motor
#define XXRPM        60   // setup xx rpm of motor.
#define MAN_MAX_LRG  250  // max the valve will manually open with switch large
#define MAN_MAX_SML  150  // max the valve will manually open with switch small
#define SET_MAN_PR   465  // value for a point of pressure for manual op.

// Step levels
#define LVL_ZERO     1
#define LVL_ONE      2
#define LVL_TWO      3
#define LVL_THREE    4
#define LVL_FOUR     5
#define LVL_FIVE     7
#define LVL_SIX      10

// offset from desired target pressure reading
#define OFFSET_ONE   2
#define OFFSET_TWO   3
#define OFFSET_THREE 4
#define OFFSET_FOUR  6
#define OFFSET_FIVE  9
#define OFFSET_SIX   12

#define DIST_TGT(target,sensor) abs((target)-(sensor)) // distance from target

// Step sizes
#define INIT_STEP    1    // initial steps for coils 
#define SINGLE_STEP  1    // single step
#define QCK_STEP     10   // quick step above single step
#define DBL_QCK_STEP 20   // double the above step. note: Manually maintained.

// ISR targetPressure granularity // saved for future version
//#define INC_TARGET  1   // increase ammount
//#define DEC_TARGET  1   // decrease ammount

// Physical pins:
#define PRESSURE    A0    // ANALOG INPUT FOR PRESSURE SENSOR
#define BUTTON_DN   2     // DECREASE PRESSURE
#define BUTTON_UP   3     // INCREASE PRESSURE
#define BUTTON_OFF  4     // DISABLE/ENABLE MOTOR
#define MOTOR_LED   5     // MOTOR POWER INDICATOR LED
/******************************************************************************
*                               EOF
******************************************************************************/
