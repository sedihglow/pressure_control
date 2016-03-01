/***********************************************************
*JAMES ROSS
*6 FEBRUARY 2016
*CLIENT: 
*PROJECT: PSI pressure value w/ stepper motor.
*
*ABSTRACT: Header file defining pin assignments and 
*common values used througout the program.
***********************************************************/
// Debug print macros
#ifdef DEBUG
    #define DEBUG_STATE_PRINT(state, target, sens){                            \
        Serial.print("Pressure " state " , Target: ");                         \
        Serial.print(target);                                                  \
        Serial.print(" Sample: ");                                             \
        Serial.println(sens);                                                  \
    } //end debug_state_print

    #define DEBUG_PRINT(toPrint)   Serial.print(toPrint);
    #define DEBUG_PRINTNL(toPrint) Serial.println(toPrint);
#else // The macros do nothing.
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
#define MAN_MAX      70   // max the valve will manually open with switch

// Step levels
#define LVL_ZERO     5    // standard step size
#define LVL_ONE      10
#define LVL_TWO      20
#define LVL_THREE    40
#define LVL_FOUR     50
#define LVL_FIVE     60

// offset from desired target pressure
#define OFFSET_ONE   10 
#define OFFSET_TWO   20 
#define OFFSET_THREE 36
#define OFFSET_FOUR  50
#define OFFSET_FIVE  60

#define DIST_TGT(target,sensor) abs((target)-(sensor)) // distance from target

// Step sizes
#define INIT_STEP   1     // initial steps for coils 
#define SINGLE_STEP 1     // single step
#define QCK_STEP    10    // quick step above single step

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
