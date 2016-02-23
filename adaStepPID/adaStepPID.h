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

    #define DEBUG_PRINT(toPrint)   Serial.print(tPrint);
    #define DEBUG_PRINTNL(toPrint) Serial.println(toPrint);
#else // The macros do nothing.
    #define DEBUG_STATE_PRINT(state, target, sens)
    #define DEBUG_PRINT(toPrint)
    #define DEBUG_PRINTNL(toPrint)
#endif

#ifdef PID_DEBUG
    #define DEBUG_PID_PRINT(input, output, setPoint){                          \
        Serial.print("PID VALUES, input: ");                                   \
        Serial.print(input);                                                   \
        Serial.print(" output: ");                                             \
        Serial.print(output);                                                  \
        Serial.print(" setPoint: ");                                           \
        Serial.println(setPoint);                                              \
    } //end debug_pid_print
#else
    #define DEBUG_PID_PRINT(input, output, setPoint)
#endif


// Arduino values
#define SERIAL_V         9600  // analog voltage on display
#define CLOSED           0     // valve is closed
#define OPEN             1     // valve is open

// Clog values
#define CLOG_OFFSET      36.0  // offset from a target value showing a clog
#define CLOG_VAL(target) ((target)+CLOG_OFFSET) //value when clogged
#define DIST_TGT(target,sensor) abs((target)-(sensor)) // distance from target

// Motor values
#define STEP_REV    200  // steps per revolution on the motor
#define STEP_CH     2    // associated stepper channel for the motor
#define XXRPM       60   // setup xx rpm of motor.

// Step sizes
#define INIT_STEP   1   // initial steps for coils 
#define CLOSE_STD   1   // close valve
#define OPEN_STD    1   // open valve
#define CLOSE_AGG   10  // aggressive close
#define OPEN_AGG    10  // aggressive open

// PID tunning values
#define KP          0
#define KI          0
#define KD          0
#define AGG_KP
#define AGG_KI
#define AGG_KD
#define PID_LOW     0     // lower output limit of PID 
#define PID_HIGH    1023  // high output limit of PID 

// ISR targetPressure granularity
#define INC_TARGET  1   // increase ammount
#define DEC_TARGET  1   // decrease ammount

// Physical pins:
#define PRESSURE    A0  //ANALOG INPUT FOR PRESSURE SENSOR
#define BUTTON_DN   2   //DECREASE PRESSURE
#define BUTTON_UP   3   //INCREASE PRESSURE
#define BUTTON_OFF  4   //DISABLE/ENABLE MOTOR
#define MOTOR_LED   5   //MOTOR POWER INDICATOaR LED
/******************************************************************************
*                               EOF
******************************************************************************/
