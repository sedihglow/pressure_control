/***********************************************************
*JAMES ROSS adapted from <name>
*6 FEBRUARY 2016
*CLIENT: 
*PROJECT: PSI pressure value w/ stepper motor.
*
*ABSTRACT: Program to monitor pressure and adjust by
*activating a stepper controlled preseure relief valve.
***********************************************************/
 
//#define DEBUG 1    //comment out to suppress debug prints
#include <Wire.h>  
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_MotorShield.h>
#include "adaStep02.h"

volatile float targetPressure = 682.0;  // Arbitrarily set to about 3000psi.
float sensorValue = 0.0;                // Return value from pressure read.
int16_t openStepCount = 0;              // count from starting position opened

// shield object
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// stepper object w/ number of motors. counts, port#2 (m3 m4)
Adafruit_StepperMotor *myMotor = AFMS.getStepper(STEP_REV, STEP_CH);

// function prototypes
void init_valve();  // initialize the LED, coils 
void open_valve();  // opens valve, decreases pressure.
void close_valve(); // close valve, increase pressure.
void clear_clog();  // clog occured, clear the valve.

/******************************************************************************
*                   INTERRUPT SERVICE ROUTINES:
******************************************************************************/

/******************************************************************************
*           ISR TO CLOSE VALVE BASED ON BUTTON_DN PRESS:
******************************************************************************/
void increase_pressure_ISR(){
  targetPressure += INC_TARGET;
}//end increase_pressure_ISR

/******************************************************************************
*           ISR TO OPEN VALVE BASED ON BUTTON_UP PRESS:
******************************************************************************/
void decrease_pressure_ISR(){
  targetPressure -= DEC_TARGET;
}//end decrease_pressure_ISR

/******************************************************************************
*                           VOID SETUP:
******************************************************************************/
void setup()  // Setup called once in every program to initialize everything.
{
    AFMS.begin();          // default frequency 1.6KHz
    Serial.begin(SERIAL_V);// prints out analog voltage on display

    // motor setup routines
    myMotor->setSpeed(XXRPM);
    myMotor->release(); 

    pinMode(PRESSURE,   INPUT); // pressure reading from sensor
    pinMode(BUTTON_DN,  INPUT); // pressure increase button
    pinMode(BUTTON_UP,  INPUT); // pressure decrease button
    pinMode(BUTTON_OFF, INPUT); // sepper motor off button
    
    pinMode(MOTOR_LED, OUTPUT); // motor power green LED
    
    // Interupts only works on pins 2,3 for ATMEGA328
    attachInterrupt(0, decrease_pressure_ISR, CHANGE);
    attachInterrupt(1, increase_pressure_ISR, CHANGE);

    // TODO: think about moving away from prepressurizing the system.
    sensorValue = analogRead(PRESSURE); // the initial pressure.
    targetPressure = sensorValue;       // the initial target pressure.

    init_valve(); // initialize the LED and motor coils
} // end setup

/******************************************************************************
*                           VOID LOOP:
******************************************************************************/
void loop()
{//******************* Motor Power Logic **************************************
    if(0==digitalRead(BUTTON_OFF)){ // check off button.
        myMotor->release(); 
        digitalWrite(MOTOR_LED, LOW); // motor power light OFF
        DEBUG_PRINTNL("Motor OFF");
    }//end if

//**************** Acheive target pressure ************************************
    while((sensorValue = analogRead(PRESSURE)) > targetPressure){
        if(sensorValue > CLOG_VAL(targetPressure)){
            clear_clog();
            DEBUG_STATE_PRINT("CLOG", targetPressure, sensorValue);
        }//end if
        else{
            open_valve();
            DEBUG_STATE_PRINT("HIGH", targetPressure, sensorValue);
        }//end else if
    }// end while 

    while((sensorValue = analogRead(PRESSURE)) < targetPressure){
        close_valve();
        DEBUG_STATE_PRINT("LOW", targetPressure, sensorValue);
    } //end if

    #ifdef DEBUG
    if(sensorValue == targetPressure){ 
        DEBUG_STATE_PRINT("STEADY", targetPressure, sensorValue);}
    #endif
} // end loop()

/******************************************************************************
*                           VOID INIT_VALVE:
******************************************************************************/
void init_valve(){
    digitalWrite(MOTOR_LED, HIGH); // motor power light ON
    // Enable to coils. TODO: Required when default position code is done?
    myMotor->step(INIT_STEP, BACKWARD, INTERLEAVE);
    delay(10); 
    myMotor->step(INIT_STEP, FORWARD, INTERLEAVE);
}// end init_valve

/******************************************************************************
*                           VOID OPEN_VALVE:
******************************************************************************/
void open_valve(){
    myMotor->step(OPEN_STEP, BACKWARD, INTERLEAVE);
    ++openStepCount;
    DEBUG_PRINTNL("OPENING Valve, Decreasing Pressure");
}// end open_valve

/******************************************************************************
*                       VOID CLOSE_VALVE:
******************************************************************************/
void close_valve(){
    if(openStepCount > 0){
        myMotor->step(CLOSE_STEP, FORWARD, INTERLEAVE);  
        DEBUG_PRINTNL("CLOSING Valve, Increasing Pressure");
        --openStepCount;
    }//end if 
}// end close_valve

/******************************************************************************
*                           VOID CLEAR_CLOG:
******************************************************************************/
void clear_clog(){
    myMotor->step(CLOG_STEP, BACKWARD, INTERLEAVE);
    delay(1);
    myMotor->step(CLOG_STEP, FORWARD, INTERLEAVE);
    DEBUG_PRINTNL("ALTERNATING pressure to clear a clog");
}// end clear_clog
/******************************************************************************
*                               EOF
******************************************************************************/
