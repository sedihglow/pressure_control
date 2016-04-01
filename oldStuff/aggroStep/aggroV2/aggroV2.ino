/***********************************************************
*JAMES ROSS 
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
#include "aggroStep.h"

int16_t targetPressure = 0;     // Arbitrarily set to about 3000psi.
int16_t sensorValue    = 0;     // Return value from pressure read.
int16_t openStepCount  = 0;     // count from starting position opened.
int16_t manualOpenStep = 0;     // The steps manually opening valve occured.
bool motorPower        = LOW;   // keep motor off/on
bool aggro             = false; // tuning is aggressive true/false

// shield object
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// stepper object w/ number of motors. counts, port#2 (m3 m4)
Adafruit_StepperMotor *myMotor = AFMS.getStepper(STEP_REV, STEP_CH);

// function prototypes
void init_valve();                  // initialize the LED, coils 
bool aggro_check();                 // check if an aggro step is required
void open_valve(int16_t stepSize);  // opens valve, decreases pressure.
void close_valve(int16_t stepSize); // close valve, increase pressure.
void clear_clog();                  // clog occured, clear the valve.

/******************************************************************************
*                   INTERRUPT SERVICE ROUTINES:
******************************************************************************/
/******************************************************************************
*           ISR TO CLOSE VALVE BASED ON BUTTON_DN PRESS:
******************************************************************************/
/* saved for future updates TODO: Use interupts to test pressure reading vs act.
void increase_pressure_ISR(){
    targetPressure += INC_TARGET;
}//end increase_pressure_ISR
*/
/******************************************************************************
*           ISR TO OPEN VALVE BASED ON BUTTON_UP PRESS:
******************************************************************************/
/* saved for future updates
void decrease_pressure_ISR(){
    targetPressure -= DEC_TARGET;
}//end decrease_pressure_ISR
*/

/******************************************************************************
*                           VOID SETUP:
******************************************************************************/
void setup()  // Setup called once in every program to initialize everything.
{
    AFMS.begin();           // default frequency 1.6KHz
    Serial.begin(SERIAL_V); // prints out analog voltage on display

    // motor setup routines
    myMotor->setSpeed(XXRPM);
    myMotor->release(); 

    pinMode(PRESSURE,   INPUT); // pressure reading from sensor
    pinMode(BUTTON_DN,  INPUT); // pressure increase button
    pinMode(BUTTON_UP,  INPUT); // pressure decrease button
    pinMode(BUTTON_OFF, INPUT); // sepper motor off button
    
    pinMode(MOTOR_LED, OUTPUT); // motor power green LED
    
    // Interupts only works on pins 2,3 for ATMEGA328
    //attachInterrupt(0, decrease_pressure_ISR, LOW);
    //attachInterrupt(1, increase_pressure_ISR, HIGH);

    // TODO: check if this in setup is required at all.
    sensorValue = analogRead(PRESSURE); // the initial pressure.
    targetPressure = sensorValue;       // the initial target pressure.
} // end setup

/******************************************************************************
*                           VOID LOOP:
******************************************************************************/
void loop()
{
//******************* Check activation button *********************************
    while(LOW == digitalRead(BUTTON_OFF)){ // button is off, active LOW
        myMotor->release(); 
        digitalWrite(MOTOR_LED, LOW);    // motor power light OFF
        motorPower = false;
        DEBUG_PRINTNL("Motor OFF");
    }//end while

    init_valve(); // initialize the valve and set initial targetPressure

//*************** Check toggle switch for pressure adjustment *****************
    do{
        if(LOW == (switchDn = digitalRead(BUTTON_DN))){
            if(manualOpenStep < MAN_MAX){
                ++manualOpenStep;
                open_valve(SINGLE_STEP);
            }//end if 
        }//end if
        else if(LOW == (switchUp = digitalRead(BUTTON_UP))){
            close_valve(SINGLE_STEP);
        }//end else
        targetPressure = analogRead(PRESSURE); // set new manual target value
    }while(switchDn == LOW || switchUp == LOW);
    manualOpenStep = 0;

//**************** Acheive target pressure ************************************
    sensorValue = analogRead(PRESSURE);
    aggro = aggro_check();
    if(sensorValue > targetPressure){
        if(false == aggro){ open_valve(OPEN_STD); }
        else              { open_valve(AGG_OPEN); }
        DEBUG_STATE_PRINT("HIGH", targetPressure, sensorValue);
    }//end if 

    if(sensorValue < targetPressure){
        if(false == aggro){ close_valve(CLOSE_STD); }
        else              { close_valve(AGG_CLOSE); }
        DEBUG_STATE_PRINT("LOW", targetPressure, sensorValue);
    }//end if

    #ifdef DEBUG
    if(sensorValue == targetPressure){ 
        DEBUG_STATE_PRINT("STEADY", targetPressure, sensorValue);}
    #endif
} // end loop()

/******************************************************************************
*                       VOID INIT_VALVE:
******************************************************************************/
void init_valve()
{
    digitalWrite(MOTOR_LED, HIGH); // motor power light ON
    // Enable to coils.
    if(motorPower == false){ 
        targetPressure = analogRead(PRESSURE);
        myMotor->step(INIT_STEP, BACKWARD, INTERLEAVE);
        delay(1);
        myMotor->step(INIT_STEP, FORWARD, INTERLEAVE);
        motor = true;
    }//end if 
}// end init_valve

/******************************************************************************
*                       BOOL AGGRO_CHECK:
******************************************************************************/
bool aggro_check(){
//******************* Check for a pressure build *******************************
    if(DIST_TGT(targetPressure, sensorValue) < CLOG_VAL(targetPressure)){ 
        return false;
    }//end if 
    DEBUG_STATE_PRINT("AGGRO", targetPressure, sensorValue);
    return true;
}// end aggro_check

/******************************************************************************
*                       VOID OPEN_VALVE:
******************************************************************************/
void open_valve(int16_t stepSize){
    myMotor->step(stepSize, BACKWARD, INTERLEAVE);
    openStepCount += stepSize;
    DEBUG_PRINTNL("OPENING Valve, Decreasing Pressure");
}// end open_valve

/******************************************************************************
*                       VOID CLOSE_VALVE:
******************************************************************************/
void close_valve(int16_t stepSize){
    if(openStepCount > 0){
        if((openStepCount - stepSize) > 0){ // check for over-closing
            myMotor->step(stepSize, FORWARD, INTERLEAVE);  
            openStepCount -= stepSize;
        }//end if
        else{ // fully close the valve
            while(openStepCount > 0){
                myMotor->step(SINGLE_STEP, FORWARD, INTERLEAVE);
                --openStepCount;
            }//end while
        }//end else
    }//end if
    DEBUG_PRINTNL("CLOSING Valve, Increasing Pressure");
}// end close_valve
/******************************************************************************
*                               EOF
******************************************************************************/
