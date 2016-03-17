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
#include "multiStepV2.h"

int16_t targetPressure = 0;   // Arbitrarily set to about 3000psi.
int16_t sensorValue    = 0;   // Return value from pressure read.
int16_t openStepCount  = 0;   // count from starting position opened.
int16_t manualOpenStep = 0;   // The steps manually opening valve occured.
int16_t stepLvl        = 0;   // the corresponding step level for pressure
bool    manualInit     = LOW; // manual initialization was used
bool    switchDn       = LOW; // active low switch flag, toggle down
bool    switchUp       = LOW; // active low switch flag, toggle up
bool    motorPower     = LOW; // keep motor off/on

// shield object
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// stepper object w/ number of motors. counts, port#2 (m3 m4)
Adafruit_StepperMotor *myMotor = AFMS.getStepper(STEP_REV, STEP_CH);

// function prototypes
void init_valve();                  // initialize the LED, coils 
int16_t step_level();               // check what step size is required
void open_valve(int16_t stepSize);  // opens valve, decreases pressure.
void close_valve(int16_t stepSize); // close valve, increase pressure.

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
void setup(){ // Setup called once in every program to initialize everything.
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
} // end setup

/******************************************************************************
*                           VOID LOOP:
******************************************************************************/
void loop(){
//******************* Check activation button *********************************
    while(LOW == digitalRead(BUTTON_OFF)){ // button is off, active LOW
        myMotor->release(); 
        digitalWrite(MOTOR_LED, LOW);    // motor power light OFF
        motorPower     = LOW;
        openStepCount  = 0;
        manualOpenStep = 0;
        DEBUG_PRINTNL("Motor OFF");
    }//end while

    init_valve(); // initialize the valve and set initial targetPressure

//*************** Check toggle switch for pressure adjustment *****************
    do{
        if(LOW == (switchDn = digitalRead(BUTTON_DN))){
            if(manualOpenStep < MAN_MAX){
                ++manualOpenStep;
                open_valve(SINGLE_STEP);
                //targetPressure = analogRead(PRESSURE); // set new manual target value
                manualInit = true;
            }//end if 
        }//end if
        else if(LOW == (switchUp = digitalRead(BUTTON_UP))){
            close_valve(QCK_STEP);
            //targetPressure = analogRead(PRESSURE); // set new manual target value
            manualInit = true;
        }//end else
    }while(switchDn == LOW || switchUp == LOW);
    
    if(true == manualInit){ // if manual open was engaged
        targetPressure = analogRead(PRESSURE);
        close_valve(manualOpenStep);
        manualOpenStep = 0;
        manualInit = false;
    }// end if 

//**************** Acheive target pressure ************************************
    sensorValue = analogRead(PRESSURE);
    stepLvl = step_level(); // get appropriate step size
    DEBUG_PRINT("STEP_LEVEL = ");
    DEBUG_PRINTNL(stepLvl);
    if(sensorValue > targetPressure){
        open_valve(stepLvl);
        DEBUG_STATE_PRINT("HIGH", targetPressure, sensorValue);
    }//end if 

    if(sensorValue < targetPressure){
        close_valve(stepLvl+QCK_STEP+QCK_STEP);
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
void init_valve(){
    digitalWrite(MOTOR_LED, HIGH); // motor power light ON
    // Enable to coils.
    if(motorPower == false){ 
        targetPressure = analogRead(PRESSURE);
        myMotor->step(INIT_STEP, BACKWARD, INTERLEAVE);
        delay(100);
        myMotor->step(INIT_STEP, FORWARD, INTERLEAVE);
        delay(100);
        motorPower = true;
    }//end if 
}// end init_valve

/******************************************************************************
*                       int16_t step_level:
******************************************************************************/
int16_t step_level(){
//******************* Check for a pressure build ******************************
    int16_t dist = DIST_TGT(targetPressure, sensorValue);
    DEBUG_STATE_PRINT("STEP_CHECK", targetPressure, sensorValue);
    DEBUG_PRINT("DIST = ");
    DEBUG_PRINTNL(dist);
    if     (dist < OFFSET_ONE  ) {  return LVL_ZERO;  }
    else if(dist > OFFSET_SIX  ) {  return LVL_SIX;   }
    else if(dist > OFFSET_FIVE ) {  return LVL_FIVE;  }
    else if(dist > OFFSET_FOUR ) {  return LVL_FOUR;  }
    else if(dist > OFFSET_THREE) {  return LVL_THREE; }
    else if(dist > OFFSET_TWO  ) {  return LVL_TWO;   }
    else if(dist > OFFSET_ONE  ) {  return LVL_ONE;   }
}// end step_level

/******************************************************************************
*                       VOID OPEN_VALVE:
******************************************************************************/
void open_valve(int16_t stepSize){
    myMotor->step(stepSize, BACKWARD, INTERLEAVE);
    delay(100);
    openStepCount += stepSize;
    DEBUG_PRINTNL("OPENING Valve, Decreasing Pressure");
}// end open_valve

/******************************************************************************
*                       VOID CLOSE_VALVE:
******************************************************************************/
void close_valve(int16_t stepSize){
    if(openStepCount > 0){
        if((openStepCount - stepSize) >= 0){ // check for over-closing
            myMotor->step(stepSize, FORWARD, INTERLEAVE);
            delay(100);  
            openStepCount -= stepSize;
        }//end if
        else{ // fully close the valve
            while(openStepCount > QCK_STEP){ // quick close
                myMotor->step(QCK_STEP, FORWARD, INTERLEAVE);
                delay(100);
                openStepCount -= QCK_STEP;
            }//end while
            while(openStepCount > 0){ // single step to fully closed
                myMotor->step(SINGLE_STEP, FORWARD, INTERLEAVE);
                delay(100);
                --openStepCount;
            }//end while
        }//end else
    }//end if
    DEBUG_PRINTNL("CLOSING Valve, Increasing Pressure");
}// end close_valve
/******************************************************************************
*                               EOF
******************************************************************************/
