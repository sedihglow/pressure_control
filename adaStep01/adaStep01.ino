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
#include "adaStep01.h"

/* TODO: Once all pressure values are honed and tested: If targetPressure or
         sensorValue never utilizes float, make integers. */
volatile float targetPressure = 682.0;  // Arbitrarily set to about 3000psi.
volatile float sensorValue = 0.0;       // Return value from pressure read.
unsigned int openStepCount = 0;         // number of OPEN_STEP's done pre close

// shield object
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// stepper object w/ number of motors. counts, port#2 (m3 m4)
Adafruit_StepperMotor *myMotor = AFMS.getStepper(STEP_REV, STEP_CH);

// function prototypes
void init_valve();  // initialize the LED, coils and TODO: default position.
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


    pinMode(PRESSURE, INPUT);   // pressure reading from sensor
    pinMode(BUTTON_DN, INPUT);  // pressure increase button
    pinMode(BUTTON_UP, INPUT);  // pressure decrease button
    pinMode(BUTTON_OFF, INPUT); // sepper motor off button
    
    pinMode(MOTOR_LED, OUTPUT); // motor power green LED
    
    // Interupts only works on pins 2,3 for ATMEGA328
    attachInterrupt(0, decrease_pressure_ISR, CHANGE);
    attachInterrupt(1, increase_pressure_ISR, CHANGE);

    // TODO: think about moving away from prepressurizing the system.
    sensorValue = analogRead(PRESSURE); // the initial pressure.
    targetPressure = sensorValue;       // the initial target pressure.

    init_valve(); // initialize the LED, coils and TODO: default position
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
    else{// motor is ON
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

        if(sensorValue < targetPressure){
            close_valve();
            DEBUG_STATE_PRINT("LOW", targetPressure, sensorValue);
        } //end if

        #ifdef DEBUG
        if(sensorValue == targetPressure){ 
            DEBUG_STATE_PRINT("STEADY", targetPressure, sensorValue);}
        #endif
    }//end else
} // end loop()

/******************************************************************************
*                           VOID INIT_VALVE:
******************************************************************************/
void init_valve(){
    digitalWrite(MOTOR_LED, HIGH); // motor power light ON
    // Enable to coils. 
    myMotor->step(INIT_STEP, BACKWARD, INTERLEAVE);
    delay(10); // TODO: check if delay is required
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
        myMotor->step(CLOSE_STEP(openStepCount), FORWARD, INTERLEAVE);
        openStepCount = 0;
        DEBUG_PRINTNL("CLOSING Valve, Increasing Pressure");
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
