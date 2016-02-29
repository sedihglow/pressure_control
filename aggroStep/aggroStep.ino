/***********************************************************
*JAMES ROSS adapted from <name>
*6 FEBRUARY 2016
*CLIENT: 
*PROJECT: PSI pressure value w/ stepper motor.
*
*ABSTRACT: Program to monitor pressure and adjust by
*activating a stepper controlled preseure relief valve.
***********************************************************/
 
#define DEBUG 1    //comment out to suppress debug prints
#include <Wire.h>  
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_MotorShield.h>
#include "aggroStep.h"

int16_t targetPressure = 0;     // Arbitrarily set to about 3000psi.
int16_t sensorValue    = 0;     // Return value from pressure read.
int16_t openStepCount  = 0;     // count from starting position opened
bool motorPower        = false; // keep motor off/on
bool aggro             = false; // tuning is aggressive true/false

volatile unsigned long currentTime = 0;
volatile unsigned long other = 0;
volatile unsigned long initTime = 0;


// shield object
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// stepper object w/ number of motors. counts, port#2 (m3 m4)
Adafruit_StepperMotor *myMotor = AFMS.getStepper(STEP_REV, STEP_CH);

// function prototypes
void init_valve();  // initialize the LED, coils 
void open_valve(int16_t stepSize);  // opens valve, decreases pressure.
void close_valve(int16_t stepSize); // close valve, increase pressure.
void clear_clog();  // clog occured, clear the valve.

/******************************************************************************
*                   INTERRUPT SERVICE ROUTINES:
******************************************************************************/

/******************************************************************************
*           ISR TO CLOSE VALVE BASED ON BUTTON_DN PRESS:
******************************************************************************/

void increase_pressure_ISR(){
  
  //targetPressure += INC_TARGET;
  initTime = millis();
  while(0 == digitalRead(BUTTON_UP)){
   // currentTime = abs(initTime - millis());
   // if(currentTime > 1){
    close_valve(10);
    delay(100);
    targetPressure = analogRead(PRESSURE);
  //  targetPressure += INC_TARGET;
 //   initTime = millis();
  }
    //else{
     // initTime = millis();
   // }
 // }
}//end increase_pressure_ISR

/******************************************************************************
*           ISR TO OPEN VALVE BASED ON BUTTON_UP PRESS:
******************************************************************************/
void decrease_pressure_ISR(){
  //targetPressure -= DEC_TARGET;
initTime = millis();
  while(0 == digitalRead(BUTTON_DN)){
   // currentTime = abs(initTime - millis());
   // if(currentTime > 1){
        myMotor->step(1, BACKWARD, INTERLEAVE);
    openStepCount += 1;

    targetPressure = analogRead(PRESSURE);
  //  targetPressure += INC_TARGET;
 //   initTime = millis();
  }
    //else{
     // initTime = millis();
   // }
  //}
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
    //attachInterrupt(0, decrease_pressure_ISR, LOW);
    //attachInterrupt(1, increase_pressure_ISR, HIGH);

    // TODO: think about moving away from prepressurizing the system.
    sensorValue = analogRead(PRESSURE); // the initial pressure.
    targetPressure = sensorValue;       // the initial target pressure.
} // end setup

/******************************************************************************
*                           VOID LOOP:
******************************************************************************/
void loop()
{
//******************* Check activation button **************************************
    if(0==digitalRead(BUTTON_OFF)){ // button is off
        myMotor->release(); 
        digitalWrite(MOTOR_LED, LOW); // motor power light OFF
        motorPower = false;
        DEBUG_PRINTNL("Motor OFF");
    }//end if
    else{
        init_valve();
        sensorValue = analogRead(PRESSURE);
     

        while(0 == digitalRead(BUTTON_DN) || 0 == digitalRead(BUTTON_UP)){
            if(0 == digitalRead(BUTTON_DN))
            {
                open_valve(1);
                targetPressure = analogRead(PRESSURE);
            }//end if
            else if(0 == digitalRead(BUTTON_UP))
            {
                close_valve(1);
                targetPressure = analogRead(PRESSURE);
            }//end else if
        }//end while
//******************* Check for a pressure clog *******************************
    if(DIST_TGT(targetPressure, sensorValue) < CLOG_VAL(targetPressure)){ 
        aggro = false;
    }//end if 
    else{ 
        aggro = true;
        DEBUG_STATE_PRINT("AGGRO", targetPressure, sensorValue);
    } //end else

//**************** Acheive target pressure ************************************
    if(sensorValue > targetPressure){
        if(false == aggro){ open_valve(OPEN_STD); }
        else              { open_valve(AGG_OPEN); }
        DEBUG_STATE_PRINT("HIGH", targetPressure, sensorValue);
    }// end while 

    if(sensorValue < targetPressure){
        if(false == aggro){ close_valve(CLOSE_STD); }
        else              { close_valve(AGG_CLOSE);  }
        DEBUG_STATE_PRINT("LOW", targetPressure, sensorValue);
    } //end while

    #ifdef DEBUG
    if(sensorValue == targetPressure){ 
        DEBUG_STATE_PRINT("STEADY", targetPressure, sensorValue);}
    #endif
    } 
} // end loop()

/******************************************************************************
*                           VOID INIT_VALVE:
******************************************************************************/
void init_valve()
{
    digitalWrite(MOTOR_LED, HIGH); // motor power light ON
    // Enable to coils.
   if(motorPower == false){ 
    targetPressure = analogRead(PRESSURE);
    DEBUG_PRINT("\n\n\n TARGET PRESSURE: ")
    DEBUG_PRINTNL(targetPressure);
    myMotor->step(INIT_STEP, BACKWARD, INTERLEAVE);
    delay(1);
    myMotor->step(INIT_STEP, FORWARD, INTERLEAVE);
    motor = true;
    
    initTime = millis();
     } 
}// end init_valve

/******************************************************************************
*                           VOID OPEN_VALVE:
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
