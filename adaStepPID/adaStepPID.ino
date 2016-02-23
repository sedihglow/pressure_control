/***********************************************************
*JAMES ROSS adapted from <name>
*6 FEBRUARY 2016
*CLIENT: 
*PROJECT: PSI pressure value w/ stepper motor.
*
*ABSTRACT: Program to monitor pressure and adjust by
*activating a stepper controlled preseure relief valve.
***********************************************************/
 
//#define DEBUG      // comment out to suppress general debug prints
//#define PID_DEBUG  // comment out to suppress PID specific debug prints
#include <Wire.h>  
#include <PID_v1.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_MotorShield.h>
#include "adaStepPID.h"

volatile double targetPressure = 0.0;   // Arbitrarily set to about 3000psi.
double sensorValue = 0.0;               // Return value from pressure read.
double outputPID = 0.0;                 // PID variables for calculations.
int16_t openStepCount = 0;              // steps from inital valve placement
bool aggro = false;                     // tuning is aggressive true/false

// shield object
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// stepper object w/ number of motors. counts, port#2 (m3 m4)
Adafruit_StepperMotor *myMotor = AFMS.getStepper(STEP_REV, STEP_CH);
// PID object used to do the PID calculations.
PID myPID(&sensorValue, &outputPID, (double*)&targetPressure, KP, KI, KD, DIRECT);

// function prototypes
void init_valve();  // initialize the LED, coils
void open_valve(int16_t stepSize);  // opens valve, decreases pressure.
void close_valve(int16_t stepSize); // close valve, increase pressure.

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
    AFMS.begin();           // default frequency 1.6KHz
    Serial.begin(SERIAL_V); // prints out analog voltage on display

    // motor setup routines
    myMotor->setSpeed(XXRPM);
    myMotor->release(); 

    pinMode(PRESSURE,   INPUT);  // pressure reading from sensor
    pinMode(BUTTON_DN,  INPUT);  // pressure increase button
    pinMode(BUTTON_UP,  INPUT);  // pressure decrease button
    pinMode(BUTTON_OFF, INPUT);  // sepper motor off button
    pinMode(MOTOR_LED,  OUTPUT); // motor power green LED
    
    // Interupts only works on pins 2,3 for ATMEGA328
    attachInterrupt(0, decrease_pressure_ISR, CHANGE);
    attachInterrupt(1, increase_pressure_ISR, CHANGE);

    // TODO: think about moving away from prepressurizing the system.
    sensorValue = analogRead(PRESSURE); // the initial pressure.
    targetPressure = sensorValue;       // the initial target pressure.

    myPID.SetMode(AUTOMATIC);           // set PID
    myPID.SetOutputLimits(PID_LOW, PID_HIGH);

    init_valve(); // initialize the LED, coils and TODO: default position
} // end setup

/******************************************************************************
*                           VOID LOOP:
******************************************************************************/
void loop()
{/******************* Motor Power Logic **************************************/
    if(0==digitalRead(BUTTON_OFF)){ // check off button.
        myMotor->release(); 
        digitalWrite(MOTOR_LED, LOW); // motor power light OFF
        DEBUG_PRINTNL("Motor OFF");
    }//end if
    //else{// motor is ON


/*********************** APPLY PID *******************************************/
    //TODO: See notes in log book about myPID.SetTunings()
    if(DIST_TGT(targetPressure, sensorValue) < CLOG_VAL(targetPressure)){ 
        //myPID.SetTunings(KP, KI, KD); // standard reaction
        aggro = false;
    }//end if 
    else{ 
        //myPID.SetTunings(AGG_KP, AGG_KI, AGG_KD);  // aggerssive reaction
        aggro = true;
        DEBUG_STATE_PRINT("AGGRO", targetPressure, sensorValue);
    } //end else

    myPID.Compute();   // compute outputPID
    DEBUG_PID_PRINT(sensorValue, outputPID, targetPressure);//TODO: restrictive timer?

/**************** Acheive target pressure ************************************/
    if(outputPID > 0){
        if(false == aggro){ open_valve(OPEN_STD); }
        else              { open_valve(OPEN_AGG); }
        DEBUG_STATE_PRINT("HIGH", targetPressure, sensorValue);
    }//end if 
    else if(outputPID < 0){
        if(false == aggro){ close_valve(CLOSE_STD); }
        else              { close_valve(CLOSE_AGG); }
        DEBUG_STATE_PRINT("LOW", targetPressure, sensorValue);
    }//end else-if
    else{ DEBUG_STATE_PRINT("STEADY", targetPressure, sensorValue) }

//}//end else 
} // end loop()

/******************************************************************************
*                           VOID INIT_VALVE:
******************************************************************************/
void init_valve(){
    digitalWrite(MOTOR_LED, HIGH); // motor power light ON
    myMotor->step(INIT_STEP, BACKWARD, INTERLEAVE);
    delay(1);
    myMotor->step(INIT_STEP, FORWARD, INTERLEAVE);
}// end init_valve

/******************************************************************************
*                           VOID OPEN_VALVE:
******************************************************************************/
void open_valve(int16_t stepSize){
    myMotor->step(stepSize, BACKWARD, INTERLEAVE);
    ++openStepCount;
    DEBUG_PRINTNL("OPENING Valve, Decreasing Pressure");
}// end open_valve

/******************************************************************************
*                       VOID CLOSE_VALVE:
******************************************************************************/
void close_valve(int16_t stepSize){
    if(openStepCount > 0){
        myMotor->step(stepSize, FORWARD, INTERLEAVE);  
        --openStepCount;
        DEBUG_PRINTNL("CLOSING Valve, Increasing Pressure");
    }//end if
}// end close_valve
/******************************************************************************
*                               EOF
******************************************************************************/
