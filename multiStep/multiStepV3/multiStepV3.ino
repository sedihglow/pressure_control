/******************************************************************************
*JAMES ROSS 
*6 FEBRUARY 2016
*PROJECT: PSI pressure value w/ stepper motor.
*
*ABSTRACT: Program to monitor pressure and adjust by
*activating a stepper controlled preseure relief valve.
******************************************************************************/
 
//#define DEBUG 1    //comment out to suppress debug prints
#include <Wire.h>  
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_MotorShield.h>
#include "multiStepV3.h"

//************************ function prototypes ********************************
void motor_off();                   // checks power button, resets values.
void init_valve();                  // initialize the LED, coils.
int16_t step_level();               // check what step size is required.
void open_valve(int16_t stepSize);  // opens valve, decreases pressure.
void close_valve(int16_t stepSize); // close valve, increase pressure.
//void increase_pressure_ISR();       // interupt service routine
//void decrease_pressure_ISR();       // interupt service routine

//**************************** MACROS *****************************************
/******************************************************************************
*        MAN_OPEN: routine for opening valve manually setting flag
******************************************************************************/
#define MAN_OPEN(stepSize, initFlag){                                          \
    manualOpenStep += stepSize;                                                \
    open_valve(stepSize);                                                      \
    initFlag = true;                                                           \
}//end MAN_OPEN

/******************************************************************************
*               STEP_LEVEL: Find appropriate step level
******************************************************************************/
#define STEP_LEVEL(stepLevel, pressure, target){                               \
    int16_t DIST = DIST_TGT(target, pressure);                                 \
    DEBUG_STATE_PRINT("STEP_CHECK", target, pressure);                         \
    DEBUG_PRINT("DIST = ");                                                    \
    DEBUG_PRINTNL(DIST);                                                       \
    if     (DIST < OFFSET_ONE  ) {  stepLevel = LVL_ZERO;   }                  \
    else if(DIST > OFFSET_SIX  ) {  stepLevel = LVL_SIX;    }                  \
    else if(DIST > OFFSET_FIVE ) {  stepLevel = LVL_FIVE;   }                  \
    else if(DIST > OFFSET_FOUR ) {  stepLevel = LVL_FOUR;   }                  \
    else if(DIST > OFFSET_THREE) {  stepLevel = LVL_THREE;  }                  \
    else if(DIST > OFFSET_TWO  ) {  stepLevel = LVL_TWO;    }                  \
    else if(DIST > OFFSET_ONE  ) {  stepLevel = LVL_ONE;    }                  \
}//end STEP_LEVEL

//****************** variable declarations ************************************
volatile int16_t targetPressure = 0;   // Arbitrarily set to about 3000psi.
volatile int16_t sensorValue    = 0;   // Return value from pressure read.
volatile int16_t stepLvl        = 0;   // selects step level for pressure
int16_t openStepCount  = 0;   // count from starting position opened.
int16_t manualOpenStep = 0;   // The steps manually opening valve occured.
bool    manualInit     = LOW; // manual initialization was used
bool    switchDn       = LOW; // active low switch flag, toggle down
bool    switchUp       = LOW; // active low switch flag, toggle up
bool    motorPower     = LOW; // keep motor off/on

// shield object
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// stepper object w/ number of motors. counts, port#2 (m3 m4)
Adafruit_StepperMotor *myMotor = AFMS.getStepper(STEP_REV, STEP_CH);

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
    while(LOW == digitalRead(BUTTON_OFF)){ motor_off(); }//check power button

    init_valve();    // initialize the valve and set initial targetPressure
    manual_toggle(); // check for manual override with toggle switch

    sensorValue = analogRead(PRESSURE);
    STEP_LEVEL(stepLvl, sensorValue, targetPressure); // stepLvl from pressure
    DEBUG_PRINT("STEP_LEVEL = ");
    DEBUG_PRINTNL(stepLvl);

//**************** Acheive target pressure ************************************
    if(sensorValue > targetPressure){
        open_valve(stepLvl);
        DEBUG_STATE_PRINT("HIGH", targetPressure, sensorValue);
    }//end if 
    else if(sensorValue < targetPressure){
        close_valve(stepLvl+DBL_QCK_STEP);
        DEBUG_STATE_PRINT("LOW", targetPressure, sensorValue);
    }//end else if
    else{
        myMotor->release(); 
        DEBUG_STATE_PRINT("STEADY", targetPressure, sensorValue);
    }// end else
} // end loop()

/******************************************************************************
*                       VOID MOTOR_OFF:
******************************************************************************/
void motor_off(){
    myMotor->release(); 
    digitalWrite(MOTOR_LED, LOW); // motor power light OFF
    motorPower     = LOW;
    openStepCount  = 0;
    manualOpenStep = 0;
    DEBUG_PRINTNL("Motor OFF");
}//end motor_off

/******************************************************************************
*                       VOID INIT_VALVE:
******************************************************************************/
void init_valve(){
    digitalWrite(MOTOR_LED, HIGH); // motor power light ON
    // Enable to coils.
    if(false == motorPower){ 
        targetPressure = analogRead(PRESSURE);
        myMotor->step(INIT_STEP, BACKWARD, INTERLEAVE);
        delay(10);
        myMotor->step(INIT_STEP, FORWARD, INTERLEAVE);
        delay(10);
        motorPower = true;
    }//end if 
}// end init_valve

/******************************************************************************
*                       VOID MANUAL_TOGGLE:
******************************************************************************/
void manual_toggle(){
    do{ // check toggle switch, react when appropriate
        if(LOW == (switchDn = digitalRead(BUTTON_DN))){
          sensorValue = analogRead(PRESSURE);
            if(manualOpenStep < MAN_MAX_SML && sensorValue > SET_MAN_PR){
                MAN_OPEN(QCK_STEP, manualInit);
                DEBUG_PRINTNL("HIGH pressure, SMALL max");
            }//end if
          else if(manualOpenStep < MAN_MAX_LRG && sensorValue <= SET_MAN_PR){
                MAN_OPEN(QCK_STEP, manualInit);
                DEBUG_PRINTNL("LOW pressure, LARGE max");
            }//end else if
         }//end if
        else if(LOW == (switchUp = digitalRead(BUTTON_UP))){
            close_valve(QCK_STEP);
            manualInit = true;
        }//end else
    }while(LOW == switchDn || LOW == switchUp);
    
    if(true == manualInit){ // if manual open was engaged
        targetPressure = analogRead(PRESSURE);
        close_valve(manualOpenStep);
        manualOpenStep = 0;
        manualInit = false;
    }// end if 
}//end manual_toggle

/******************************************************************************
*                       VOID OPEN_VALVE:
******************************************************************************/
void open_valve(int16_t stepSize){
    myMotor->step(stepSize, BACKWARD, INTERLEAVE);
    openStepCount += stepSize;
    delay(10);
    DEBUG_PRINTNL("OPENING Valve, Decreasing Pressure");
}// end open_valve

/******************************************************************************
*                       VOID CLOSE_VALVE:
******************************************************************************/
void close_valve(int16_t stepSize){
    if(openStepCount > 0){
        if((openStepCount - stepSize) >= 0){ // check for over-closing
            myMotor->step(stepSize, FORWARD, INTERLEAVE);
            delay(10);
            openStepCount -= stepSize;
        }//end if
        else{ // fully close the valve
            while(openStepCount > QCK_STEP){ // quick close
                myMotor->step(QCK_STEP, FORWARD, INTERLEAVE);
                delay(10);
                openStepCount -= QCK_STEP;
            }//end while
            while(openStepCount > 0){ // single step to fully closed
                myMotor->step(SINGLE_STEP, FORWARD, INTERLEAVE);
                delay(10);
                --openStepCount;
            }//end while
        }//end else
    }//end if
    DEBUG_PRINTNL("CLOSING Valve, Increasing Pressure");
}// end close_valve

/******************************************************************************
*                   INTERRUPT SERVICE ROUTINES:
******************************************************************************/
/* TODO: Use interupts to test pressure reading vs act. on varous versions of 
         the system for future reference */
/******************************************************************************
*           ISR TO CLOSE VALVE BASED ON BUTTON_DN PRESS:
******************************************************************************/
/* saved for future updates 
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
*                               EOF
******************************************************************************/
