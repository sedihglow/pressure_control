//change
// Program currently used as of 5-18-15        
 
// ISSUES can not program multiple boards at a time. The serial port keeps switching. 
// Need to go to Tools>Port>and click on the port number it shows you to keep programing.

#include <Wire.h>
#include "Adafruit_MotorShield.h"  //  using " means look in current directory too.
#include "Adafruit_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // create shield object
Adafruit_StepperMotor *myMotor = AFMS.getStepper(200, 2); // stepper object, counts, port#2 (m3 m4)// creates motor object, one for each motor

 /******Pins definitions*************/
        #define MOTORSHIELD_IN1 8          //constant
        #define MOTORSHIELD_IN2 11        //constant
        #define MOTORSHIELD_IN3 12        //constant
        #define MOTORSHIELD_IN4 13        //constant
        #define CTRLPIN_A   9   //constant
        #define CTRLPIN_B   10  //constant
        int sensorPin = A0;    // pressure transducer input pin
        float TargetPressure = 660.0;  // set this to the target pressure to run at
        float VarPause = 5000.0;  // in milliseconds
        int SensorValue = 5;
        int millis_delay = 5; // pause between opening and closing valve
        int IncreaseButton = 0;
        int DecreaseButton = 0;
        int OffButton = 0;
        int MotorPowerLight = 0;
        int varOpen = 0;  // holds state of valve
        int varMotorState = 1; // holds state of motor switch. Used to activate motors first time out of button change.
        int x = 0; // excel plot variable
        int row = 0; // excel plot variable

    //***************** SETUP *************************************
       void setup()  // Setup called once in every program to initialize everything. 
       {
        	//initialize();//Initialization for the stepper motor.
           AFMS.begin();  // create with the default frequency 1.6KHz
            //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
            myMotor->setSpeed(60);  // xx rpm
            myMotor->release() ; 

            Serial.begin(9600);  // prints out analog voltage on display
            SensorValue = analogRead(A0);
            pinMode(2, INPUT); // pressure increase button on input pin 2
            pinMode(3, INPUT); // pressure decrease button on input pin 3
            pinMode(4, INPUT); // sepper motor off button on input pin 4
            
            pinMode(5, OUTPUT); // Motor power green LED

            //Serial.begin(128000); // opens serial port, sets data rate to 9600 bps
            //Serial.println("CLEARDATA"); // excel plot
            //Serial.println("LABEL,Time,sin(x)"); // excel plot
        }

    //************ LOOP *******************************************
    void loop()
    {
       SensorValue = analogRead(A0);
       DecreaseButton = digitalRead(2);
       IncreaseButton = digitalRead(3);
       OffButton = digitalRead(4);

             float voltage = SensorValue ;     // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
             //Serial.print("target= ");
             //Serial.println(TargetPressure);
             //Serial.print("sensor value = ");
             //Serial.println(SensorValue);
             //Serial.print("sensor - target= ");
             //Serial.println(SensorValue-TargetPressure);
             //Serial.print("delay = ");
             //Serial.println(pow((TargetPressure-SensorValue),2)*.01);
             Serial.println(OffButton);

             //Serial.println();
             //Serial.println();	

       //while(digitalRead(4)==0){delay(200);} // Serial.println("holding");} // stop button activated

    //**************************Motor Power Logic **********************
         
        if(OffButton==0){ myMotor->release(); Serial.println("Motor OFF");
              varMotorState = 0;
              digitalWrite(5, LOW); delay(200);} // turn off motor power light            

        else {digitalWrite(5, HIGH);  //motor power light on
              if(varMotorState == 0) // if starting from off state need to increment motor to activate coils
                  {myMotor->step(1, BACKWARD, INTERLEAVE);
                  myMotor->step(1, FORWARD, INTERLEAVE);
                  varMotorState = 1; }
                  // Serial.print("normal delay= ");
               }
       
    //********************** OPEN VALVE LOGIC *****************

        if (digitalRead(3)==0 && varOpen==0 && varMotorState==1) {TargetPressure=SensorValue;} // increasing pressure hold closed

            else if (digitalRead(3)==0 && varOpen==1&& varMotorState==1) {TargetPressure=SensorValue; myMotor->step(100, BACKWARD, INTERLEAVE);varOpen=0;} // close valve

            else if(SensorValue > TargetPressure && varOpen == 0 && varMotorState==1){myMotor->step(100, BACKWARD, INTERLEAVE); varOpen=1;} //Stepper motors rotate open

            else if(SensorValue > TargetPressure && varOpen == 1 && varMotorState==1){} // exit do nothing

            else if(DecreaseButton==0 && varOpen==0 && varMotorState==1){myMotor->step(100, BACKWARD, INTERLEAVE);varOpen=1;TargetPressure=SensorValue;} // open valve to decrease setting


// ********************CLOSE VALVE LOGIC ***************************

        if(DecreaseButton==0 && varMotorState==1 && varMotorState==1){TargetPressure=SensorValue; Serial.println("Decreasing");delay(200);} // Decrease pressure button down            

        else if(varOpen==1 && SensorValue < TargetPressure && varMotorState==1){varOpen=0;myMotor->step(100, FORWARD, INTERLEAVE);  //Stepper motors rotate closed
                  // Serial.print("normal delay= ");
               }

//****************EXCEL PLOT OUTPUT **********************************************
               //Serial.print("DATA,TIME,");  Serial.println(SensorValue);
                //row++;
               // x++;
               // if (row > 360) 
                // {
                 // row=0;
                 // x=0;
                 // Serial.println("ROW,SET,2");
                 //}
                
              //myMotor->step(100, FORWARD, INTERLEAVE); 
              
              //delay(2000);
              //myMotor->step(100, BACKWARD, INTERLEAVE); 

    }
