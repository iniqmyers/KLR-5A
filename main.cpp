// KLR-5A [Klarissa], 5-axis 3d-printed Robot Arm Controller
// ©2023 Iniq Myers
// Released under GPLv3
// Controller Hardware: Teensy 4.1
// Closed-loop Motor Controllers:
//    Axis1,2,3 - SKR Servo042C V1.1.1  (Step+Dir and UART) [Nema 17 2.0A]
//    Axis4 - Bigtreetech S42C V1.0 (Step+Dir and UART) [Nema 17 1.5A]
// Home positions determined using magnetic hall-effect sensors
// Axis limits defined with micro limit switches (axis 4 has no hall sensor, only microswitch)
// Motors inteface with 3d-printed Split-ring compound planetary epicyclic gearboxes (credit @skyentific)
// Body and linkage design originates from Skyentific 5-axis robot, but modified to resemble Kuka KR600/700
// All parts printed in PLA or Syriatech BluV2[SLA]
// Teach pendant has a 3-axis joystick and emergency stop to kill motor power


// ^^^^^^^^^^^^^^ Libraries ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
#include "Arduino.h"      // Library for supporting standard Arduino functions on Teensy Hardware
#include "Joystick.h"     // Custom Library for controlling Axes with TeachPendant Joystick
#include <EEPROM.h>       // Library for storing/recalling data from onboard EEPROM
#include <Bounce2.h>      // Library for debouncing inputs
#include "ArduPID.h"      // Library for PID motor control
#include "teensystep4.h"  // Library for fast, asynchronous stepper motor control on Teensy4
#include "RobotAxis.h"    //Custom Library for controlling motor/encoder and sensors as a single axis object
using namespace TS4;      // Namespace for TeensyStep4

// $$$$$$$$$$$ function prototypes
void setupIO(); // Set pins to Input/Output modes and zero joystick axes
void setupMotors(); //Enable outputs, begin TS4 and set speeds

JoyStick joystick(21,22,23,20);

// ################# Constant Declarations ########################
// Motor Driver I/O Pins
/*
#define AXIS1EN  XX // Axis1 - Motor Enable (Low Active)
#define AXIS1DIR XX // Axis1 - Motor Direction
#define AXIS1STP XX // Axis1 - Motor Step
*/
#define AXIS2EN  6  // Axis2 - Motor Enable (Low Active)
#define AXIS2DIR 8  // Axis2 - Motor Direction
#define AXIS2STP 7  // Axis2 - Motor Step

#define AXIS3EN  35 // Axis3 - Motor Enable (Low Active)
#define AXIS3DIR 33 // Axis3 - Motor Direction
#define AXIS3STP 34 // Axis3 - Motor Step

#define AXIS4EN  38 // Axis4 - Motor Enable (Low Active)
#define AXIS4DIR 36 // Axis4 - Motor Direction
#define AXIS4STP 37 // Axis4 - Motor Step
/*
#define AXIS5EN  XX // Axis5 - Motor Enable
#define AXIS5DIR XX // Axis5 - Motor Direction
#define AXIS5STP XX // Axis5 - Motor Step
*/
// Motor Feedback I/O Pins
/*
#define AXIS1HOM XX // Axis1 - Home Position Magnetic Sensor
#define AXIS1ENC XX // Axis1 - Magnetic Absolute Position Encoder (Analog)
#define AXIS1END XX // Axis1 - Endstop Limit Switch (Both Sides)
*/
#define AXIS2HOM 10 // Axis2 - Home Position Magnetic Sensor
#define AXIS2ENC 19 // Axis2 - Magnetic Absolute Position Encoder (Analog)
#define AXIS2END 8  // Axis2 - Endstop Limit Switch (Both Sides)

#define AXIS3HOM 11 // Axis3 - Home Position Magnetic Sensor
#define AXIS3ENC 40 // Axis3 - Magnetic Absolute Position Encoder (Analog)
#define AXIS3END 31 // Axis3 - Endstop Limit Switch (Both Sides)

#define AXIS4HOM 39 // Axis4 - Home/Endstop Limit Switch (Infinite Rotation)
#define AXIS4ENC 41 // Axis4 - Magnetic Absolute Position Encoder (Analog)
/*
#define AXIS5HOM XX // Axis5 - Home Position Magnetic Sensor
#define AXIS5ENC XX // Axis5 - Magnetic Absolute Position Encoder (Analog)
#define AXIS5END XX // Axis5 - Endstop Limit Switch (Both Sides)
*/
// Generic
#define LED 13 //Onboard Feedback Led


// ************************** Variable Declarations *******************************

uint16_t maximumSpeed = 6000;         // Maximum motor speed      (STP+DIR)
uint16_t acceleration = 25000;        // Motor Acceleration       (STP+DIR)
uint16_t speed = 9000;                // Motor Speed              (STP+DIR)

uint16_t  homingRotationSpeed = 2000; // Motor speed while homing (STP+DIR)
int32_t   homingRange;                // Total range between endstops
int32_t   homingBottom;               // Bottom endstop position
int32_t   homingTop;                  // Top endstop position
bool      homing = false;             // Is the robot in homing state (allows for running the endstops without disabling motors)   
uint16_t  rawPos[5];                  // Axis3 Raw analog encoder position
bool      a3end = false;              // Axis3 endstop state
uint8_t   encoderPins[5];             // Array of analog pins for updating encoder values

volatile bool a3home=false;           // The current home switch status of Axis 3
volatile bool a4home=false;           // The current home switch status of Axis 4

volatile bool estop = true;
bool mstop = true;

//double input3,output3,setpoint3=512;
double input4,output4,setpoint4=512;
double Kp3 =0.022, Ki3 = 0, Kd3 = .0001; // Define PID tuning values
double Kp4 =.01, Ki4 = 0.0, Kd4 = 0; // Define PID tuning values
//ArduPID a3PID; // Define PID Object
ArduPID a4PID;
int targetPosition; // Define target position for PID controller

// %%%%%%%%%%%%%%%%%%%%%% Stepper Motor Declarations %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Stepper axis1(AXIS1STP, AXIS1DIR); // Define stepper motor with Step and Direction pins
Stepper axis2(AXIS2STP, AXIS2DIR); // Define stepper motor with Step and Direction pins
Stepper axis4(AXIS4STP, AXIS4DIR);  // Define stepper motor with Step and Direction pins
//Stepper axis5(AXIS5STP, AXIS5DIR);  // Define stepper motor with Step and Direction pins
RobotAxis axisThree(AXIS3ENC,AXIS3END,AXIS3HOM,AXIS3EN,AXIS3DIR,AXIS3STP,0,1,Kp3,Ki3,Kd3);
// ()()()() Other Declarations ()()()()

void setupIO(){ // Setup pin modes for I/O
  joystick.setPinModes();  

  encoderPins[0]= AXIS3ENC; //Set to axis3 while testing
  encoderPins[1]= AXIS3ENC; //Set to axis3 while testing
  encoderPins[2]= AXIS3ENC;
  encoderPins[3]= AXIS4ENC;
 // encoderPins[4]= AXIS5ENC;
 // pinMode(AXIS1EN,OUTPUT);       // Axis 1 Enable (Step + Direction set by TS4)
  pinMode(AXIS2EN,OUTPUT);        // Axis 2 Enable (Step + Direction set by TS4)
  pinMode(AXIS3EN,OUTPUT);        // Axis 3 Enable (Step + Direction set by TS4)
  pinMode(AXIS4EN,OUTPUT);        // Axis 4 Enable (Step + Direction set by TS4)
 // pinMode(AXIS5EN,OUTPUT);       // Axis 5 Enable

  // Record Zero Position for all joystick axes to calculate offsets 
  joystick.setHome();  
  }

void setupMotors(){ // Enable motor outputs, begin TS4 service and set motor speeds

  // Enable Motor Outputs
  //digitalWrite(AXIS1EN,LOW); //Low Active, enable axis1 motor motion
  digitalWrite(AXIS2EN,LOW); //Low Active, enable axis2 motor motion
  digitalWrite(AXIS3EN,LOW); //Low Active, enable axis3 motor motion
  digitalWrite(AXIS4EN,LOW); //Low Active, enable axis4 motor motion
  //digitalWrite(AXIS5EN,LOW); // enable axis5 motor motion

  TS4::begin(); //Begin TeensyStep4 Service

  // Set Motor Speeds and acceleration
  axis2.setMaxSpeed(maximumSpeed);
  axis2.setAcceleration(acceleration);
  axis4.setMaxSpeed(maximumSpeed);
  axis4.setAcceleration(acceleration);

}


void updatePositions(){
  for(int i=0;i < 5;i++){ // For each axis
      rawPos[i]=analogRead(encoderPins[i]); //Read encoder pin[0-1023] and write to rawPos[axis-1]
  }
}

uint16_t updatePosition(uint8_t axis){
  rawPos[axis-1]=analogRead(encoderPins[axis-1]);
  return rawPos[axis-1];
}

void a3homingTest(){
  
}


// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<   SETUP (Run Once at Startup)  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void setup()
{
  estop = false; //Allow robot motion
  mstop = true;  //Force ManualControl
  Serial.begin(115200); //Begin USB serial for debugging
  Serial.print("Homing Joystick, Leave Centered...");
  setupIO(); // Set pins to Input/Output modes and zero joystick axes
  Serial.print("...");
  setupMotors(); //Enable outputs, begin TS4 and set speeds
  Serial.print("done.");
  //a3homingTest(); //Home axis3, verify homing switch, endstops and magnetic position encoder
  targetPosition = 700; //Set target position for PID axis control
  /* $$$$$$$$$$$ Will be handled internal to the axis object
  setpoint3 = targetPosition;

  a3PID.begin(&input3,                // input
              &output3,               // current output
              &setpoint3,             // setpoint
              Kp3,Ki3,Kd3          );   // P,I,D
  a3PID.setSampleTime(25);
  a3PID.start();
  a3PID.setOutputLimits(-1,1);
  */

  a4PID.begin(&input4,                // input
              &output4,               // current output
              &setpoint4,             // setpoint
              Kp4,Ki4,Kd4          );   // P,I,D
  a4PID.setSampleTime(25);
  a4PID.start();
  a4PID.setOutputLimits(-1,1);
}

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> LOOP (Run repeatedly after Setup) >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void loop()
{
  // @#@#@#@#@#@#@#@#@# Safety Checks @#@#@#@#@#@#@#@#@#@#@#
  axisThree.endStop.update();
  if(axisThree.endStop.fell()){
    axisThree.disable();
    mstop = true;
    Serial.println("Endstop Limit Switch Activated! Stopping Motors");
    Serial.println("Recover Robot Manually (DO NOT CRASH!)");
    if(103>axisThree.getPosition()||axisThree.getPosition()>-106){
      Serial.println("Encoder/Endstop Position Mismatch! Check Alignment");
    }
  }

  if(!estop&&!mstop){
    axisThree.tick();
    a4PID.compute();

   
    if(abs(output4)>0){
      axis4.rotateAsync(maximumSpeed);
      axis4.overrideSpeed(-output4);
    }else{
      if(input4==setpoint4){
        axis4.stop();
        Serial.print(" Target Reached: ");
        Serial.println(((input4/1023.0)*360)-180);
        if(input4==setpoint4){
          int newTarget = random(220,511);
          while(abs(newTarget-setpoint4)<10){
            newTarget = random(220,511);
          }
          setpoint4 = newTarget;
          Serial.print(" New Target: ");
          Serial.print(setpoint4);
          Serial.print("...");
        }
      }
      
    }

    
  }else{
    //*************** Manual Control Mode ********************
    updatePositions();
   // int a3nicePos = map (rawPos[2],0,1023,-180,180); //Map raw encoder to degrees
    // *#*#*#*#*#*#*#*#*#*#*#*#*# Safety Checks *#*#*#*#*#*#*#*#*#*#*#*#
    axisThree.endStop.update(); //update endstop bounce
    if(axisThree.endStop.fell()){ //if endstop triggered
      axisThree.disable();
      //estop = false;
      mstop = true;
      Serial.println("Endstop Limit Switch Activated! Stopping Motors");
      if(axisThree.getPosition()>-106 || axisThree.getPosition()<103){ //Is the encoder outside an expected endstop zone?
        Serial.println("Encoder/Endstop Position Mismatch! Check Alignment");
      }
    }
    axisThree.homeSensor.update(); //update home switch bounce
    if(axisThree.homeSensor.rose()){ //if home switch activated
        a3home = true;
        a3end = false;
        //Serial.println("Home Position Detected");
        mstop = false;
      // estop = true;
      if(axisThree.getPosition()<-5 || axisThree.getPosition()>8){ //Is the encoder outside the acceptable homing zone?
        //estop = true;
        mstop=true;
        Serial.println("Homing Sensor / Encoder Mismatch");
        Serial.println(axisThree.getPosition());
      }
    }
    //#*#*#*#*#*#* Safety Checks Complete, Allow Motion Calculation *#*#*#*#**#*#

  //Manual Joystick Inputs to Motor Outputs
    if(!estop){ //Skip all if estop
        //joystick.rotate(axis::Y, axis3, speed);
        joystick.rotate(axis::X, axis2, speed);
        joystick.rotate(axis::Z, axis4, speed);
        
        if(abs(joystick.getPosition(Y)-joystick.getHome(Y))>joystick.getDeadzone()){
          axisThree.rotate(speed,(-(joystick.getPosition(Y)-joystick.getHome(Y))/512.0));
        }else{
          axisThree.rotate(speed,0.0);
        }
    }else{ //Estop = true!!
    //  axis1.overrideSpeed(0.0);
      axis2.overrideSpeed(0.0);
     // axis3.overrideSpeed(0.0); //Set motor speed 0
      axis4.overrideSpeed(0.0);
      //axis5.overrideSpeed(0.0);
    }
  }
  
  
  
  
  
 
}
