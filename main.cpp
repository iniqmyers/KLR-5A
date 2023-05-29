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

// ################# Constant Declarations ########################
#define JOYXPIN 21
#define JOYYPIN 22
#define JOYZPIN 23
#define JOYBUT 20

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
int timer = 0;
int lastTimer = 0;

uint16_t maximumSpeed = 8000;         // Maximum motor speed      (STP+DIR)
uint16_t acceleration = 25000;        // Motor Acceleration       (STP+DIR)
uint16_t speed = 3000;                // Motor Speed              (STP+DIR)

uint16_t  homingRotationSpeed = 2000; // Motor speed while homing (STP+DIR)
int32_t   homingRange;                // Total range between endstops
int32_t   homingBottom;               // Bottom endstop position
int32_t   homingTop;                  // Top endstop position
bool      homing = false;             // Is the robot in homing state (allows for running the endstops without disabling motors)   
uint16_t  rawPos[5];                  // Axis3 Raw analog encoder position
bool      a3end = false;              // Axis3 endstop state
uint8_t   encoderPins[5];             // Array of analog pins for updating encoder values

volatile bool estop = true;
bool mstop = true;
double Kp2 =0.022, Ki2 = 0, Kd2 = .0001; // Define PID tuning values
double Kp3 =.05, Ki3 = 0, Kd3 = .0001; // Define PID tuning values
double Kp4 = 0,Ki4=0,Kd4=0;
int targetPosition; // Define target position for PID controller

// %%%%%%%%%%%%%%%%%%%%%% Stepper Motor Declarations %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Stepper axis1(AXIS1STP, AXIS1DIR); // Define stepper motor with Step and Direction pins
//Stepper axis2(AXIS2STP, AXIS2DIR); // Define stepper motor with Step and Direction pins
//Stepper axis4(AXIS4STP, AXIS4DIR);  // Define stepper motor with Step and Direction pins
//Stepper axis5(AXIS5STP, AXIS5DIR);  // Define stepper motor with Step and Direction pins
RobotAxis axisThree(AXIS3ENC,AXIS3END,AXIS3HOM,AXIS3EN,AXIS3DIR,AXIS3STP,34,35,Kp3,Ki3,Kd3);
RobotAxis axisTwo(AXIS2ENC,AXIS2HOM,AXIS2HOM,AXIS2EN,AXIS2DIR,AXIS2STP,34,35,Kp2,Ki2,Kd2);
RobotAxis axisFour(AXIS4ENC,AXIS4HOM,AXIS4HOM,AXIS4EN,AXIS4DIR,AXIS4STP,34,35,Kp4,Ki4,Kd4);
JoyStick joystick(JOYXPIN,JOYYPIN,JOYZPIN,JOYBUT);
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



// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<   SETUP (Run Once at Startup)  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void setup()
{
  estop = false; //Allow robot motion
 // mstop = true;  //Force ManualControl
  Serial.begin(115200); //Begin USB serial for debugging
  Serial.println("Start");
  Serial.print("Homing Joystick, Leave Centered...");
  setupIO(); // Set pins to Input/Output modes and zero joystick axes
  Serial.print("...");
  setupMotors(); //Enable outputs, begin TS4 and set speeds
  Serial.println("done.");
  targetPosition = 45 ; //Set target position for PID axis control
  axisThree.setTargetPosition(targetPosition);
  joystick.invertY();
}

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> LOOP (Run repeatedly after Setup) >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void loop()
{
  timer = millis();
 // Serial.print("Estop:");
 // Serial.print(estop);
 // Serial.print(" Mstop:");
 // Serial.print(mstop);  
 axisThree.endStop.update();
  //Serial.println("Next");
  if(!axisThree.endStop.read()){
    axisThree.disable();
    //Serial.println("Disabled");
    mstop = true;
    Serial.println("Endstop Limit Switch Activated! Stopping Motors");
    Serial.println("Recover Robot Manually (DO NOT CRASH!)");
    if(103>axisThree.getEncoderPosition()||axisThree.getEncoderPosition()>-106){
      Serial.println("Encoder/Endstop Position Mismatch! Check Alignment");
    }
  }
  
  

  if(!estop&&!mstop){
   // Serial.println("Auto");
    if(abs(axisThree.getPosition()-targetPosition)<0.2){
      //axisThree.disable();

      if(timer-lastTimer>5000){
      Serial.print(timer-lastTimer);
      //delay(2000);
      Serial.print("Target Reached, new target:");
      
      targetPosition = random(-90,100);

      Serial.println(targetPosition);
      axisThree.setTargetPosition(targetPosition);
      lastTimer = millis();
      }


    }
    joystick.rotate(Z,axisFour,speed);
    joystick.rotate(Y,axisTwo,speed);
    axisThree.tick();
  }else{
    if(mstop&&!estop){
      joystick.rotate(X,axisThree,speed);
      axisTwo.updatePosition();
      //Serial.print(axisThree.getPosition());
     // Serial.print(" | ");
      Serial.println(axisTwo.getMotorPosition());
      joystick.rotate(Z,axisFour,speed);
      joystick.rotate(Y,axisTwo,speed);
      axisThree.homeSensor.update();
      if(axisThree.homeSensor.read()){
        mstop = false;
      } 
  }else{
    Serial.println("FullStop");
     
    }
  }
  
}

