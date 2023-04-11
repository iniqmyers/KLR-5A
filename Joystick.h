//#pragma once
#include "teensystep4.h"  // Library for fast, asynchronous stepper motor control on Teensy4
using namespace TS4;      // Namespace for TeensyStep4
#include "Arduino.h"   
#include <Bounce2.h>      // Library for debouncing inputs
#include "RobotAxis.h"
//#include "ArduPID.h"

enum axis {X,Y,Z};
axis& operator++(axis& orig) {  //pre increment operator
  orig = static_cast<axis>(orig + 1);
  return orig;
}
axis operator++(axis& orig, int) {  //post increment operator
  axis rVal = orig;
  ++orig;
  return rVal;}

class JoyStick{
  private:
    int teachPendantPinX;
    int teachPendantPinY;
    int teachPendantPinZ;
    int buttonTeachPendantPin;
    //uint16_t Xpos;             // For reading/calculating Joystick X input [0-1023]
                              // ****** shouldn't need, call getPosition() when needed
    uint16_t home[3];            // Home value for zeroing Joystick input [0-1023]
    //uint16_t Zpos;             // For reading/calculating Joystick Z input [0-1023]
    //uint16_t Zhome;            // Home value for zeroing Joystick Z input [0-1023]
    uint8_t  Deadzone; // How far off of home does joystick need to be to activate? [0-255] <--- We will want to have one for each axis, and will want to build a calibration routine
    Bounce buttonBounce; // Define debounce object for joystickButton
  public:
    JoyStick(int pinX, int pinY, int pinZ, int buttonPin); //constructor
    void setPinModes();    
    uint16_t getHome(axis direction);
    uint8_t getDeadzone() {return Deadzone;}
    uint16_t getPosition(axis direction);       
    void getXYZ(uint16_t &x, uint16_t &y, uint16_t &z);
    void setHome();
    void rotate(axis direction, RobotAxis& robAxis, uint16_t speed);    //use an enumerated type for direction
};//end of Joystick class

JoyStick::JoyStick(int pinX, int pinY, int pinZ, int buttonPin) {
      teachPendantPinX = pinX;
      teachPendantPinY = pinY;
      teachPendantPinZ = pinZ;
      buttonTeachPendantPin = buttonPin;      
      Deadzone = 50;     
      buttonBounce = Bounce();    
} //end of constructor

void JoyStick::setPinModes() { pinMode(teachPendantPinX,INPUT); //**** is INPUT an enumerated type? <-- Probably, and it's also probably defined wherever pinMode is...
                        pinMode(teachPendantPinY,INPUT); //**** would this ever be set to something besides INPUT <----No, for analog inputs this will always just be INPUT. Technically this is redundant since analog pins are input by default
                        pinMode(teachPendantPinZ,INPUT);  //****which library does pinMode belong to?<--- I guess Arduino.h but I'm having difficulty figuring that out...
                        // pinMode(JOYSTICK_BUTTON,INPUT); //Joystick Button
} //end of setPinModes    

void JoyStick::getXYZ(uint16_t &x, uint16_t &y, uint16_t &z){
          x = getPosition(axis::X);
          y = getPosition(axis::Y);
          z = getPosition(axis::Z);                  
} // end of getAllPositions 

void JoyStick::setHome(){
  axis direction = axis::X;
  for(int i=0;i<3;i++)
    home[i] = getPosition(direction++);
} //end of setHome

uint16_t JoyStick::getHome(axis direction){
  return home[direction];
}

uint16_t JoyStick::getPosition(axis direction){
  switch(direction){
    case X: return analogRead(teachPendantPinX); break;
    case Y: return analogRead(teachPendantPinY); break;
    case Z: return analogRead(teachPendantPinZ); break;
  }
return 0;  //this would be an error
}

void JoyStick::rotate(axis direction, RobotAxis& robAxis, uint16_t speed){
  int position=0;
  uint16_t currentHome=0;

  switch(direction){
    case X: position = getPosition(axis::X); 
                        currentHome = home[0]; break;
    case Y: position = getPosition(axis::Y); 
                        currentHome = home[1]; break;
    case Z: position = getPosition(axis::Z); 
                        currentHome = home[2]; break;                                       
  }
      if(abs(position-currentHome)>Deadzone){ //Is movement greater than deadzone?
        if(position<currentHome){//Stick Pushed Right, Rotate Right
          robAxis.enable();
          robAxis.rotate(speed,-(position-currentHome)/512.0);
          //myMotor.overrideSpeed(-(position-currentHome)/512.0); //Scale motor to axis 
        }else{//Stick Pushed Left, Rotate Left
          robAxis.enable();
          robAxis.rotate(speed,(currentHome-position)/512.0);
         // myMotor.overrideSpeed(((currentHome-position)/512.0));
        }
      }else{ //Not outside deadzone
         robAxis.disable();
         // myMotor.overrideSpeed(0.0); //Set speed 0
      }
}
