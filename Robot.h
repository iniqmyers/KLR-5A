#include "Arduino.h"   
#include <Bounce2.h>      // Library for debouncing inputs 
#include "teensystep4.h"  // Library for fast, asynchronous stepper motor control on Teensy4
#include "RobotAxis.h"
#include "Joystick.h"
using namespace TS4;      // Namespace for TeensyStep4

class Robot{
  private:
    enum controlMode {T1,T2,AUTO,AUTOEXT};
    controlMode& operator++(controlMode& orig) {  //pre increment operator
      orig = static_cast<controlMode>(orig + 1);
      return orig;
    }
    controlMode operator++(controlMode& orig, int) {  //post increment operator
    controlMode rVal = orig;
    ++orig;
    return rVal;}

    RobotAxis axis[5];
    int currentPose[5];
    int targetPose[5];
    bool estop;
    bool mstop;
    bool fault;
    int faultCode
    bool moving;
    bool calibrated;
    Joystick pendant;

  public:
    Robot();
    
    void enableMotors();
    void disableMotors();

    void getCurrentPose();
    void getTargetPose();
    void setTargetPose();
    


};//end of Robot class

Robot::Robot() {

      encoderPin = encPin;
      endstopPin = endPin;
      homingPin = homPin;
      enablePin = enPin;
      directionPin = dirPin;
      stepPin = stpPin;
      serialTxPin = txPin;
      serialRxPin = rxPin;
      Kp = p;
      Ki = i;
      Kd = d;
      homingSpeed = 5000;
      motor = Stepper(stpPin, dirPin);
     // TS4::begin(); //Begin TeensyStep4 Service
      homeSensor = Bounce();
      endStop = Bounce();
      calibrated = false;
      moving = false;
      fault = true;
      enabled = false;
      faultCode = 7; //Not Calibrated
      //Check EEPROM for Hard or Soft Stop Stored Positions
      //If valid, clear fault and change faultCode to 6 for "Unverified Calibration" 
} //end of constructor

void Robot::