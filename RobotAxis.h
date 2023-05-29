#pragma once
#include "Arduino.h"   
#include <Bounce2.h>      // Library for debouncing inputs 
#include "teensystep4.h"  // Library for fast, asynchronous stepper motor control on Teensy4
//using namespace TS4;      // Namespace for TeensyStep4
#include "ArduPID.h"
namespace TS4{
    class RobotAxis{
      private:
        Stepper motor;
        int encoderPin;
        int endstopPin;
        int homingPin;
        int enablePin;
        int directionPin;
        int stepPin;
        int serialTxPin;
        int serialRxPin;
        int maximumSpeed;
        int homingSpeed;
        int position;
        int stepPosition;
        double degrees;
        int targetPosition;
        int targetSpeed;
        ArduPID positionController;
        double Kp,Ki,Kd;
        double input,output,setpoint;
        int hardTop; // The encoder value when the top stop is triggered
        int hardBottom; // The encoder value when the bottom stop is triggered
        int homePosition; // Center of the magnetic homing sensor range 
        int homeWidth;
        bool calibrated; //Home/End/Encoder sensors agree after a calibration routine
        bool enabled; //Drive is allowed to run by safety circuit
        bool moving; //Drive is currently moving
        bool fault; // Axis indicates fault on one or more parameters
        uint8_t faultCode; //Code indicating current [highest priority] fault




      public:
        Bounce homeSensor;
        Bounce endStop;
        RobotAxis();
        RobotAxis(int encoderPin, 
            int endstopPin, 
            int homingPin, 
            int enablePin, 
            int directionPin, 
            int stepPin,
            int serialTxPin, 
            int serialRxPin,
            double Kp,double Ki, double Kd); //constructor

        void setPinModes();        // This could just happen during object initialization, should never change at runtime
  //     void calibrateSensors();  // Run a physical calibration routine, running the axis to its endstops, recording positions
        void calibrateHomeSensor();
        void updatePosition();
        double getHomeOffset();
        double getPosition();
        int getEncoderPosition();
        int getMotorPosition();
        int getHome();
        int getHomeWidth();
        int getHardTop();
        int getHardBottom();
        bool isCalibrated();
        bool isEnabled();
        bool isFaulted();
        bool isMoving();
        uint8_t getFault();
        void setHome(int width);
        void setMotorHome();
        void disable();
        void enable();
        void setTargetPosition(int target);
        void setTargetPosition(int target,int speed);
        void rotate(uint16_t speed, double override);    //use an enumerated type for direction
        void tick();
    };//end of RobotAxis class

    RobotAxis::RobotAxis(int encPin, 
            int endPin, 
            int homPin, 
            int enPin, 
            int dirPin, 
            int stpPin, 
            int txPin, 
            int rxPin,
            double p,
            double i,
            double d) {

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
          maximumSpeed = 8000;
          motor = Stepper(stpPin, dirPin);
          motor.setMaxSpeed(maximumSpeed);
          motor.setAcceleration(12000);
        // TS4::begin(); //Begin TeensyStep4 Service
          homeSensor = Bounce();
          homeSensor.attach(homingPin,INPUT);
          homeSensor.interval(25);
          endStop = Bounce();
          endStop.attach(endstopPin,INPUT_PULLUP);
          endStop.interval(25);
          calibrated = false;
          moving = false;
          fault = true;
          enabled = false;
          faultCode = 7; //Not Calibrated
          //Check EEPROM for Hard or Soft Stop Stored Positions
          //If valid, clear fault and change faultCode to 6 for "Unverified Calibration" 
          positionController.begin(&input,                // input
              &output,               // current output
              &setpoint,             // setpoint
              Kp,Ki,Kd          );   // P,I,D
          positionController.setSampleTime(25);
          positionController.start();
          positionController.setOutputLimits(-1,1);
          motor.setPosition(0);


    } //end of constructor

    void RobotAxis::setPinModes() { pinMode(encoderPin,INPUT);
                                    //pinMode(endstopPin,INPUT);
                                    //pinMode(homingPin,INPUT);
                                    pinMode(enablePin,OUTPUT);
                                    pinMode(directionPin,OUTPUT);
                                    pinMode(stepPin,OUTPUT);
                                    pinMode(serialTxPin,OUTPUT);
                                    pinMode(serialRxPin,INPUT);
                                    homeSensor.attach ( homingPin , INPUT ); // Attach debounce object
                                    homeSensor.interval(10); // Define debounce interval (ms)
                                    endStop.attach(endstopPin,INPUT_PULLUP);
                                    endStop.interval(30);
    } //end of setPinModes    

  /* void RobotAxis::calibrateSensors(){
      uint16_t top,bot,hom1,hom2,hom3;
        Serial.print("Calibration Routine Begun: ");
        homeSensor.update(); //Process debounce on axis3Home
        endStop.update(); //Process debounce on axis3End
          Serial.print("Top Switch First...");
          motor.rotateAsync(-homingSpeed); //Rotate upwards at the homing speed
          while(!endStop.fell()){ //While the endstop hasn't triggered
            endStop.update(); //Update bounce input
          }
          motor.stop(); //Stop when endstop reached
          delayMicroseconds(100); //let inputs and motors settle 100us
          updatePositions(); //Update encoder, motor, and degree position values
          Serial.print("Top Switch Position: ");
          top = position; //Store top position
          Serial.println(top);

          Serial.print("Home Topside Next...");
          homeSensor.update(); //Process home switch debounce
          motor.rotateAsync(homingSpeed); //Rotate downwards at homing speed
          while(!homeSensor.read()){ //While the homing switch hasn't triggered
            homeSensor.update(); //Update bounce input
          }
          updatePositions();
          hom2=position;
          Serial.print("Homing Top: ");
          Serial.print(hom2);

          motor.rotateAsync(homingSpeed); //re-assert rotation in the downwards direction
          while(homeSensor.read()){//While homing switch is still triggered
            homeSensor.update(); //Update the bounce input
          }
          updatePositions();
          hom3=position;
          Serial.print(", Homing Bottom(a):");
          Serial.println(hom3);
          delayMicroseconds(100);
          endStop.update();
          Serial.println("Bottom Switch Next...");
          motor.rotateAsync(homingSpeed); //re-assert downward rotation at homing speed
          while(!endStop.fell()){ //while the endstop hasn't triggered
            endStop.update(); //Update bounce processor
          }
          motor.stop(); //When triggered, stop motors
          updatePositions();
          bot = position; 
          Serial.print("Bottom Switch: ");
          Serial.println(bot);

          Serial.println("Home Verification/Return Last...");
          motor.rotateAsync(-homingSpeed); //Begin rotation in upwards direction at homing speed
          while(!homeSensor.read()){ //While the homing switch is not active
            homeSensor.update(); //Update bounce processor
          }
          motor.stop(); //When triggered, stop and record encoder value
          updatePositions();
          hom1 =(position+hom3)/2;
          Serial.print("Home Bottom(b)");
          Serial.println(position);
          hom1 = (position+hom3)/2; //Set home range bottom
          hom3 = (hom1+hom2)/2; //Set home range middle

          hardTop = top;
          hardBottom = bot;
          homePosition = hom3;
          homeWidth = abs(hom2-hom1);
          

          Serial.println("Homing Test Complete");
          updatePositions();
          motor.rotateAsync(-homingSpeed/10); //Continue upward rotation at homingSpeed/10
          while(position<hom3){ //Wait until encoder position is at least "home middle"
          updatePositions();
          //Wait
          }
          motor.stop(); //Stop motor
          setMotorHome(); //Set TeensyStep motor position to 0 (Also sets speed to 0)



        //Verify home/encoder matched
        while(!homeSensor.read()){ //While homing sensor is not flagged...
          updatePositions();
          if(endStop.read()){ //Check if endstop flagged
            motor.stop(); // STOP! (We should be rotating towards home!)
            if(position>homePosition){ // Check Position and begin rotating away from stops
              motor.rotateAsync(homingSpeed);
            }else{
              motor.rotateAsync(-homingSpeed);
            }
          }else{ // Endstop not flagged
          if(position>homePosition){ //Rotate towards home
              motor.rotateAsync(homingSpeed);
            }else{
              motor.rotateAsync(-homingSpeed);
            }
          }
          endStop.update();
          homeSensor.update();
        }
        updatePositions();
        if(degrees<-4 || degrees>8){ //did the sensor flag outside the accepted range?
          Serial.println("Encoder/Homing Switch Mismatch! Check Alignment.");
          Serial.println(position);
          motor.overrideSpeed(0.0);
        }else{ //Homing switch and encoder agree, test complete
          Serial.println("Basic Homing Test Complete");
          Serial.println(degrees); //Usually homes at -2
          motor.overrideSpeed(0.0);
        }
                    
    } // end of calibrateSensors
  */
    void RobotAxis::calibrateHomeSensor(){
      endStop.update();
      if(endStop.read()){
        homeSensor.update();
        if(!homeSensor.read()){
          updatePosition();
          motor.rotateAsync(homingSpeed);
          if(position>0){
            motor.overrideSpeed(1.0/10);
          }else{
            motor.overrideSpeed(-1.0/10);
          }
          homeSensor.update();
          while(!homeSensor.read()){
            homeSensor.update();
          }
          updatePosition();
          double hometop = position;
          homeSensor.update();
          while(homeSensor.read()){
            homeSensor.update();
          }
          motor.stop();
          double homebottom = position;
          targetPosition = (int)abs(hometop-homebottom)/2;
        }
      }
      updatePosition();
      
    }
    double RobotAxis::getHomeOffset(){
      return ((homePosition*360.0)/1023)-180;
    }

    double RobotAxis::getPosition(){
     // updatePosition();
      return degrees;
    }

    int RobotAxis::getEncoderPosition(){
    //  updatePosition();
      return position;
    }

    int RobotAxis::getMotorPosition(){
     // updatePosition();
      return stepPosition;
    }

    int RobotAxis::getHome(){
      return homePosition;
    }

    int RobotAxis::getHomeWidth(){
      return homeWidth;
    }

    int RobotAxis::getHardTop(){
      return hardTop;
    }

    int RobotAxis::getHardBottom(){
      return hardBottom;
    }

    bool RobotAxis::isCalibrated(){
      return calibrated;
    }

    bool RobotAxis::isEnabled(){
      return enabled;
    }

    bool RobotAxis :: isFaulted(){
      return fault;
    }

    bool RobotAxis::isMoving(){
      return moving;
    }

    uint8_t RobotAxis::getFault(){
      return faultCode;
    }

    void RobotAxis::setHome(int width){
      updatePosition();
      homePosition = position;
      homeWidth = width;
    }

    void RobotAxis::setMotorHome(){
      //updatePositions();
      motor.setPosition(0);
    }

    void RobotAxis::disable(){
      enabled = false;
      //Serial.println("DisablingAxis");
      motor.overrideSpeed(0.0);
      return;
    }

    void RobotAxis::enable(){
      enabled = true;
    }

    void RobotAxis::setTargetPosition(int target){
        targetPosition = target;
        setpoint = target;
    }

    void RobotAxis::setTargetPosition(int target,int speed){
        targetPosition = target;
        targetSpeed = speed;
    }
    

    void RobotAxis::updatePosition(){
          position = analogRead(encoderPin);
          stepPosition = motor.getPosition();
          degrees = ((position*360.0)/1023)-180;
          input = degrees;
    }
    void RobotAxis::rotate(uint16_t speed,double override){
      motor.rotateAsync(speed);
      motor.overrideSpeed(override); //Scale motor to axis 
    }
    void RobotAxis::tick(){
      updatePosition();
      positionController.compute();
   //   Serial.print(output);
    //  Serial.print("|");
    //  Serial.print(setpoint);
    //  Serial.print("|");
    //  Serial.println(input);

       if(abs(output)>.01){
         rotate(maximumSpeed,-output);
        }
        else{
         motor.overrideSpeed(0.0);

        }
}
}
