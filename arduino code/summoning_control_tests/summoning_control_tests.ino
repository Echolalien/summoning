#include <AccelStepper.h>

//pin declaration
const int rL = 2; //roll anticlockwise/left pin
const int rR = 3; //roll clockwise/right pin
const int ySt = 4; //endstop pin for yaw
const int tSt = 3; //endstop for tip roll
const int yL = 5; //yaw left pin
const int yR = 6; //yaw right pin
const int killSwitch = 8; //shared enable pins for motors
const int rS1 = A0; //endstop pin for roll 1
const int yK = A1; //yaw position knob pin
const int rK = A2; //roll position knob pin
const int m1s = 10; //motor driver 1 step pin
const int m1d = 9; //motor driver 1 direction pin
const int m2s = 12; //motor driver 2 step pin
const int m2d = 11; //motor driver 2 direction pin

//states
bool yawStop = false; //is yaw endstop pressed
bool tipStop = false; //is the face vertical
int yawKnob = 0; //yaw knob position
int rollKnob = 0; //roll knob position
bool yawLeft = 0;
bool yawRight = 0;
bool rollLeft = 0;
bool rollRight = 0;
bool abrt = 0; //abort, pull enable high on both motors, kill processes
int calibrationStage = 0; //if calibration needs several steps, increment this counter
bool calibrated = 0; //robot is calibrated, ready to use
bool yawOn = 0; //yawing or (2ndary) rolling? can't do both at 1ce
bool rollOn = 0; //as above, set roll position to motors
int yawSpeed = 2000;
int rollSpeed = 2000;

//articulation positions
int yawPos = 0; //actual yaw position
int yawDeg = 0; //yaw position in angles
int rollPos = 0; //actual roll position
int yawTarget = 1;
int rollTarget = 1;

//calibration variables
int yawMax = 0;
int rollMax = 0;
int calibIncr = 3500;
int calibTarget = 0; //point to move to
bool rollHomed = 0; //has roll been homed yet?

//define motor drivers, dir and step
AccelStepper motor1(AccelStepper::DRIVER, m1d, m1s);
AccelStepper motor2(AccelStepper::DRIVER, m2d, m2s);

void setup() {

  //pin modes, pull up increment control pins
  for (int i = 2; i < 9; i++) {
    pinMode(i, INPUT_PULLUP);
  }
  pinMode(killSwitch, OUTPUT);

  Serial.begin(9600);
  while (!Serial);
  Serial.println();
  Serial.println("Serial open, beginning startup");

  //set motor speeds
  motor1.setMaxSpeed(3000);
  motor1.setAcceleration(1000);
  motor2.setMaxSpeed(3000);
  motor2.setAcceleration(1000);

  //release motors, check you're not already at the yaw endstop, if so, abort
  digitalWrite(killSwitch, HIGH);
  Serial.println("Checking yaw");
  yawStop = ! digitalRead(ySt);
  if (yawStop == HIGH) {
    abrt = 1;
    Serial.println("Endstop already engaged, startup aborted!");
  }
  else {
    Serial.println("Yaw is go");
    Serial.println("Startup complete, Summoning online, engaging motors in 5");
    delay(5000);
    digitalWrite(killSwitch, LOW);
  }
}

void loop() {

  sensorPull(); //get all sensor data and dump into variables
  

  //check that abort hasn't been activated
  if (abrt == 0) {
    //check if calibration is needed
    if (calibrated == 1) {

      //check yaw axis yknow, exists
      if (yawMax > 0) {

        //core runtime code
        functionCheck(); //check for errors in sensor data, positions, get mode
        printOut();
        goToPos(); //move to knob positions
        Serial.println();
      }
      else {

        abrt = 1;
        Serial.println("Yaw axis not available, aborting all processes");
      }
    }
    else {
      calibrate();
    }
  }

  //abort procedure: disengage motors, error light on, 5 sec pause to read console
  else {
    digitalWrite(killSwitch, HIGH);
    digitalWrite(13, HIGH);
    Serial.println("Summoning offline");
    delay(5000);
  }
}

void calibrate() {

  //STAGE 0 yaw calibrates by moving to endstop, releasing it then saving the max position
  if(calibrationStage == 0){

    //moves to endstop
    if (yawTarget != 0) {
      if(yawStop != 1){
        if (motor1.distanceToGo() == 0 && motor2.distanceToGo() == 0) {
          yawTarget += calibIncr;
          motor1.moveTo(yawTarget);
          motor2.moveTo(yawTarget);
          Serial.print("\t Calibrating yaw...");
          Serial.println("\t Yawing to endstop");
        }
        yawPos = yawTarget - motor2.distanceToGo();
        motor1.run();
        motor2.run();
      }
      
      //endstop reached, retreat from endstop
      else{
        Serial.print("Endstop reached at ");
        Serial.print(yawPos);
        Serial.println(", retreating from endstop...");
        yawTarget = 0;
        motor1.setCurrentPosition(yawPos);
        motor2.setCurrentPosition(yawPos);
        motor1.moveTo(yawTarget);
        motor2.moveTo(yawTarget);
      }
    }

    //retreating
    else if(yawTarget != yawPos){
      if(yawMax == 0 && yawStop != 1){
          yawMax = yawPos;
          Serial.print("Endstop released at ");
          Serial.print(yawMax);
          Serial.println(", setting maximum yaw.");
          Serial.println("Returning to home");
      }
      yawPos = yawTarget - motor2.distanceToGo();
      motor1.run();
      motor2.run();
    }
    else{
      calibrationStage++;
      Serial.println("Arrived at home, yaw axis calibrated successfully!");
      motor1.setCurrentPosition(0);
      motor2.setCurrentPosition(0);
    }
  }

  //STAGE 1 tip rolls until it tilts the series switches, marking its home point
  else if(calibrationStage == 1){

    //moves to endstop
      if(tipStop != 1){
        if (motor1.distanceToGo() == 0 && motor2.distanceToGo() == 0) {
          rollTarget -= calibIncr;
          motor1.moveTo(0-rollTarget);
          motor2.moveTo(rollTarget);
          Serial.print("\t Preparing to calibrate tip roll...");
          Serial.print("\t Rolling to find home for ");
          Serial.println(calibIncr);
        }
        motor1.run();
        motor2.run();
      }
      
      //endstop reached, set this as 0 and we can now actually calibrate endstop
      else{
        Serial.println("Endstop reached, setting this as home");
        calibrationStage++;
        rollPos = 0;
        motor1.setCurrentPosition(0);
        motor2.setCurrentPosition(0);
      }
    }

    //retreating
    else if (calibrationStage == 2){

      //if it hasn't yet retreated off the stop
      if(rollHomed == 0){
      
        //if the endstop is on
        if(tipStop == 1){
          //move opposite direction to endstop until it releases
          if(motor1.distanceToGo() == 0 && motor2.distanceToGo() == 0){
            Serial.print("Retreating from endstop for ");
            Serial.print(calibIncr);
            rollTarget += calibIncr;
            motor1.moveTo(0-rollTarget);
            motor2.moveTo(rollTarget);
          }
          motor1.run();
          motor2.run();
          rollPos = rollTarget - motor2.distanceToGo();
        }

        //when it releases
        else{
          rollHomed = 1;
          Serial.println("Endstop released");
        }
      }

      //once it's retreated from endstop
      else{

        //if it hasn't reached endstop at other end
        if(tipStop =! 1){
          
          //move opposite direction to endstop
          if(motor1.distanceToGo() == 0 && motor2.distanceToGo() == 0){
            Serial.print("Retreating from endstop for ");
            Serial.print(calibIncr);
            rollTarget += calibIncr;
            motor1.moveTo(0-rollTarget);
            motor2.moveTo(rollTarget);
          }
          motor1.run();
          motor2.run();
          rollPos = rollTarget - motor2.distanceToGo();
        }

        //when it hits endstop at other end
        else{
          rollPos = rollTarget - motor2.distanceToGo();
          rollMax = rollTarget;
          Serial.print("Hit endstop at ");
          Serial.print(rollPos);
          Serial.println(", setting this as maxiumum roll.");
          Serial.println("Tip roll axis calibrated successfully!");
          calibrationStage++;
          motor1.setCurrentPosition(0);
          motor2.setCurrentPosition(0);
        }
      }
    }
  

  
  else {
    //calibration complete
    calibrated = 1;
    Serial.println("Calibration complete! Operational in 5");
    delay(5000);
  }
}

void sensorPull() {
  //takes all sensor data and dumps into variables, prints to serial

  yawStop = ! digitalRead(ySt);
  tipStop = ! digitalRead(tSt);
  yawLeft = ! digitalRead(yL);
  yawRight = ! digitalRead(yR);
  rollLeft = ! digitalRead(rL);
  rollRight = ! digitalRead(rR);
  yawKnob = map(analogRead(yK), 0, 1023, 0, yawMax);
  rollKnob = map(analogRead(rK), 0, 1023, 0, 360);
  yawDeg = map(yawPos, 0, yawMax, 0, 180);
}

void printOut() {
  //publishes serial data
  
  Serial.print("YawKnob position: ");
  Serial.print(yawKnob);
  Serial.print("\t RollKnob position: ");
  Serial.print(rollKnob);
  Serial.print("\t Yaw position: ");
  Serial.print(yawDeg);
}

void functionCheck() {
  //tests that sensors and positional data match up

    if(yawStop == true){
      Serial.print("Hit yawStop unexpectedly! Clipping yaw range to ");
      Serial.print(yawPos-10);
      Serial.println(" and restarting in 5");
      //current yaw position is new maximum
      yawMax = yawPos-10;
      delay(5000);
    }

  //dictate control mode based on whether knobs are at 0
//  if (rollKnob == 0 && yawKnob == 0) {
//    fineMode = 0;
//  }
//  else {
//    fineMode = 1;
//  }
}

void goToPos() {
  //move to position of knobs

  //check if need roll
  if(rollOn == 0){
    //roll
    motor1.setCurrentPosition(rollPos);
    motor2.setCurrentPosition(rollPos);
    rollOn = 1;
    motor1.setPinsInverted(true, false, false);
  }
  else if (rollKnob != rollPos) {
    Serial.print("\t Rolling to ");
    Serial.print(rollKnob);
    motor1.moveTo(0-rollKnob);
    motor2.moveTo(rollKnob);
    motor1.setSpeed(rollSpeed);
    motor2.setSpeed(rollSpeed);
    motor1.runSpeedToPosition();
    motor2.runSpeedToPosition();
    Serial.print("\t Distance remaining: ");
    Serial.print(motor2.distanceToGo());
    rollPos = rollKnob - motor2.distanceToGo();
  }
  //check if need yaw
  else if(yawOn == 0){
    motor1.setCurrentPosition(yawPos);
    motor2.setCurrentPosition(yawPos);
    yawOn = 1;
    motor1.setPinsInverted(false, false, false);
  }
  else if (yawKnob != yawPos) {
    Serial.print("\t Yawing to ");
    Serial.print(yawKnob);
    //int yawDist = yawKnob-yawPos;
    motor1.moveTo(yawKnob);
    motor2.moveTo(yawKnob);
    motor1.setSpeed(yawSpeed);
    motor2.setSpeed(yawSpeed);
    motor1.runSpeedToPosition();
    motor2.runSpeedToPosition();
    Serial.print("\t Distance remaining: ");
    Serial.print(motor2.distanceToGo());
  }
  else{
    Serial.print("In position");
    rollOn = 0;
    yawOn = 0;
  }
}
