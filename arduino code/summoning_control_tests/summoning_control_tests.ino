#include <AccelStepper.h>

//pin declaration
const int rL = 2; //roll anticlockwise/left pin
const int rR = 3; //roll clockwise/right pin
const int ySt = 4; //endstop pin for yaw
const int yL = 5; //yaw left pin
const int yR = 6; //yaw right pin
const int killSwitch = 8; //shared enable pins for motors
const int yK = A0; //yaw position knob pin
const int rK = A1; //roll position knob pin
const int m1s = 10; //motor driver 1 step pin
const int m1d = 9; //motor driver 1 direction pin
const int m2s = 12; //motor driver 2 step pin
const int m2d = 11; //motor driver 2 direction pin

//states
bool yawStop = false; //is yaw endstop pressed
int yawKnob = 0; //yaw knob position
int rollKnob = 0; //roll knob position
bool yawLeft = 0;
bool yawRight = 0;
bool rollLeft = 0;
bool rollRight = 0;
bool abrt = 0; //abort, pull enable high on both motors, kill processes
bool calibrated = 0; //robot is calibrated, ready to use
bool yawMode = 0; //yawing or (2ndary) rolling? can't do both at 1ce
int yawSpeed = 2000;

//articulation positions
int yawPos = 0; //actual yaw position
int yawDeg = 0; //yaw position in angles
int rollPos = 0; //actual roll position
int yawTarget = 0;

//calibration variables
int yawMax = 0;
int rollMax = 0;
int calibIncr = 10000;
int calibTarget = 0; //point to move to

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
        goToPos(); //move to knob positions
       
      }
      else {

        abrt = 1;
        Serial.println("Yaw axis not available, aborting all processes");
      }
    }
    else {
      calibrate();
    }
    Serial.println();
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

  //yaw calibrates by moving to endstop
  if (yawStop != 1) {
    Serial.print("\t Calibrating yaw...");
    Serial.print("\t Yawing to endstop");
    if (motor1.distanceToGo() == 0 && motor2.distanceToGo() == 0) {
      calibTarget += calibIncr;
      motor1.moveTo(calibTarget);
      motor2.moveTo(calibTarget);
    }
    yawPos = calibTarget - motor2.distanceToGo();
    motor1.run();
    motor2.run();
  }
  else {
    //calibration complete
    calibrated = 1;
    Serial.println();
    Serial.print("Yaw calibrated!");
    yawMax = yawPos;
    Serial.print("\t Yaw steps: ");
    Serial.print(yawMax);
    Serial.println();
    Serial.println("Calibration complete! Operational in 5");
    delay(5000);
  }
}

void sensorPull() {
  //takes all sensor data and dumps into variables, prints to serial

  yawStop = ! digitalRead(ySt);
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

    if(yawStop == true && yawMax != yawPos+(motor1.distanceToGo()+motor2.distanceToGo())/2){
      Serial.print("Hit yawStop unexpectedly! Clipping yaw range to ");
      Serial.print(yawPos);
      Serial.print(" and restarting in 5");
      //current yaw position is new maximum
      yawMax = yawPos;
      delay(5000);
    }
  
    if(yawStop == false && yawMax == yawPos && calibrated == 1){
      Serial.println("Reached max yaw without hitting endStop, recalibrating...");
  
      //reset and uncalibrate eventually, abort while no reset functionality
      abrt=1;
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
  if (rollKnob != rollPos) {
    //roll
    motor1.setCurrentPosition(rollPos);
    motor2.setCurrentPosition(rollPos);
  }
  //check if need yaw
  else if(yawMode == 0){
    motor1.setCurrentPosition(yawPos);
    motor2.setCurrentPosition(yawPos);
    yawMode = 1;
  }
  else if (yawKnob != yawPos) {
    Serial.print("\t Yawing to ");
    Serial.print(yawKnob);
    int yawDist = yawKnob-yawPos;
    motor1.moveTo(yawKnob);
    motor2.moveTo(yawKnob);
    motor1.setSpeed(yawSpeed);
    motor2.setSpeed(yawSpeed);
    motor1.runSpeedToPosition();
    motor2.runSpeedToPosition();
    Serial.print("\t Distance remaining: ");
    Serial.print(motor2.distanceToGo());
    yawPos = yawKnob - motor2.distanceToGo();
  }
}
