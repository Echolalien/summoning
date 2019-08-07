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
bool abrt = 0; //abort, pull enable high on both motors, kill processes
bool calibrated = 0; //robot is calibrated, ready to use


//articulation positions
int yawPos = 0; //actual yaw position
int rollPos = 0; //actual roll position
int m1Pos = 0; //position for motor 1
int m2Pos = 0; //position for motor 2

//calibration variables
int yawMax = 0;
int rollMax = 0;
int calibIncr = 1000;
int calibTarget = 0; //point to move to

//define motor drivers, dir and step
AccelStepper motor1(AccelStepper::DRIVER, m1d, m1s);
AccelStepper motor2(AccelStepper::DRIVER, m2d, m2s);

void setup() {

  //pin modes, pull up increment control pins
  for(int i = 2; i<9; i++){
    pinMode(i, INPUT_PULLUP);
  }
  pinMode(killSwitch, OUTPUT);

  Serial.begin(9600);
  while(!Serial);
  Serial.println("Serial open, beginning startup");

  //set motor speeds
  motor1.setMaxSpeed(3000);
  motor1.setAcceleration(1000);
  motor2.setMaxSpeed(3000);
  motor2.setAcceleration(1000);

  //release motors, check you're not already at the yaw endstop, if so, abort
  digitalWrite(killSwitch, HIGH);
  Serial.println("Checking yaw");
  yawStop =! digitalRead(ySt);
  if(yawStop == HIGH){
    abrt = 1;
    Serial.println("Endstop already engaged, startup aborted!");
  }
  else{
    Serial.println("Yaw is go");
    Serial.println("Startup complete, Summoning online, engaging motors in 5");
    delay(5000);
    digitalWrite(killSwitch, LOW);
  }
}

void loop() {

  if(abrt == 0){

    //endstop tests
    yawStop =! digitalRead(ySt);

    if(calibrated==0){
      calibrate();
    }
    else{
      if(yawMax == 0){
        abrt = 1;
        Serial.println("Yaw axis not available, aborting all processes");
      }
      else{
        //core runtime code
      }
    }
    
  }
  else{
    digitalWrite(killSwitch, HIGH);
    digitalWrite(13, HIGH);
  }
}

void calibrate() {
  
  //yaw calibrates by moving to endstop
  if(yawStop != 1){
    Serial.print("Calibrating yaw...\t");
    Serial.print("Yawing to endstop \t");
    if(motor1.distanceToGo()==0 && motor2.distanceToGo()==0){
      calibTarget += calibIncr;
      motor1.moveTo(calibTarget);
      motor2.moveTo(calibTarget);
    }
    yawPos = calibTarget-motor1.distanceToGo();
    Serial.print("Yaw position: ");
    Serial.print(yawPos);
    Serial.print("\t Motor1 position: ");
    Serial.print(m1Pos);
    Serial.print("\t Motor2 position: ");
    Serial.print(m2Pos);
    Serial.print("\t Motor1 distance to go: ");
    Serial.print(motor1.distanceToGo());
    Serial.print("\t Motor2 distance to go: ");
    Serial.print(motor2.distanceToGo());
    Serial.println();
    motor1.run();
    motor2.run();
  }
  else{
    //calibration complete
    calibrated = 1;
    Serial.print("Yaw calibrated!");
    yawMax = yawPos;
    Serial.print("\t Yaw max: ");
    Serial.print(yawMax);
    Serial.println();
    Serial.println("Calibration complete!");
  }
}
