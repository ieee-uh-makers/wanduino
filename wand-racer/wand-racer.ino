//#define JOYSTICK_DEBUG
//#define VECTOR_DEBUG

// Inbetween 0.0 and 1.0, values less than the norm of both motors drive mean we don't move
#define MIN_MOTOR_ACTUATION 0.3
#define MAX 1022 

#define CORRECTION_COEFFECIENT_LEFT 1.0
#define CORRECTION_COEFFECIENT_RIGHT 1.1

// IO
const int joyctrl[] = {A0, A1, 2};
const int LftMtr[] = {3, 5}; 
const int RgtMtr[] = {6, 9}; 

// Control input values
int ctrlVal[]= {0, 0, 0};

// Calibrated center of joystick axis
int origin[] = {500, 500};

// Steering for differential drive
double steeringVector[]= {0., 0.};
double lastSteeringVector[]= {0., 0.};

// LR motor drive for differential drive
double drive[] = {0., 0.};

// Joystick minimum movement threshold on an axis
int threshold= 50; 

// Time each loop takes in ms
int dT = 0;

// Movement modes of the robot
enum Mode{
  NORMAL, ROTATION_LEFT, ROTATION_RIGHT, WIGGLE, DRUNK, WAND
};

// Planar rotation Angle/Time differential (rad / miliseconds)
// Rotate controls 2*pi radians every 5 seconds
double drunk_dTheta_dT = 2*PI / (4*1000);
double rotate_dTheta_dT = 2*PI / (20*1000);

double runningAngle = 0;

char lastCommand = 'O';

Mode mode = WAND;

#include <SoftwareSerial.h>
SoftwareSerial wand(11, 12); // RX, TX

void setup() {
  for (int i = 0; i < 3; i++) {
    pinMode(joyctrl[i], INPUT) ;  
  }
  for (int i = 0; i < 2; i++) {
    pinMode(LftMtr[i], OUTPUT) ;
    pinMode(RgtMtr[i], OUTPUT) ;
  }
  Serial.begin(9600);
  wand.begin(115200);

  steeringVector[0] =  0.0;
  steeringVector[1] =  0.0;
 
  calibrate();
}

void loop() {
  
  int timeStart, timeEnd;

  timeStart = millis();
    
  if (mode != WAND){
    joyStick();
    steering();
  }

  switch(mode){
    case NORMAL:
      driveNormal();
      break;
    case ROTATION_RIGHT:
      driveRotation(false);
      break;
    case ROTATION_LEFT:
      driveRotation(true);
      break;
    case DRUNK:
      driveDrunk();
      break;
    case WAND:
      driveWand();
      break;
  }

  // ~100hz control loop
  delay(10);

  timeEnd = millis();
   
  dT = timeEnd - timeStart;
  
  serialMonitor();

}


double dabs(double v){
  if(v < 0)
    return v*-1;

  return v;
}

void differentialDrive(){

  drive[0] = 0;
  drive[1] = 0;

  double translationalPart, rotationalPart;

  if(steeringVector[0] == 0. && steeringVector[1] == 0.){
    translationalPart = 0.;
    rotationalPart = 0.;
  }else if(dabs(steeringVector[1]) >= dabs(steeringVector[0])){
    
    // rotational part larger than or equal to translational
    rotationalPart = dabs(steeringVector[1]) / (dabs(steeringVector[0]) + dabs(steeringVector[1]));
    translationalPart = 1 - rotationalPart;

  }else{
    // translational part larger than rotational
    translationalPart =  dabs(steeringVector[0]) / (dabs(steeringVector[0]) + dabs(steeringVector[1]));
    rotationalPart = 1 - translationalPart;
    
  }

  // Left
  drive[0] += translationalPart * steeringVector[0];
  drive[0] += rotationalPart * steeringVector[1];

  // Right
  drive[1] += translationalPart * steeringVector[0];  
  drive[1] += rotationalPart * -steeringVector[1];

  // Don't just barely try to move and make some annoying sound
  if(sqrt(pow(drive[0], 2) + pow(drive[1], 2)) <= MIN_MOTOR_ACTUATION){
    analogWrite(LftMtr[0], 0);
    analogWrite(LftMtr[1], 0);
    analogWrite(RgtMtr[0], 0);
    analogWrite(RgtMtr[1], 0);
    return;
  }

  int drive_left[] = {0, 0};
  int drive_right[] = {0, 0};
  
  if(drive[0] >= 0){
      drive_left[0] = (int) dabs(CORRECTION_COEFFECIENT_LEFT * drive[0] * 255);
      drive_left[1] = 0;
  }else{
      drive_left[0] = 0;
      drive_left[1] = (int) dabs(CORRECTION_COEFFECIENT_LEFT * drive[0] * 255);
  }

  if(drive[1] >= 0){
      drive_right[0] =(int) dabs(CORRECTION_COEFFECIENT_RIGHT * drive[1] * 255);
      drive_right[1] = 0;
  }else{
      drive_right[0] = 0;
      drive_right[1] = (int) dabs(CORRECTION_COEFFECIENT_RIGHT * drive[1] * 255);
  }

  analogWrite(LftMtr[0], drive_left[0]);
  analogWrite(LftMtr[1], drive_left[1]);
  analogWrite(RgtMtr[0], drive_right[0]);
  analogWrite(RgtMtr[1], drive_right[1]);

}

#define VEL 0.325
void driveWand(){
    
  if (wand.available() > 0){
    char cmd  = wand.read();

    if (cmd == 'L' || cmd =='R' || cmd == 'U' || cmd == 'D' || cmd == 'O')
      lastCommand = cmd;
        
    switch(lastCommand){
      case 'D':
        steeringVector[0] =  0.0;
        steeringVector[1] =  -VEL;
        break;
      case 'U':
        steeringVector[0] =  0.0;
        steeringVector[1] =  VEL;
        break;
      case 'L':
        steeringVector[0] =  VEL;
        steeringVector[1] =  0.0;
        break;
      case 'R':
        steeringVector[0] = -VEL;
        steeringVector[1] =  0.0;
        break;
      case 'O':
        steeringVector[0] =  0.0;
        steeringVector[1] =  0.0;
        break;
      default:
        steeringVector[0] =  0.0;
        steeringVector[1] =  0.0;
        break;
  }
    Serial.print("CMD: ");
    Serial.print(lastCommand);
    Serial.print("\n");
  }
  
  differentialDrive();
}

void driveNormal(){
  differentialDrive();
}

void driveRotation(boolean invert){
  // The first control loop we don't have a deltaT so just return for now
  if(dT == 0)
    return;
  
  if(!invert){
    runningAngle += (double)(dT) * rotate_dTheta_dT;
  }else{
    runningAngle -= (double)(dT) * rotate_dTheta_dT;
  }

  // Don't just barely try to move and make some annoying sound
  if(sqrt(pow(steeringVector[0], 2) + pow(steeringVector[1], 2)) <= MIN_MOTOR_ACTUATION){
    differentialDrive();
    return;
  }

  double norm = sqrt(pow(steeringVector[0], 2) + pow(steeringVector[1], 2));
  double oldAngle = atan2(steeringVector[1], steeringVector[0]);

  float newAngle = oldAngle + runningAngle;

  // Update the ctrlVal with the rotation
  steeringVector[0] = norm*cos(newAngle);
  steeringVector[1] = norm*sin(newAngle);

  differentialDrive();
  
}

void driveDrunk(){
  // The first control loop we don't have a deltaT so just return for now
  if(dT == 0)
    return;

  runningAngle += dT * drunk_dTheta_dT;

  // Don't just barely try to move and make some annoying sound
  if(sqrt(pow(steeringVector[0], 2) + pow(steeringVector[1], 2)) <= MIN_MOTOR_ACTUATION){
    differentialDrive();
    return;
  }

  double norm = sqrt(pow(steeringVector[0], 2) + pow(steeringVector[1], 2));
  double oldAngle = atan2(steeringVector[1], steeringVector[0]);

  float newAngle = oldAngle + (sin(runningAngle) * PI/4.0);

  steeringVector[0] = cos(newAngle)*norm;
  steeringVector[1] = sin(newAngle)*norm;

  differentialDrive();
  
}

void calibrate() {
  delay(1000);

  for (int i = 0; i < 2; i++) {
    origin[i] = analogRead(joyctrl[i]);
  }

  Serial.print("Calibration: [");
  Serial.print(origin[0]);
  Serial.print(", ");
  Serial.print(origin[1]);
  Serial.print("]\n");
}

void joyStick (){
  for (int i = 0; i < 2; i++) {
    ctrlVal[i] = analogRead(joyctrl[i]);
  }
  ctrlVal[2] = digitalRead(joyctrl[2]);
}

void serialMonitor () { // Formatted output

#ifdef JOYSTICK_DEBUG
  Serial.print("[X: ");
  Serial.print(ctrlVal[0]);
  Serial.print(", Y: ");
  Serial.print(ctrlVal[1]);  
  Serial.print(", Btn: ");
  Serial.print(ctrlVal[2]); 
  Serial.println("] ");  
#endif

#ifdef VECTOR_DEBUG
  Serial.print("steeringVector: [");
  Serial.print(steeringVector[0]);
  Serial.print(", ");
  Serial.print(steeringVector[1]);
  Serial.print("]\n");
#endif

}

void steering() {

  steeringVector[0] = ((double)(ctrlVal[0] - (double)MAX/2) / (double)(MAX/2));
  steeringVector[1] = -((double)(ctrlVal[1] - (double)MAX/2) / (double)(MAX/2));
  
}
