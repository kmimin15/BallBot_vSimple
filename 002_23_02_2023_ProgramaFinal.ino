#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

//Motor 1 (PWM divided into two factors) YAW
#define R_IS_yaw 8
#define R_EN_yaw 9
#define R_PWM_yaw 2
#define L_IS_yaw 10
#define L_EN_yaw 11
#define L_PWM_yaw 3

//Motor 2 (PWM divided into two factors)PITCH
#define R_IS_pitch 22
#define R_EN_pitch 24
#define R_PWM_pitch 4
#define L_IS_pitch 26
#define L_EN_pitch 28
#define L_PWM_pitch 5

//Motor 3 (PWM divided into two factors)ROLL
#define R_IS_roll 30
#define R_EN_roll 32
#define R_PWM_roll 6
#define L_IS_roll 34
#define L_EN_roll 36
#define L_PWM_roll 7

float q0;
float q1;
float q2;
float q3;

float k1=.7; //proportional
//float k2=70; //derivative 
//float k3=.001; //integral

int milliOld;
int milliNew;
int dt;

float rollTarget=0; //Desired Value
float rollActual; //Value of the real position 
float rollServoVal=0; //Value sent to the servo to perform
float rollError=0; //diference between Target value and Actual value
float rollErrorOld; //the last time through the loop
//float rollErrorChange;
//float rollErrorSlope; //change of the error over the time
//float rollErrorArea=0;

float pitchTarget=0; //Desired Value
float pitchActual; //Value of the real position 
float pitchServoVal=0; //Value sent to the servo to perform
float pitchError=0; //diference between Target value and Actual value
float pitchErrorOld; //the last time through the loop
//float pitchErrorChange;
//float pitchErrorSlope; //change of the error over the time
//float pitchErrorArea=0;

float yawTarget=0; //Desired Value
float yawActual; //Value of the real position 
float yawServoVal=0; //Value sent to the servo to perform
float yawError=0; //diference between Target value and Actual value
float yawErrorOld; //the last time through the loop
//float yawErrorChange;
//float yawErrorSlope; //change of the error over the time
//float yawErrorArea=0;

/*If the ACTUAL value that I am measuring is greater than the TARGET value, the code must 
subract one degree from it until its the same*/


#define BNO055_SAMPLERATE_DELAY_MS (100) //This indicates the system, how fast it will work per sample rate

Adafruit_BNO055 myIMU = Adafruit_BNO055(); //sensor object

void setup() {
  Serial.begin(115200); //the program needs to be faster because it will show several types of data
  myIMU.begin(); //Turns on the thing on
  delay(1000);
  int8_t temp = myIMU.getTemp(); //store a number from 128 to 127
  //Serial.println(temp);
  myIMU.setExtCrystalUse(true); //says not to use the crysrtal of the chip but on the board

//Declaring all the pins as input or output
  pinMode(R_IS_yaw,OUTPUT);
  pinMode(R_EN_yaw,OUTPUT);
  pinMode(R_PWM_yaw,OUTPUT);
  pinMode(L_IS_yaw,OUTPUT);
  pinMode(L_EN_yaw,OUTPUT);
  pinMode(L_PWM_yaw,OUTPUT);

  pinMode(R_IS_pitch,OUTPUT);
  pinMode(R_EN_pitch,OUTPUT);
  pinMode(R_PWM_pitch,OUTPUT);
  pinMode(L_IS_pitch,OUTPUT);
  pinMode(L_EN_pitch,OUTPUT);
  pinMode(L_PWM_pitch,OUTPUT);

  pinMode(R_IS_roll,OUTPUT);
  pinMode(R_EN_roll,OUTPUT);
  pinMode(R_PWM_roll,OUTPUT);
  pinMode(L_IS_roll,OUTPUT);
  pinMode(L_EN_roll,OUTPUT);
  pinMode(L_PWM_roll,OUTPUT);

//Original Way of begin working
  digitalWrite(R_IS_yaw,LOW);
  digitalWrite(R_EN_yaw,HIGH);
  digitalWrite(L_IS_yaw,LOW);
  digitalWrite(L_EN_yaw,HIGH);

  digitalWrite(R_IS_pitch,LOW);
  digitalWrite(R_EN_pitch,HIGH);
  digitalWrite(L_IS_pitch,LOW);
  digitalWrite(L_EN_pitch,HIGH);

  digitalWrite(R_IS_roll,LOW);
  digitalWrite(R_EN_roll,HIGH);
  digitalWrite(L_IS_roll,LOW);
  digitalWrite(L_EN_roll,HIGH);


  milliNew=millis();
}

void loop() {
  uint8_t system, gyro, accel, mg = 0; //efficient way to store data
  myIMU.getCalibration(&system, &gyro, &accel, &mg);
  imu::Quaternion quat=myIMU.getQuat(); //Ask the BNO055 to get the quaternion vector for u

  q0=quat.w();
  q1=quat.x();
  q2=quat.y();
  q3=quat.z();

  rollActual=-atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2)); //Calculation of the actual position of the roll on radiants
  pitchActual=asin(2*(q0*q2-q3*q1));
  yawActual=-atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))-3.141592654/2;

  rollActual=rollActual/(2*3.141592654)*360; //Calculation of the actual position of the roll on degrees
  pitchActual=pitchActual/(2*3.141592654)*360;
  yawActual=(yawActual/(2*3.141592654))*360;
//
milliOld=milliNew;
milliNew=millis();
dt=milliNew-milliOld;

//*********************PID roll
rollErrorOld=rollError;
rollError=rollTarget-rollActual;  //p
//rollErrorChange=rollError-rollErrorOld; 
//rollErrorSlope=rollErrorChange/dt; //d
//rollErrorArea=rollErrorArea+rollError*dt; //i

rollServoVal=k1*rollError; //+k2*rollErrorSlope+k3*rollErrorArea;

//Determine whether it will go right or left

if(rollActual>10 && rollActual<=87){ //it will start correccting the value depending on the direction
//  digitalWrite(in1yaw,HIGH); //these are not longer needed when using BRS7960
//  digitalWrite(in2yaw,LOW);
  
  analogWrite(R_PWM_yaw,0); //speed and direction
  analogWrite(L_PWM_yaw,abs(rollServoVal)+15); //one one cero to indicate the direction in which is going
  analogWrite(R_PWM_pitch,abs(rollServoVal)+15);
  analogWrite(L_PWM_pitch,0); 
  analogWrite(R_PWM_roll,0); 
  analogWrite(L_PWM_roll,0);
}
if(rollActual<-10 && rollActual>=-87){
//  digitalWrite(in1yaw,LOW);
//  digitalWrite(in2yaw,HIGH);

  analogWrite(R_PWM_yaw,abs(rollServoVal)+15); 
  analogWrite(L_PWM_yaw,0); 
  analogWrite(R_PWM_pitch,0);
  analogWrite(L_PWM_pitch,abs(rollServoVal)+15); 
  analogWrite(R_PWM_roll,0); 
  analogWrite(L_PWM_roll,0);
}
else{
  analogWrite(R_PWM_yaw,rollServoVal); 
  analogWrite(L_PWM_yaw,0); 
  analogWrite(R_PWM_pitch,rollServoVal);
  analogWrite(L_PWM_pitch,0); 
  analogWrite(R_PWM_roll,rollServoVal); 
  analogWrite(L_PWM_roll,0);
}



//**********************PID pitch
pitchErrorOld=pitchError;
pitchError=pitchTarget-pitchActual;
//pitchErrorChange=pitchError-pitchErrorOld;
//pitchErrorSlope=pitchErrorChange/dt;
//pitchErrorArea=pitchErrorArea+pitchError*dt;

pitchServoVal=k1*pitchError; //+k2*pitchErrorSlope+k3*pitchErrorArea;
//Determine whether it will go right or left

if(pitchActual>10 && pitchActual<=87){ //it will start correccting the value depending on the direction
//  digitalWrite(in1yaw,HIGH); //these are not longer needed when using BRS7960
//  digitalWrite(in2yaw,LOW);
  
  analogWrite(R_PWM_yaw,abs(pitchServoVal)+15); //speed and direction
  analogWrite(L_PWM_yaw,0); //one one cero to indicate the direction in which is going
  analogWrite(R_PWM_pitch,0);
  analogWrite(L_PWM_pitch,0); 
  analogWrite(R_PWM_roll,0); 
  analogWrite(L_PWM_roll,abs(pitchServoVal)+15);
}
if(pitchActual<-10 && pitchActual>=-87){
//  digitalWrite(in1yaw,LOW);
//  digitalWrite(in2yaw,HIGH);

  analogWrite(R_PWM_yaw,0); 
  analogWrite(L_PWM_yaw,abs(pitchServoVal)+15); 
  analogWrite(R_PWM_pitch,0);
  analogWrite(L_PWM_pitch,0); 
  analogWrite(R_PWM_roll,abs(pitchServoVal)+15); 
  analogWrite(L_PWM_roll,0);
}
else{
  analogWrite(R_PWM_yaw,pitchServoVal); 
  analogWrite(L_PWM_yaw,0); 
  analogWrite(R_PWM_pitch,pitchServoVal);
  analogWrite(L_PWM_pitch,0); 
  analogWrite(R_PWM_roll,pitchServoVal); 
  analogWrite(L_PWM_roll,0);
}

//********************PID yaw
yawErrorOld=yawError;
yawError=yawTarget-yawActual;
//yawErrorChange=yawError-yawErrorOld;
//yawErrorSlope=yawErrorChange/dt;
//yawErrorArea=yawErrorArea+yawError*dt;

yawServoVal=k1*yawError;//+k2*yawErrorSlope+k3*yawErrorArea;

if(yawActual>10 && yawActual<=87){ //it will start correccting the value depending on the direction
//  digitalWrite(in1yaw,HIGH); //these are not longer needed when using BRS7960
//  digitalWrite(in2yaw,LOW);
  
  analogWrite(R_PWM_yaw,abs(yawServoVal)+15); //speed and direction
  analogWrite(L_PWM_yaw,0); //one one cero to indicate the direction in which is going
  analogWrite(R_PWM_pitch,abs(yawServoVal)+15);
  analogWrite(L_PWM_pitch,0); 
  analogWrite(R_PWM_roll,abs(yawServoVal)+15); 
  analogWrite(L_PWM_roll,0);
}
if(yawActual<-10 && yawActual>=-87){

  analogWrite(R_PWM_yaw,0); 
  analogWrite(L_PWM_yaw,abs(yawServoVal)+15); 
  analogWrite(R_PWM_pitch,0);
  analogWrite(L_PWM_pitch,abs(yawServoVal)+15); 
  analogWrite(R_PWM_roll,0); 
  analogWrite(L_PWM_roll,abs(yawServoVal)+15);
}
else{
  analogWrite(R_PWM_yaw,yawServoVal); 
  analogWrite(L_PWM_yaw,0); 
  analogWrite(R_PWM_pitch,yawServoVal);
  analogWrite(L_PWM_pitch,0); 
  analogWrite(R_PWM_roll,yawServoVal); 
  analogWrite(L_PWM_roll,0);
}

//printing stuff
  Serial.print(quat.w()); 
  Serial.print(",");
  Serial.print(quat.x()); 
  Serial.print(",");
  Serial.print(quat.y()); 
  Serial.print(",");
  Serial.print(quat.z());
  Serial.print(",   ");
  Serial.print(pitchActual); //y
  Serial.print(",");
  Serial.print(yawActual); //z
  Serial.print(",");
  Serial.print(rollActual); //x
  Serial.print(",   ");
  Serial.print(accel);
  Serial.print(",");
  Serial.print(gyro);
  Serial.print(",");
  Serial.print(mg);
  Serial.print(",");
  Serial.println(system);


  delay(BNO055_SAMPLERATE_DELAY_MS);
}
