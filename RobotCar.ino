#define speedPinR 9
#define RightMotorDirPin1  12   
#define RightMotorDirPin2  11
#define speedPinL 6
#define LeftMotorDirPin1  7
#define LeftMotorDirPin2  8

#define buttonPin 3

#include "Wire.h"
#include <MPU6050_light.h>

#include "OptimalPath.h"
#include "Point.h"

MPU6050 mpu(Wire);

void initPins()
{
  pinMode(RightMotorDirPin1, OUTPUT); 
  pinMode(RightMotorDirPin2, OUTPUT); 
  pinMode(speedPinL, OUTPUT);  
  pinMode(LeftMotorDirPin1, OUTPUT);
  pinMode(LeftMotorDirPin2, OUTPUT); 
  pinMode(speedPinR, OUTPUT); 
  stop();
}

void forwards()
{
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2,LOW);
  digitalWrite(LeftMotorDirPin1,HIGH);
  digitalWrite(LeftMotorDirPin2,LOW);
  analogWrite(speedPinL, 120);
  analogWrite(speedPinR, 100);
}

void reverse()
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2,HIGH);
  digitalWrite(LeftMotorDirPin1,LOW);
  digitalWrite(LeftMotorDirPin2,HIGH);
  analogWrite(speedPinL, 120);
  analogWrite(speedPinR, 100);
}

void stop()
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2,LOW);
  digitalWrite(LeftMotorDirPin1,LOW);
  digitalWrite(LeftMotorDirPin2,LOW);
}

void turnLeft(byte speed = 140)
{
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2,LOW);
  digitalWrite(LeftMotorDirPin1,LOW);
  digitalWrite(LeftMotorDirPin2,HIGH);
  analogWrite(speedPinL,speed);
  analogWrite(speedPinR,speed);
}
void turnRight(byte speed = 140)
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2,HIGH);
  digitalWrite(LeftMotorDirPin1,HIGH);
  digitalWrite(LeftMotorDirPin2,LOW);
  analogWrite(speedPinL,speed);
  analogWrite(speedPinR,speed);
}

// put angle within 0-360 range
float normalizeAngle(float angle)
{
  float normalizedAngle;
  if (angle < 0)
  {
    normalizedAngle = 360 - fmod(abs(angle), 360);
  }
  else 
  {
    normalizedAngle = fmod(angle, 360);
  }
  return normalizedAngle;
}

// delay while still updating MPU
void delayMPU(unsigned long ms)
{
  long startTime = millis();
  while (millis() - startTime < ms)
  {
    mpu.update();
  }
}

bool turning;
void setAngle(float angle, byte speed = 140)
{
  mpu.update();
  float currAngle = normalizeAngle(mpu.getAngleZ());
  turning = true;
  while (abs(normalizeAngle(mpu.getAngleZ()) - angle) > 1)
  {
    mpu.update();
    currAngle = normalizeAngle(mpu.getAngleZ());
    float clockwiseTurn = fmod(currAngle - angle + 360, 360);
    float counterclockwiseTurn = fmod(angle - currAngle + 360, 360);
    if (clockwiseTurn < counterclockwiseTurn)
    {
      turnRight(speed);
    }
    else 
    {
      turnLeft(speed);
    } 
  }
  stop();
  turning = false;
}
int pulses = 0;

// don't include pulses from wheel turning
void pulseEvent() {
  if (!turning)
  { 
    pulses++;
  }
}

void moveDistanceCM(float distance)
{
  // 3.65 pulses/cm calibrated on 50cm distance
  int pulsesRequired = abs(distance) * 3.65;
  while (pulses < pulsesRequired)
  {
    mpu.update();
    if (distance > 0)
    {
      forwards();
    }
    else 
    {
      reverse();
    }
  }
  pulses = 0;
  stop();
}

void setup() {
  initPins();
  pinMode(buttonPin, INPUT);
  // wait for button press to start program
  while (digitalRead(buttonPin) != HIGH) {};
  pulses = 0;
  Wire.begin();
  byte status = mpu.begin();
  delay(1000);
  attachInterrupt(digitalPinToInterrupt(2), pulseEvent, RISING);
  mpu.calcOffsets();
  mpu.update();
}

void loop(){
  pulses = 0;
  Vector<Point> path = getPath();
  for (int i = 0; i < path.size() - 1; i++)
  {
      if (path[i + 1].y - path[i].y == 2)
      {
          // down
          setAngle(180);
          delayMPU(100);
          setAngle(180, 100);
          delayMPU(50);
          setAngle(180, 100);
          delayMPU(50);
          moveDistanceCM(50);
          delayMPU(100);
      }
      else if (path[i + 1].y - path[i].y == -2)
      {
          // up
          setAngle(0);
          delayMPU(100);
          setAngle(0, 100);
          delayMPU(50);
          setAngle(0, 100);
          delayMPU(50);
          moveDistanceCM(50);
          delayMPU(100);
      }
      else if (path[i + 1].x - path[i].x == 2)
      {
          // right
          setAngle(270);
          delayMPU(100);
          setAngle(270, 100);
          delayMPU(50);
          setAngle(270, 100);
          delayMPU(50);
          moveDistanceCM(50);
          delayMPU(100);
      }
      else if (path[i + 1].x - path[i].x == -2)
      {
          // left
          setAngle(90);
          delayMPU(100);
          setAngle(90, 100);
          delayMPU(50);
          setAngle(90, 100);
          delayMPU(50);
          moveDistanceCM(50);
          delayMPU(100);
      }
  }
  
  while (digitalRead(buttonPin) != HIGH) {};
}