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

uint8_t * heapptr, * stackptr;

int mem_left() 
{
  stackptr = (uint8_t *)malloc(4); 
  heapptr = stackptr;             
  free(stackptr);                        
  stackptr =  (uint8_t *)(SP);      
  return stackptr - heapptr;
}

void initPins(){
  pinMode(RightMotorDirPin1, OUTPUT); 
  pinMode(RightMotorDirPin2, OUTPUT); 
  pinMode(speedPinL, OUTPUT);  

  pinMode(LeftMotorDirPin1, OUTPUT);
  pinMode(LeftMotorDirPin2, OUTPUT); 
  pinMode(speedPinR, OUTPUT); 
  stop();
}

void forwards(){
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

void stop(){
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2,LOW);
  digitalWrite(LeftMotorDirPin1,LOW);
  digitalWrite(LeftMotorDirPin2,LOW);
}

void turnLeft(byte speed = 140, int t = 0)  //Turn left
{
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2,LOW);
  digitalWrite(LeftMotorDirPin1,LOW);
  digitalWrite(LeftMotorDirPin2,HIGH);
  analogWrite(speedPinL,speed);
  analogWrite(speedPinR,speed);
  delay(t);
}
void turnRight(byte speed = 140, int t = 0)  //Turn right
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2,HIGH);
  digitalWrite(LeftMotorDirPin1,HIGH);
  digitalWrite(LeftMotorDirPin2,LOW);
  analogWrite(speedPinL,speed);
  analogWrite(speedPinR,speed);
  delay(t);
  // delayMPU(t);
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

// change speed based on degrees to turn
// y = 100x/90 + 100
byte calcSpeed(int degreesToTurn)
{
  if (degreesToTurn > 90) return 180;
  if (degreesToTurn < 0) return 0;
  return (70 * degreesToTurn / 90) + 110;
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

// float currAngle = 0;
bool turning;
// returns true if it turned, false if it just reversed (for 180 degrees)
bool setAngle(float angle, byte speed = 140)
{
  
  mpu.update();
  float currAngle = normalizeAngle(mpu.getAngleZ());
  // if (abs(currAngle - angle) >= 88 && abs(currAngle - angle) <= 92)
  // {
  //   angle-= 7;
  // }
  // else if (abs(currAngle - angle) >= 178 && abs(currAngle - angle) <= 182)
  // {
  //   angle-= 10;
  // }
  // Serial.println(angle);
  if (abs(currAngle - angle) >= 177 && abs(currAngle - angle) <= 183)
  {
    moveDistanceCM(-50.0);
    return false;
  }
  turning = true;
  while (abs(normalizeAngle(mpu.getAngleZ()) - angle) > 1)
  {
    mpu.update();
    currAngle = normalizeAngle(mpu.getAngleZ());
    float clockwiseTurn = fmod(currAngle - angle + 360, 360);
    float counterclockwiseTurn = fmod(angle - currAngle + 360, 360);
    // Serial.println(mpu.getAngleZ());
    if (clockwiseTurn < counterclockwiseTurn)
    {
      // turnRight(calcSpeed(clockwiseTurn));
      turnRight(speed);
    }
    else 
    {
      // turnLeft(calcSpeed(counterclockwiseTurn));
      turnLeft(speed);
    } 
  }
  stop();
  turning = false;
  return true;
  // currAngle = angle;
  
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
  // 3.42 pulses/cm for 50cm
  
  int pulsesRequired = abs(distance) * 3.42;
  // Serial.println(pulsesRequired);
  while (pulses < pulsesRequired)
  {
    // keep calling mpu to update 
    mpu.update();
    // Serial.println(pulses);
    if (distance > 0)
    {
      forwards();
    }
    else 
    {
      reverse();
    }
    // mpu.update();
    // float angle = normalizeAngle(mpu.getAngleZ());
    // if (angle > 3 && angle < 357)
    // {
    //   setAngle(currAngle);
    // }
  }
  pulses = 0;
  stop();
}

void setup() {
  initPins();
  pinMode(buttonPin, INPUT);
  Serial.begin(9600);
  while (!Serial) {};
  // Serial.println("hello");
  pulses = 0;
  Wire.begin();
  byte status = mpu.begin();
  delay(1000);
  attachInterrupt(digitalPinToInterrupt(2), pulseEvent, RISING);
  mpu.calcOffsets();
  mpu.update();
  // wait for button press to start program
  while (digitalRead(buttonPin) != HIGH) {};
}

void loop(){
  // Serial.println(mpu.getAngleZ());
  // Serial.println(mem_left());
  // moveDistanceCM(200);
  Vector<Point> path = getPath();
  // moveDistanceCM(10);
  // delayMPU(100);
  // setAngle(270);
  // delayMPU(100);
  // setAngle(270, 100);
  // delayMPU(50);
  // // moveDistanceCM(10);
  // delayMPU(50);
  // setAngle(90);
  
  // moveDistanceCM(-10);
  // setAngle(90);
  // setAngle(0);
  // setAngle(270);
  // setAngle(90);
  // // Serial.println("after");
  // for (Point p : path)
  // {
  //   Serial.print(p.y);
  //   Serial.print(" ");3
  //   Serial.println(p.x);
  // }
  // turnRight(150, 580);
  // stop();
  // turnLeft();
  // while(mpu.getAngleZ() < 90)
  // {
  //   mpu.update();
  //   Serial.println(mpu.getAngleZ());
  // }
  // stop();
  // setAngle(90);
  // delayMPU(100);
  // setAngle(90);
  // Serial.println(mpu.getAngleZ());
  // setAngle(270);
  // setAngle(180);
  // setAngle(170);
  // delayMPU(100);
  // Serial.println(mpu.getAngleZ());
  for (int i = 0; i < path.size() - 1; i++)
  {
      // Serial.println(path[i].y);
      // Serial.print(" ");
      // Serial.println(path[i].x);
      if (path[i + 1].y - path[i].y == 2)
      {
          // down
          // Serial.println("down");
          if (setAngle(180))
          {
            delayMPU(100);
            setAngle(180, 100);
            delayMPU(50);
            moveDistanceCM(50);
            delayMPU(100);
          }
          
      }
      else if (path[i + 1].y - path[i].y == -2)
      {
          // up
          // Serial.println("up");
          if (setAngle(0))
          {
            delayMPU(100);
            setAngle(0, 100);
            delayMPU(50);
            moveDistanceCM(50);
            delayMPU(100);
          }
          
      }
      else if (path[i + 1].x - path[i].x == 2)
      {
          // right
          // Serial.println("right");
          if (setAngle(270))
          {
            delayMPU(100);
            setAngle(270, 100);
            delayMPU(50);
            moveDistanceCM(50);
            delayMPU(100);
          }
          
      }
      else if (path[i + 1].x - path[i].x == -2)
      {
          // left
          // Serial.println("left");
          if (setAngle(90))
          {
            delayMPU(100);
            setAngle(90, 100);
            delayMPU(50);
            moveDistanceCM(50);
            delayMPU(100);
          }
          
      }
  }

  delay(50000);
}