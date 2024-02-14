#define speedPinR 9
#define RightMotorDirPin1  12   
#define RightMotorDirPin2  11
#define speedPinL 6
#define LeftMotorDirPin1  7
#define LeftMotorDirPin2  8

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

float currAngle = 0;
bool turning;
void setAngle(float angle)
{
  turning = true;
  mpu.update();
  while (abs(normalizeAngle(mpu.getAngleZ()) - angle) > 1)
  {
    mpu.update();
    float currAngle = normalizeAngle(mpu.getAngleZ());
    float clockwiseTurn = fmod(currAngle - angle + 360, 360);
    float counterclockwiseTurn = fmod(angle - currAngle + 360, 360);

    if (clockwiseTurn < counterclockwiseTurn)
    {
      turnRight(calcSpeed(clockwiseTurn));
    }
    else 
    {
      turnLeft(calcSpeed(counterclockwiseTurn));
    } 
    delayMPU(10);
  }
  stop();
  turning = false;
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
  int pulsesRequired = distance * 3.42;
  while (pulses < pulsesRequired)
  {
    forwards();
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
  Serial.begin(9600);
  while (!Serial) {};
  // Serial.println("hello");
  pulses = 0;
  Wire.begin();
  byte status = mpu.begin();
  delay(2000);
  attachInterrupt(digitalPinToInterrupt(2), pulseEvent, RISING);
  mpu.calcOffsets();
  mpu.update();
}

void loop(){
  // Serial.println(mpu.getAngleZ());
  // Serial.println(mem_left());
  // moveDistanceCM(200);
  Vector<Point> path = getPath();
  // setAngle(90);
  // setAngle(0);
  // setAngle(270);
  // setAngle(90);
  // // Serial.println("after");
  // for (Point p : path)
  // {
  //   Serial.print(p.y);
  //   Serial.print(" ");
  //   Serial.println(p.x);
  // }
  for (int i = 0; i < path.size() - 1; i++)
  {
      // Serial.println(path[i].y);
      // Serial.print(" ");
      // Serial.println(path[i].x);
      if (path[i + 1].y - path[i].y == 2)
      {
          // down
          // Serial.println("down");
          setAngle(180);
          moveDistanceCM(50);
      }
      else if (path[i + 1].y - path[i].y == -2)
      {
          // up
          // Serial.println("up");
          setAngle(0);
          moveDistanceCM(50);
      }
      else if (path[i + 1].x - path[i].x == 2)
      {
          // right
          // Serial.println("right");
          setAngle(270);
          moveDistanceCM(50);
      }
      else if (path[i + 1].x - path[i].x == -2)
      {
          // left
          // Serial.println("left");
          setAngle(90);
          moveDistanceCM(50);
      }
  }

  delay(50000);
}