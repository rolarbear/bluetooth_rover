
#include <Servo.h>
#include <NewPing.h>

#define NONE    48
#define UP      49
#define DOWN    50
#define LEFT    51
#define RIGHT   52
#define XBUTT   53
#define OBUTT   54
#define SQBUTT  55
#define TRBUTT  56
#define SELECT  57
#define START   65

#define TRIGGER_PIN  7  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     2  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 400 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
int firePin = 6;
int autoPin = 8;
int mc1 = A0;
int mc2 = A1;
int mc3 = A2;
int mc4 = A3;

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

Servo URservo;  // create servo object to control a servo 
Servo Xservo;
Servo Yservo;
int servoPos[] = {
  30, 90, 150}; //{0, 45, 90, 135, 180};
unsigned int distance[] = {
  20,20,20};
unsigned int xServoPos = 90;
unsigned int yServoPos = 130;
int i;
int a = 1;
int closeDistance = 12;
int inChar;
unsigned int distanceAhead;
boolean blockedAhead;

String inputString = "";

void fire(void)
{
  digitalWrite(firePin, HIGH);
  delay(100);
  digitalWrite(firePin, LOW);
}

boolean Blocked(void)
{
  if ((distance[0] < closeDistance) && (distance[1] < closeDistance) && 
    (distance[2] < closeDistance))
  {
    blockedAhead = true;
  }
  else
  {
    blockedAhead = false;
  }
  return blockedAhead;
}

void ReadDistanceCW(void)
{
  for (i = 0; i<3; i++)
  {
    URservo.write(servoPos[i]);
    delay(400);
    unsigned int uS = sonar.ping();
    distance[i] = (uS / US_ROUNDTRIP_CM);
    delay(50);
    URservo.write(servoPos[i]);
    delay(100);
  }
}

void ReadDistanceCCW(void)
{
  for (i = 2; i>=0; i--)
  {
    URservo.write(servoPos[i]);
    delay(400);
    unsigned int uS = sonar.ping();
    distance[i] = (uS / US_ROUNDTRIP_CM);
    delay(50);
  }
}

void ReadDistanceAhead(void)
{
  URservo.write(servoPos[1]);
  delay(200);
  unsigned int uS = sonar.ping();
  distance[1] = (uS / US_ROUNDTRIP_CM);
  delay(50);
}

int FindOpenArea(void)
{
  unsigned int disA, cnt;
  disA = distance[0];
  cnt = 0;
  for (i = 0; i <3; i++)
  {

    if (distance[i] == 0)
    {
      distance[i] = 400;
    }

    if (distance[i] > disA)
    {
      disA = distance[i];
      cnt = i;
    }
  }
  return cnt;
}

void PrintReadings(void)
{
  for (i = 0; i<3; i++)
  {
    Serial.print(i);
    Serial.print(": ");
    Serial.print(distance[i]);
    Serial.println("cm");
  }
}

void stopDriving(void)
{
  digitalWrite(mc1,LOW);
  digitalWrite(mc3,LOW);
  digitalWrite(mc2,LOW);
  digitalWrite(mc4,LOW);
}
void DriveForeward()
{
  digitalWrite(mc1,HIGH);
  digitalWrite(mc2,LOW);
  digitalWrite(mc3,HIGH);
  digitalWrite(mc4,LOW);
}
void DriveReverse()
{
  digitalWrite(mc1,LOW);
  digitalWrite(mc2,HIGH);
  digitalWrite(mc3,LOW);
  digitalWrite(mc4,HIGH);
}
void TurnLeft()
{
  digitalWrite(mc1,LOW);
  digitalWrite(mc2,HIGH);
  digitalWrite(mc3,HIGH);
  digitalWrite(mc4,LOW);
}
void TurnRight()
{
  digitalWrite(mc1,HIGH);
  digitalWrite(mc2,LOW);
  digitalWrite(mc3,LOW);
  digitalWrite(mc4,HIGH);
}

void setup() {

  pinMode(mc1, OUTPUT);
  pinMode(mc2, OUTPUT);
  pinMode(mc3, OUTPUT);
  pinMode(mc4, OUTPUT);

  pinMode(firePin, OUTPUT);

  pinMode(autoPin, INPUT);

  digitalWrite(firePin, LOW);
  digitalWrite(mc1,LOW);
  digitalWrite(mc2,LOW);
  digitalWrite(mc3,LOW);
  digitalWrite(mc4,LOW);

  URservo.attach(9);  // PAN SERVO
  Xservo.attach(10);  // x pan servo
  Yservo.attach(6);  // y tilt servo
  Xservo.write(xServoPos);
  Yservo.write(yServoPos);
  URservo.write(servoPos[1]);
  delay(1000);
  Serial.begin(115200);
  Serial.println("Start");
}

//boolean autonomous = true;

void loop() {

  int autonomous = digitalRead(autoPin);

  if(autonomous > 0)
  {
    Serial.print('A');
    delay(10);
    if(Serial.available()) {
      a = Serial.read();
    }
    while (a == 1)
    {
      //DriveForeward();
      delay(100);
      ReadDistanceAhead();

      if ((distance[1] <= closeDistance) && (distance[1] != 0))
      {
        stopDriving();
        delay(25);
        ReadDistanceCCW();
        delay(100);

        if (Blocked() == true)
        {
          DriveReverse();
          delay(1500);
          TurnLeft();
          delay(2000);
          stopDriving();
        }
        else
        {
          int opnArea = FindOpenArea();

          switch(opnArea){

          case 0:
            DriveReverse();
            delay(500);
            TurnLeft();
            delay(1000);
            stopDriving();
            break;

          case 1:
            break;

          case 2:
            DriveReverse();
            delay(500);
            TurnRight();
            delay(1000);
            stopDriving();
            break;

          default:
            stopDriving();
            break;
          }
        }
        distance[0] = 20;
        distance[1] = 20;
        distance[2] = 20;
      }
    }
    while(a == 0)
    {
      //put face servo code here
    }
  }
  
  if(autonomous == 0)
  {
    Serial.println("USER CONTROL");
    while (1)
    {
      if(Serial.available()) {
        inChar = Serial.read();
        Serial.println(inChar);

        if(inChar == UP)
        {
          DriveForeward();
          Serial.println("Forward");
        }
        if(inChar == DOWN)
        {
          DriveReverse();
          Serial.println("Backwards");
        }
        if(inChar == LEFT)
        {
          TurnLeft();
          Serial.println("Left");
        }
        if(inChar == RIGHT)
        {
          TurnRight();
          Serial.println("Right");
        }
        if(inChar == TRBUTT)
        {
          if(yServoPos < 140)
          {
            yServoPos += 3;
            Yservo.write(yServoPos);
          }
        }
        if(inChar == XBUTT)
        {
          if(yServoPos > 80)
          {
            yServoPos -= 3;
            Yservo.write(yServoPos);
          }
        }
        if(inChar == OBUTT)
        {
          if(xServoPos < 180)
          {
            xServoPos += 3;
            Xservo.write(xServoPos);
          }
        }
        if(inChar == SQBUTT)
        {
          if(xServoPos > 0)
          {
            xServoPos -= 3;
            Xservo.write(xServoPos);
          }
        }
        if(inChar == SELECT)
        {
          fire();
        }
        if(inChar == START)
        {

        }
        if(inChar == NONE)
        {
          stopDriving();
        }
        inChar == 0;
      }
    }
  }
}



