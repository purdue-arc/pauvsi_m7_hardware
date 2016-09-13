/*
  # This Sample code is for testing the Digital Servo Shield.

  # Editor : YouYou
  # Date   : 2014.10.24
  # Ver    : 1.0
  # Product: Digital Servo Shield for Arduino
  # SKU    :

  # Hardwares:
  1. Arduino UNO
  2. Digital Servo Shield for Arduino
  3. Digital Servos( Compatible with AX-12,CDS55xx...etc)
  4. Power supply:6.5 - 12V

*/

#include <SPI.h>
#include <ServoCds55.h>
ServoCds55 myservo;

#define TIME_SCALE 0.5

#define SERVO_1_LOW 50
#define SERVO_1_HIGH 250
#define SERVO_2_LOW 60
#define SERVO_2_HIGH 240
#define SERVO_3_LOW 0
#define SERVO_3_HIGH 300

#define SERVO_1_FOLD 150
#define SERVO_2_FOLD 240
#define SERVO_3_FOLD 0

int servo1_pos = 150;
int servo2_pos = 150;
int servo3_pos = 150;

//returns the time to move in seconds
double setPos(int servo1, int servo2, int servo3, int maxVelocity)
{
  //constrain the requested servo positions
  servo1 = constrain(servo1, SERVO_1_LOW, SERVO_1_HIGH);
  servo2 = constrain(servo2, SERVO_2_LOW, SERVO_2_HIGH);
  servo3 = constrain(servo3, SERVO_3_LOW, SERVO_3_HIGH);
  
  //find the maximum distance that a servo will need to move
  int servo1_move_dist = abs(servo1 - servo1_pos);
  int servo2_move_dist = abs(servo2 - servo2_pos);
  int servo3_move_dist = abs(servo3 - servo3_pos);

  int maxDelta = servo1_move_dist;
  if (servo2_move_dist > maxDelta)
  {
    maxDelta = servo2_move_dist;
  }
  if (servo3_move_dist > maxDelta)
  {
    maxDelta = servo3_move_dist;
  }

  Serial.print("MAX DELTA ");
  Serial.println(maxDelta);

  double moveTime = (double)maxDelta / (double)maxVelocity;

  if (moveTime != 0)
  {
    myservo.setVelocity(round((double)servo1_move_dist / moveTime));
    myservo.write(1, servo1);
    myservo.setVelocity(round((double)servo2_move_dist / moveTime));
    Serial.println(round((double)servo2_move_dist / moveTime));
    myservo.write(2, servo2);
    myservo.setVelocity(round((double)servo3_move_dist / moveTime));
    myservo.write(3, servo3);
  }
  else
  {
    myservo.setVelocity(maxVelocity);
    myservo.write(1, servo1);
    myservo.setVelocity(maxVelocity);
    Serial.print("ZERO MOVE TIME - USING MAX VELOCITY ");
    Serial.println(maxVelocity);
    myservo.write(2, servo2);
    myservo.setVelocity(maxVelocity);
    myservo.write(3, servo3);
  }

  servo1_pos = servo1;
  servo2_pos = servo2;
  servo3_pos = servo3;

  return moveTime * TIME_SCALE;
}

void setup (void)
{
  Serial.begin (115200);
  myservo.begin ();
  delay(10000);
  Serial.println("INITIALIZE");
  double runTime = setPos(150, 150, 150, 20);
  Serial.println(runTime);
  delay(round(1000 * runTime));
  delay(3000);
}

void loop (void)
{
  double runTime = setPos(240, 70, 230, 20);
  delay(round(1000 * runTime));
  
  runTime = setPos(50, 200, 0, 30);
  delay(round(1000 * runTime));
  
  runTime = setPos(50, 60, 150, 20);
  delay(round(1000 * runTime));

  runTime = setPos(50, 200, 10, 30);
  delay(round(1000 * runTime));

  runTime = setPos(150, 200, 10, 30);
  delay(round(1000 * runTime));

  runTime = setPos(240, 60, 150, 10);
  delay(round(1000 * runTime));

  runTime = setPos(240, 200, 10, 30);
  delay(round(1000 * runTime));

  runTime = setPos(240, 60, 300, 30);
  delay(round(1000 * runTime));
}

