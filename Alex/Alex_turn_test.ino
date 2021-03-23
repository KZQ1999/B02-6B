#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <MPU6050_tockn.h>

#define LF                  6   // Left forward pin
#define LR                  5   // Left reverse pin
#define RF                  10  // Right forward pin
#define RR                  11  // Right reverse pin
#define offSet 10

MPU6050 mpu6050(Wire);
int incomingByte = 0;

void setupMotors()
{
  pinMode(RR, OUTPUT);
  pinMode(RF, OUTPUT);
  pinMode(LF, OUTPUT);
  pinMode(LR, OUTPUT);
}

/*
ISR(INT0_vect)
{
  leftISR();
}
ISR(INT1_vect)
{
  rightISR();
}

// Functions to be called by INT0 and INT1 ISRs.
void leftISR()
{
  if (dir == FORWARD) {
    leftForwardTicks++;
    forwardDist = (unsigned long) ((float)leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }
  if (dir == BACKWARD) {
    leftReverseTicks++;
    reverseDist = (unsigned long) ((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }
}

void rightISR()
{
  if (dir == FORWARD) {
    rightForwardTicks++;
  }
  if (dir == BACKWARD) {
    rightReverseTicks++;
  }
} */

// Convert percentages to PWM values
int pwmVal(float speed)
{
  if (speed < 0.0)
    speed = 0;

  if (speed > 100.0)
    speed = 100.0;

  return (int) ((speed / 100.0) * 255.0);
}

/*
void forward(float dist, float speed)
{
  dir = FORWARD;
  int val = pwmVal(speed);
  mpu6050.update();
  float initial = mpu6050.getAngleZ(); //Alex's initial bearing. Should not change while moving forward.
  float curr = initial;
  
  if (dist > 0)
    deltaDist = dist;
  else
    deltaDist = 9999999;
  newDist = forwardDist + deltaDist;

  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.
  analogWrite(LF, val);
  analogWrite(RF, val * RMUL);
  analogWrite(LR, 0);
  analogWrite(RR, 0);
}

/* Reverse "dist" cm.
   speed is the relative power of the motors.
   distance of 0 will cause Alex to reverse indefinitely

void reverse(float dist, float speed)
{
  dir = BACKWARD;

  int val = pwmVal(speed);
  if (dist > 0)
  {
    deltaDist = dist;
  }
  else
  {
    deltaDist = 9999999;
  }

  newDist = forwardDist + deltaDist;

  analogWrite(LR, val);
  analogWrite(RR, val);
  analogWrite(LF, 0);
  analogWrite(RF, 0);
} */

/* Turn left "ang" degrees.
   Speed indicates the relative power of the motors
*/
void left(float ang, float speed)
{
  int val = pwmVal(speed);
  mpu6050.update();
  float curr = mpu6050.getAngleZ();
  float target = curr + (ang - offSet); //may need to tune the angle for target to account for momentum
  while (curr < target)
  {
    mpu6050.update();
    curr = mpu6050.getAngleZ();
    analogWrite(LR, 0);
    analogWrite(RF, 0);
    analogWrite(LF, val);
    analogWrite(RR, val);
  }
  stop();
}

/* Turn right "ang" degrees.
   speed indicates the relative power of the motors
*/
void right(float ang, float speed)
{
  int val = pwmVal(speed);
  mpu6050.update();
  float curr = mpu6050.getAngleZ();
  float target = curr - (ang - offSet); //may need to tune the angle for target to account for momentum
  while (curr > target)
  {
    mpu6050.update();
    curr = mpu6050.getAngleZ();
    analogWrite(RR, 0);
    analogWrite(LF, 0);
    analogWrite(LR, val);
    analogWrite(RF, val);
  }
  stop();
}

// Stop Alex. To replace with bare-metal code later.
void stop()
{
  analogWrite(LF, 0);
  analogWrite(LR, 0);
  analogWrite(RF, 0);
  analogWrite(RR, 0);
}

void setup()
{
  setupMotors();
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
}

void loop()
{
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
  }
  if (incomingByte == '1') {
    left(90, 50.0);
  }
  if (incomingByte == '2') {
    right(90, 50.0);
  }
}
