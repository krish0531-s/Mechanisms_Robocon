#include "USBHost_t36.h"

USBHost myusb;
JoystickController joystick(myusb);
BluetoothController bluet(myusb, true, "0000");   // Version does pairing to device
//BluetoothController bluet(myusb);   // version assumes it already was paired

//BOT VELOCITIES
float Vx = 0.0f;
float Vy = 0.0f;
float w  = 0.0f;

int RawValue = 255; //The raw value seen from 0
int DEADZONE = 10;
float maxSpeed = 0.4; //Amount of power being supplied.

//DRIVE GEOMETRY

// //RADIUS OF OMNI WHEEL IN METERS
// float r = 0.01;
//DISTANCE FROM CENTRE OF BOT TO WHEEL IN METERS
float L = 0.315;

//MOTOR PINS

//MOTOR 1
int pwmPin1 = 2;
int dirPin1 = 4;
//MOTOR 2
int pwmPin2 = 3;
int dirPin2 = 5;
//MOTOR 3
int pwmPin3 = 23;
int dirPin3 = 21;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 4000);
  Serial.println("Waiting for PS4 controller......");
  myusb.begin();

  //INITIALIZING PINS
  pinMode(pwmPin1,OUTPUT);
  pinMode(dirPin1,OUTPUT);  

  pinMode(pwmPin2,OUTPUT);
  pinMode(dirPin2,OUTPUT);

  pinMode(pwmPin3,OUTPUT);
  pinMode(dirPin3,OUTPUT);

  //STOP ALL MOTORS AT START
  driveMotor(pwmPin1, dirPin1, 0);
  driveMotor(pwmPin2, dirPin2, 0);
  driveMotor(pwmPin3, dirPin3, 0);

   
}

void loop() {
  myusb.Task();

  if(joystick.available()) {
    int lx = map1(joystick.getAxis(1), 0, RawValue, -RawValue/2 , RawValue/2);
    int ly = map1(joystick.getAxis(2), 0, RawValue, -RawValue/2 , RawValue/2);
    int rx = map1(joystick.getAxis(3), 0, RawValue, -RawValue/2 , RawValue/2);


    if(abs(lx) < DEADZONE) lx = 0;
    if(abs(ly) < DEADZONE) ly = 0;
    if(abs(rx) < DEADZONE) rx = 0;

    Serial.print("Lx: "); Serial.print(lx);
    Serial.print(" |Ly: "); Serial.print(ly);
    Serial.print(" |Rx: "); Serial.println(rx);


    Vy = (float)ly / (RawValue/2.0f);   // forward / backward
    Vx = (float)lx / (RawValue/2.0f);   // left / right
    w  = (float)rx / (RawValue/2.0f);   // rotation


    // Wheel 1 @ 90°
    float w1 = (-1.0f * Vy + 0.0f * Vx + L * w);
    // Wheel 2 @ 210°
    float w2 = ( 0.5f * Vy - 0.866f * Vx + L * w);
    // Wheel 3 @ 330°
    float w3 = ( 0.5f * Vy + 0.866f * Vx + L * w);

    //NORMALISE WHEEL VELOCITIES
    float maxWheel = max(abs(w1), max(abs(w2), abs(w3)));

    if (maxWheel > 1.0f) {
      w1 /= maxWheel;
      w2 /= maxWheel;
      w3 /= maxWheel;
    }

    //DRIVE MOTORS
    driveMotor(pwmPin1, dirPin1, w1);
    driveMotor(pwmPin2, dirPin2, w2);
    driveMotor(pwmPin3, dirPin3, w3);
  }

}
//drive individual motors
void driveMotor(int pwmPin, int dir, float speed)
{
  // speed is already in [-1, 1]

  float speedNorm = abs(speed) * maxSpeed;

  speedNorm = constrain(speedNorm, 0.0f, 1.0f);

  int pwmVal = (int)(speedNorm * 255.0f);

  // Deadband
  if (pwmVal < 20) pwmVal = 0;

  if (speed > 0) {
    digitalWrite(dir, HIGH);
    analogWrite(pwmPin, pwmVal);
  }
  else if (speed < 0) {
    digitalWrite(dir, LOW);
    analogWrite(pwmPin, pwmVal);
  }
  else {
    analogWrite(pwmPin, 0);
  }
}


// map function for float
int map1(float x, float in_min, float in_max, float out_min, float out_max) {
    float mapped = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    return (int)mapped;
}
