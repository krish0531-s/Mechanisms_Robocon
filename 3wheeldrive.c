#include <USBHost_t36.h>

//Pinouts
int pwmPin1 = 2;//2
int dirPin1 = 4;//4

int pwmPin2 = 3;
int dirPin2 = 5;

int pwmPin3 = 23;
int dirPin3 = 21;

float L = 0.315;
float r = 0.1;

float Vx = 0.0;
float Vy = 0.0;
float w = 0.0;

int pwmVal = 0;

float maxSpeed = 0.2;

int deadZone = 20;

USBHost myusb;                           
BluetoothController bluet(myusb, true, "0000"); 
JoystickController joystick(myusb);

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 4000);        
  Serial.println("Waiting for PS4 controller...");
  myusb.begin();    

  pinMode(pwmPin1,OUTPUT);
  pinMode(dirPin1,OUTPUT);  

  pinMode(pwmPin2,OUTPUT);
  pinMode(dirPin2,OUTPUT);

  pinMode(pwmPin3,OUTPUT);
  pinMode(dirPin3,OUTPUT);                         
}

void loop() {
  myusb.Task();  

  if (joystick.available()) {  
    //mapping joysticks
    int lx = map1(joystick.getAxis(1),0,255,-127,127);        
    int ly = map1(joystick.getAxis(2),0,255,-127,127);
    int rx = map1(joystick.getAxis(3),0,255,-127,127); 

    //deadzone
    if (abs(lx) < deadZone) lx = 0;
    if (abs(ly) < deadZone) ly = 0;
    if (abs(rx) < deadZone) rx = 0;            

    Serial.print("LX: "); 
    Serial.print(lx);
    Serial.print("  LY: "); 
    Serial.print(ly);
    Serial.print(" RX: ");
    Serial.println(rx);

    Vx = -((float)ly/127.0);
    Vy = -((float)lx/127.0);
    w  = ((float)rx/127.0);

    //Inverse Kinematics
    float w1 = ((-0.866 * Vx - 0.5 * Vy + L * w) / r);
    float w2 = -(( 0.866 * Vx - 0.5 * Vy + L * w) / r);
    float w3 = (( 0.0   * Vx + 1.0 * Vy + L * w) / r);     

    driveMotor(pwmPin1, dirPin1, w1);
    driveMotor(pwmPin2, dirPin2, w2);
    driveMotor(pwmPin3, dirPin3, w3);

    Serial.print("  Vx: "); Serial.print(Vx);
    Serial.print("  Vy: "); Serial.print(Vy);
    Serial.print("  w1: "); Serial.print(w1);
    Serial.print("  w2: "); Serial.print(w2);
    Serial.print("  w3: "); Serial.println(w3);

    joystick.joystickDataClear();
  }
}

//set motor function for individual wheels
void driveMotor(int pwm, int dir, float speed){
  int pwmVal = map1(abs(speed) * 1000, 0, maxSpeed * 10000, 0, 40);
  pwmVal = constrain(pwmVal, 0, 40);

  if (pwmVal < 5) pwmVal = 0;

  if(speed > 0){
    digitalWrite(dir,HIGH);
    analogWrite(pwm,pwmVal);
  } else if(speed < 0){
    digitalWrite(dir,LOW);
    analogWrite(pwm,abs(pwmVal));
  } else {
    analogWrite(pwm,0);
  }

  Serial.println(pwmVal);
}

// map function for float
int map1(float x, float in_min, float in_max, float out_min, float out_max) {
    float mapped = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    return (int)mapped;
}