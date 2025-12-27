// Example: "FL 30", "BL -40", "FW 0", "BW 20"
//FL, BL , FW, BW are motor ID's
//FL: front lifting rack; BL: back lifting rack; FW: front wheel; BW: back Wheel
//To stop enter Motor ID 0 in serial monitor. Example- FL 0
//Run any motor individually. To run multiple motors send each command in seperate line
//Edit the pinouts for each motor given below ⬇

int fl_lpwm = 4; //Front lift pinouts
int fl_rpwm = 5;

int bl_lpwm = 6; //Back lift pinouts
int bl_rpwm = 7;

int fw_lpwm = 8; //Front wheel pinouts
int fw_rpwm = 9;

int bw_lpwm = 10; //Back wheel pinouts
int bw_rpwm = 11;

struct Motor {
  int LPWM;
  int RPWM;
  int targetRPM;
  int pwmValue;
};

const int MAX_RPM = 50; // rated speed

// --- Define each actuator’s pins ---
Motor frontLift  = {fl_lpwm, fl_rpwm, 0, 0};   // Front lifting rack
Motor backLift = {bl_lpwm, bl_rpwm, 0, 0};   // Back Lifiting rack
Motor frontWheel = {fw_lpwm, fw_rpwm, 0, 0};   // Front wheel
Motor backWheel = {bw_lpwm, bw_rpwm, 0, 0};  //Back wheel
void setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT); //Lightup internal led
  digitalWrite(13, HIGH);

  setupMotor(frontLift);
  setupMotor(backLift);
  setupMotor(frontWheel);
  setupMotor(backWheel);

  Serial.println("=== Multi-Actuator Control Ready ===");
  Serial.println("Commands: FL <rpm>, BL <rpm>, FW <rpm>, BW <rpm");
  Serial.println("Example: FL 30  → Lift forward 30 RPM");
  Serial.println("         BL -40 → Back lift reverse 40 RPM");
  Serial.println("         FW 0   → Stop front wheel");
}


void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() == 0) return;

    String motorID = input.substring(0,2);  // first two letter = which motor
    motorID.toUpperCase();
    int spaceIndex = input.indexOf(' ');
    int rpm = 0;
    if (spaceIndex > 0) {
      rpm = input.substring(spaceIndex + 1).toInt();
    } else {
      Serial.println(" Invalid input! Use format: FL 30");
      return;
    }

    rpm = constrain(rpm, -MAX_RPM, MAX_RPM);
    controlMotor(motorID, rpm);
  }

  // Keep all motors running at their target RPM
  updateMotor(frontLift);
  updateMotor(backLift);
  updateMotor(frontWheel);
  updateMotor(backWheel);
}



void setupMotor(Motor &m) {
  pinMode(m.LPWM, OUTPUT);
  pinMode(m.RPWM, OUTPUT);
  analogWrite(m.LPWM, 0);
  analogWrite(m.RPWM, 0);
}

void controlMotor(String id, int rpm) {
  Motor *m = nullptr;

  if (id == "FL") m = &frontLift;
  else if (id == "BL") m = &backLift;
  else if (id == "FW") m = &frontWheel;
  else if (id == "BW") m = &backWheel;

  if (m == nullptr) {
    Serial.println(" Unknown motor ID! Use FL, BL, FW or BW");
    return;
  }

  m->targetRPM = rpm;
  Serial.print("using ");
  Serial.print(id);
  Serial.print(" target RPM set to ");
  Serial.println(rpm);
}

void updateMotor(Motor &m) {
  if (m.targetRPM > 0) {
    m.pwmValue = map(m.targetRPM, 0, MAX_RPM, 0, 255);
    analogWrite(m.LPWM, m.pwmValue);
    analogWrite(m.RPWM, 0);
  } 
  else if (m.targetRPM < 0) {
    m.pwmValue = map(abs(m.targetRPM), 0, MAX_RPM, 0, 255);
    analogWrite(m.LPWM, 0);
    analogWrite(m.RPWM, m.pwmValue);
  } 
  else {
    analogWrite(m.LPWM, 0);
    analogWrite(m.RPWM, 0);
  }
}