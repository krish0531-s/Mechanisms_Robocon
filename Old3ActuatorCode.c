// Example: "L 30", "X -40", "Y 0"

struct Motor {
  int LPWM;
  int RPWM;
  int targetRPM;
  int pwmValue;
};

const int MAX_RPM = 50; // rated speed

// --- Define each actuator’s pins ---
Motor lift  = {4, 5, 0, 0};   // Lifting actuator
Motor xAxis = {6, 7, 0, 0};   // KFS X-axis actuator
Motor yAxis = {8, 9, 0, 0};   // KFS Y-axis actuator

void setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  setupMotor(lift);
  setupMotor(xAxis);
  setupMotor(yAxis);

  Serial.println("=== Multi-Actuator Control Ready ===");
  Serial.println("Commands: L <rpm>, X <rpm>, Y <rpm>");
  Serial.println("Example: L 30  → Lift forward 30 RPM");
  Serial.println("         X -40 → X-axis reverse 40 RPM");
  Serial.println("         Y 0   → Stop Y-axis");
}


void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() == 0) return;

    char motorID = toupper(input.charAt(0));  // first letter = which motor
    int spaceIndex = input.indexOf(' ');
    int rpm = 0;
    if (spaceIndex > 0) {
      rpm = input.substring(spaceIndex + 1).toInt();
    } else {
      Serial.println(" Invalid input! Use format: L 30");
      return;
    }

    rpm = constrain(rpm, -MAX_RPM, MAX_RPM);
    controlMotor(motorID, rpm);
  }

  // Keep all motors running at their target RPM
  updateMotor(lift);
  updateMotor(xAxis);
  updateMotor(yAxis);
}



void setupMotor(Motor &m) {
  pinMode(m.LPWM, OUTPUT);
  pinMode(m.RPWM, OUTPUT);
  analogWrite(m.LPWM, 0);
  analogWrite(m.RPWM, 0);
}

void controlMotor(char id, int rpm) {
  Motor *m = nullptr;

  if (id == 'L') m = &lift;
  else if (id == 'X') m = &xAxis;
  else if (id == 'Y') m = &yAxis;

  if (m == nullptr) {
    Serial.println(" Unknown motor ID! Use L, X, or Y");
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