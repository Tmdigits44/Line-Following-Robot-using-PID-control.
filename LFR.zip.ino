// -------- PIN DEFINITIONS --------

// Sensor pins (8 sensors)
int sensorPins[8] = {2, 3, 4, 8, 9, A0, A1, A2};

// L298N Motor Driver Pins
#define ENA 5
#define IN1 6
#define IN2 7
#define ENB 10
#define IN3 11
#define IN4 12

// -------- PID CONSTANTS --------
float Kp = 20;     // Proportional
float Ki = 0.0;    // Integral
float Kd = 12;     // Derivative

// -------- VARIABLES --------
int sensorValue[8];
int weights[8] = {-3500, -2500, -1500, -500, 500, 1500, 2500, 3500};

int error = 0;
int lastError = 0;
float integral = 0;
float derivative = 0;

int baseSpeed = 120;   // Normal speed
int maxSpeed = 255;

// -------- SETUP --------
void setup() {
  for (int i = 0; i < 8; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.begin(9600);
}

// -------- MAIN LOOP --------
void loop() {
  readSensors();
  calculateError();
  pidControl();
}

// -------- SENSOR READ --------
void readSensors() {
  for (int i = 0; i < 8; i++) {
    sensorValue[i] = digitalRead(sensorPins[i]);
  }
}

// -------- ERROR CALCULATION --------
void calculateError() {
  long sum = 0;
  int activeSensors = 0;

  for (int i = 0; i < 8; i++) {
    if (sensorValue[i] == HIGH) {
      sum += weights[i];
      activeSensors++;
    }
  }

  if (activeSensors > 0) {
    error = sum / activeSensors;
  } else {
    error = lastError;  // line lost
  }
}

// -------- PID CONTROL --------
void pidControl() {
  integral += error;
  derivative = error - lastError;

  float correction = (Kp * error) +
                     (Ki * integral) +
                     (Kd * derivative);

  int leftSpeed  = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;

  leftSpeed  = constrain(leftSpeed, 0, maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, maxSpeed);

  moveMotors(leftSpeed, rightSpeed);

  lastError = error;
}

// -------- MOTOR CONTROL --------
void moveMotors(int left, int right) {
  // Forward direction
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, left);
  analogWrite(ENB, right);
}