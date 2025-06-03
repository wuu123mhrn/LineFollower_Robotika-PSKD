#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Motor pins
#define rmf 10  // IN1 (Right Motor Forward)
#define rmb 11  // IN2 (Right Motor Backward)
#define lmf 6   // IN3 (Left Motor Forward)
#define lmb 9   // IN4 (Left Motor Backward)

// Multiplexer pins
#define s0 2
#define s1 3
#define s2 4
#define s3 5
#define sig A6

// PID constants
#define Kp 27.0  // Proportional gain
#define Ki 0.2    // Integral gain
#define Kd 4.0    // Derivative gain

// Motor speed settings
#define BASE_SPEED 180
#define MAX_SPEED 200

// Sensor thresholds (individual for each sensor)
int thresholds[12] = {700, 700, 600, 600, 600, 600, 600, 600, 600, 600, 700, 700};

// Variables
int sensorValues[12]; // Analog readings
int binarySensors[12]; // Binary readings (0 or 1)
unsigned long bin_Sensor; // Binary sensor state as a single value
float error = 0;
float previousError = 0;
float integral = 0;
unsigned long lastTime = 0;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Initialize OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Line Follower"));
  display.display();

  // Set motor pins as output
  pinMode(rmf, OUTPUT);
  pinMode(rmb, OUTPUT);
  pinMode(lmf, OUTPUT);
  pinMode(lmb, OUTPUT);

  // Set multiplexer pins as output
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(sig, INPUT);
}

void loop() {
  readSensors();
  calculateError();
  pidControl();
  updateDisplay();
  delay(10); // Small delay for stability
}




void readSensors() {
  bin_Sensor = 0;
  for (int i = 0; i < 12; i++) {
    // Select multiplexer channel
    digitalWrite(s0, bitRead(i, 0));
    digitalWrite(s1, bitRead(i, 1));
    digitalWrite(s2, bitRead(i, 2));
    digitalWrite(s3, bitRead(i, 3));

    // Read analog value
    sensorValues[i] = analogRead(sig);
    // Convert to binary based on individual threshold
    binarySensors[i] = (sensorValues[i] > thresholds[i]) ? 1 : 0;
    // Build binary sensor state
    bin_Sensor |= (binarySensors[i] << (11 - i));
  }
}

void calculateError() {
  // Map binary sensor readings to error values
  if (bin_Sensor == 0b100000000000) error = -5.5;
  else if (bin_Sensor == 0b110000000000) error = -5;
  else if (bin_Sensor == 0b010000000000) error = -4.5;
  else if (bin_Sensor == 0b011000000000) error = -4;
  else if (bin_Sensor == 0b001000000000) error = -3.5;
  else if (bin_Sensor == 0b001100000000) error = -3;
  else if (bin_Sensor == 0b000100000000) error = -2.5;
  else if (bin_Sensor == 0b000110000000) error = -2;
  else if (bin_Sensor == 0b000010000000) error = -1.5;
  else if (bin_Sensor == 0b000011000000) error = -1;
  else if (bin_Sensor == 0b000001000000) error = -0.5;
  else if (bin_Sensor == 0b000001100000) error = 0;
  else if (bin_Sensor == 0b000000100000) error = 0.5;
  else if (bin_Sensor == 0b000000110000) error = 1;
  else if (bin_Sensor == 0b000000010000) error = 1.5;
  else if (bin_Sensor == 0b000000011000) error = 2;
  else if (bin_Sensor == 0b000000001000) error = 2.5;
  else if (bin_Sensor == 0b000000001100) error = 3;
  else if (bin_Sensor == 0b000000000100) error = 3.5;
  else if (bin_Sensor == 0b000000000110) error = 4;
  else if (bin_Sensor == 0b000000000010) error = 4.5;
  else if (bin_Sensor == 0b000000000011) error = 5;
  else if (bin_Sensor == 0b000000000001) error = 5.5;
  else if (bin_Sensor == 0b111110011111) error = 0;
  else error = previousError; // Maintain last error if no sensors detect line
}

void pidControl() {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0; // Time in seconds
  lastTime = currentTime;

  // Calculate PID terms
  float proportional = error;
  integral += error * deltaTime;
  float derivative = (error - previousError) / deltaTime;
  float output = Kp * proportional + Ki * integral + Kd * derivative;

  // Calculate motor speeds
  int leftSpeed = BASE_SPEED + output;
  int rightSpeed = BASE_SPEED - output;

  // Constrain motor speeds
  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  // Drive motors
  driveMotors(leftSpeed, rightSpeed);

  previousError = error;
}

void driveMotors(int leftSpeed, int rightSpeed) {
  // Left motor
  if (leftSpeed > 0) {
    analogWrite(lmf, leftSpeed);
    analogWrite(lmb, 0);
  } else {
    analogWrite(lmf, 0);
    analogWrite(lmb, -leftSpeed);
  }

  // Right motor
  if (rightSpeed > 0) {
    analogWrite(rmf, rightSpeed);
    analogWrite(rmb, 0);
  } else {
    analogWrite(rmf, 0);
    analogWrite(rmb, -rightSpeed);
  }
}


void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(F("Error: "));
  display.println(error);
  display.print(F("Sensors: "));
  for (int i = 0; i < 12; i++) {
    display.print(binarySensors[i]);
  }
  display.println();
  display.println(F("Analog:"));
  for (int i = 0; i < 12; i++) {
    display.print(sensorValues[i]);
    if (i < 11) display.print(F(" "));
    if (i == 5) display.println(); // Split into two lines for readability
  }
  display.display();
}
