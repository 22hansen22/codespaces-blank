#include <Wire.h>
#include <SoftwareSerial.h>

// Pin definitions
#define ledPin 6
#define motorPin 9
#define flowsensor 24 // Pin for flow sensor

// Motor driver pins
const int enA = 11;   // Enable pin for motor (PWM pin)
const int in1 = 9;    // Input 1 for motor
const int in2 = 10;   // Input 2 for motor

// Software Serial for Bluetooth communication
SoftwareSerial bt(0, 1); // Rx, Tx

// Variables for flow rate measurement
double flow;           // Water flow in L/min
unsigned long currentTime;
unsigned long lastTime;
unsigned long pulse_freq; // Pulses from the flow sensor

// Variables for pressure sensor and calculations
float Lower_Range_Limit = 0.0;
float Upper_Range_Limit = 65.0; // kPa
float fadc, uADC, Range;
float Pressure, Pressure1, Pressure2, Pressure3;
float Temp;

// Buffer for I2C communication
unsigned char buf[6];
unsigned long dat;

// Interrupt function for flow sensor
void pulse() {
  pulse_freq++;
}

void setup() {
  // Initialize flow sensor and interrupts
  pinMode(flowsensor, INPUT);
  attachInterrupt(0, pulse, RISING); // Attach interrupt to flowsensor

  // Initialize serial communication
  Serial.begin(9600);
  bt.begin(9600); // Bluetooth Serial communication

  // Initialize I2C communication
  Wire.begin();

  // Configure pins for motor control
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  // Configure additional pins
  pinMode(ledPin, OUTPUT);
  pinMode(motorPin, OUTPUT);

  // Stop the motor initially
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(enA, 0);

  delay(1000); // Delay to ensure proper initialization
}

void loop() {
  delay(990); // Main loop delay
  Pressure3 = Pressure2; // Store the previous pressure reading

  // Request pressure data from sensor via I2C
  Wire.beginTransmission(0x6D);
  Wire.write(0x06); // Command to read pressure data
  Wire.endTransmission();
  Wire.requestFrom(0x6D, 6); // Request 6 bytes of data

  if (Wire.available() >= 6) {
    // Read data into buffer
    for (int i = 0; i < 6; i++) {
      buf[i] = Wire.read();
    }

    // Process the pressure data
    dat = buf[0];
    dat = (dat << 8) + buf[1];
    dat = (dat << 8) + buf[2];
    fadc = (dat & 0x800000) ? dat - 16777216.0 : dat;
    uADC = 3.3 * fadc / 8388608.0;
    Range = Upper_Range_Limit - Lower_Range_Limit;
    Pressure1 = Upper_Range_Limit * (0.8 * (fadc / (8388608.0 - 0.1))) - 5.5;
    Pressure2 = (65 / 0.8 * (fadc / 8388608.0 - 0.1)) - 0.45;
    Pressure = Pressure2 - Pressure3;

    // Process temperature data
    dat = buf[3];
    dat = (dat << 8) + buf[4];
    dat = (dat << 8) + buf[5];
    fadc = (dat & 0x800000) ? dat - 16777216.0 : dat;
    Temp = 25.0 + fadc / 65536.0;

    // Calculate flow and compensation
    flow = 1; // Placeholder for flow sensor input
    Serial.print("Pressure (kPa): ");
    Serial.println(Pressure2, 3);
    float volume = flow * 50 / 60;
    float comp = volume / Pressure;
    Serial.println(comp, 3);
  }

  // Check for commands via Serial
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n'); // Read input
    command.trim(); // Remove extra spaces

    // Parse commands
    if (command.startsWith("f")) {
      int speed = command.substring(1).toInt();
      setMotor('f', constrain(speed, 0, 100)); // Forward
    } else if (command.startsWith("b")) {
      int speed = command.substring(1).toInt();
      setMotor('b', constrain(speed, 0, 100)); // Backward
    } else if (command == "s") {
      setMotor('s', 0); // Stop
    } else {
      Serial.println("Invalid command!");
    }
  }

  delay(10); // Small delay for stability
}

// Function to control motor
void setMotor(char direction, int speed) {
  int pwmValue = map(speed, 0, 100, 0, 255); // Map speed (0-100) to PWM (0-255)

  switch (direction) {
    case 'f': // Forward
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      analogWrite(enA, pwmValue);
      Serial.print("Forward at speed: ");
      Serial.println(speed);
      break;

    case 'b': // Backward
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      analogWrite(enA, pwmValue);
      Serial.print("Backward at speed: ");
      Serial.println(speed);
      break;

    case 's': // Stop
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      analogWrite(enA, 0);
      Serial.println("Motor stopped.");
      break;

    default:
      Serial.println("Invalid direction!");
  }
}



