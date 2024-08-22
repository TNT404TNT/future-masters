#include <EEPROM.h>

// Pin definitions
const int sensorPins[7] = {A0, A1, A2, A3, A4, A5, A6};
const int leftMotorPin1 = 5;
const int leftMotorPin2 = 6;
const int rightMotorPin1 = 9;
const int rightMotorPin2 = 10;

// PID constants (you'll need to tune these)
float Kp = 1.0;
float Ki = 0.0;
float Kd = 0.0;

// PID time constants (in milliseconds)
const float Ti = 1000.0;  // Integral time
const float Td = 100.0;   // Derivative time

// PID variables
volatile float error = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;

// time variables
unsigned long currentTime = 0;
unsigned long previousTime = 0;
float elapsedTime = 0;

// Motor speed variables
int baseSpeed = 150;
int maxSpeed = 255;

void setup() {
  Serial.begin(9600);

  // Set motor pins as outputs
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  // Set sensor pins as inputs
  for (int i = 0; i < 7; i++)
  {
      pinMode(sensorPins[i], INPUT);
  }

}

void loop() {
  // Calculate time elapsed since last loop
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0;  // Convert to seconds
  
  // Read sensors and calculate error
  error = calculateError();

  // Calculate PID components
  integral += (error * deltaTime) / Ti;
  integral = constrain(integral, -100, 100);  // Prevent integral windup
  
  derivative = (Td / deltaTime) * (error - lastError);
  
  // Calculate PID output
  float pidOutput = Kp * error + Ki * integral + Kd * derivative;
  
  // Apply motor speeds
  leftSpeed = constrain(leftSpeed, 0, maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, maxSpeed);
  setMotorSpeeds(leftSpeed, rightSpeed);
  
  // Save current error and time for next iteration
  lastError = error;
  previousTime = currentTime;
  
  // Optional: Print debug information
  printDebugInfo(proportional, integral, derivative);
  
  delay(10);  // Short delay for stability ------------------------------ is that actualy needed
}

float calculateError() {
  int sensorValues[7];
  long sum = 0;
  long weightedSum = 0;
  
  // Read sensor values
  for (int i = 0; i < 7; i++) {
    sensorValues[i] = analogRead(SENSOR_PINS[i]);
    sum += sensorValues[i];
    weightedSum += (long)sensorValues[i] * (i-3) * 1000;
  }
  
  // Calculate error (-3 to 3, with 0 being centered) --------------------------- idek how this works 
  return 2.0 * ((float)weightedSum / sum / 1000.0 - (7 - 1) / 2.0) / (7 - 1);
}

void setMotorSpeeds(int leftSpeed, int rightSpeed)
{
    analogWrite(leftMotorPin1, leftSpeed);
    analogWrite(leftMotorPin2, 0);
    analogWrite(rightMotorPin1, rightSpeed);
    analogWrite(rightMotorPin2, 0);
}

void printDebugInfo(float proportional, float integral, float derivative) {
  Serial.print("Error: ");
  Serial.print(error);
  Serial.print("\tP: ");
  Serial.print(proportional);
  Serial.print("\tI: ");
  Serial.print(integral);
  Serial.print("\tD: ");
  Serial.print(derivative);
  Serial.print("\tLeft Speed: ");
  Serial.print(analogRead(LEFT_MOTOR_PIN));
  Serial.print("\tRight Speed: ");
  Serial.println(analogRead(RIGHT_MOTOR_PIN));
}