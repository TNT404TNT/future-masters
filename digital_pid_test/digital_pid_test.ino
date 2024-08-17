// Sensor pin definitions
#define LEFT_MOST_SENSOR 2
#define LEFT_SENSOR 3
#define CENTER_SENSOR 4
#define RIGHT_SENSOR 5
#define RIGHT_MOST_SENSOR 6

// Motor pin definitions
#define LEFT_MOTOR_PWM 9
#define LEFT_MOTOR_DIR1 7
#define LEFT_MOTOR_DIR2 8
#define RIGHT_MOTOR_PWM 10
#define RIGHT_MOTOR_DIR1 11
#define RIGHT_MOTOR_DIR2 12

// Variables for PID control
int sensorValues[5]; // Array to hold sensor readings
int position;        // Current position of the robot relative to the line
int lastError = 0;   // Previous error value for derivative calculation
int integral = 0;    // Accumulated integral for PID

// PID constants (tune these values)
float Kp = 0.6; // Proportional gain
float Ki = 0.0; // Integral gain
float Kd = 0.4; // Derivative gain

// Motor speed control
int maxSpeed = 200;  // Maximum PWM speed (0-255)
int baseSpeed = 150; // Base PWM speed (0-255)

void setup()
{
  // Set sensor pins as input
  pinMode(LEFT_MOST_SENSOR, INPUT);
  pinMode(LEFT_SENSOR, INPUT);
  pinMode(CENTER_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR, INPUT);
  pinMode(RIGHT_MOST_SENSOR, INPUT);

  // Set motor control pins as output
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(LEFT_MOTOR_DIR1, OUTPUT);
  pinMode(LEFT_MOTOR_DIR2, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR1, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR2, OUTPUT);
}

void loop()
{
  // Read the sensor values
  readSensors();

  // Calculate the error based on sensor readings
  int error = calculateError();

  // Calculate the correction value using PID
  int correction = calculatePID(error);

  // Determine motor speeds based on the correction value
  int leftMotorSpeed = baseSpeed + correction;
  int rightMotorSpeed = baseSpeed - correction;

  // Constrain motor speeds to within maximum limits
  leftMotorSpeed = constrain(leftMotorSpeed, 0, maxSpeed);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, maxSpeed);

  // Set the motor speeds
  setMotorSpeed(leftMotorSpeed, rightMotorSpeed);
}

// Function to read sensor values
void readSensors()
{
  sensorValues[0] = digitalRead(LEFT_MOST_SENSOR);
  sensorValues[1] = digitalRead(LEFT_SENSOR);
  sensorValues[2] = digitalRead(CENTER_SENSOR);
  sensorValues[3] = digitalRead(RIGHT_SENSOR);
  sensorValues[4] = digitalRead(RIGHT_MOST_SENSOR);
}

// Function to calculate the error based on sensor readings
int calculateError()
{
  int error = 0;
  int activeSensors = 0;

  // Check each sensor and adjust the error value accordingly
  if (sensorValues[0] == HIGH)
  {
    error -= 2;
    activeSensors++;
  }
  if (sensorValues[1] == HIGH)
  {
    error -= 1;
    activeSensors++;
  }
  if (sensorValues[2] == HIGH)
  {
    activeSensors++; // Center sensor has no error value (0)
  }
  if (sensorValues[3] == HIGH)
  {
    error += 1;
    activeSensors++;
  }
  if (sensorValues[4] == HIGH)
  {
    error += 2;
    activeSensors++;
  }

  // If no sensors are detecting the line, use the last error value
  if (activeSensors == 0)
  {
    return lastError;
  }

  // Average the error based on the number of active sensors
  error /= activeSensors;
  lastError = error; // Save the current error for the next loop
  return error;
}

// Function to calculate the PID correction
int calculatePID(int error)
{
  integral += error;                  // Accumulate the integral
  int derivative = error - lastError; // Calculate the derivative

  // Calculate the correction using the PID formula
  int correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
  return correction;
}

// Function to set motor speeds using PWM
void setMotorSpeed(int leftSpeed, int rightSpeed)
{
  // Set direction and speed for the left motor
  if (leftSpeed >= 0)
  {
    digitalWrite(LEFT_MOTOR_DIR1, HIGH);
    digitalWrite(LEFT_MOTOR_DIR2, LOW);
  }
  else
  {
    digitalWrite(LEFT_MOTOR_DIR1, LOW);
    digitalWrite(LEFT_MOTOR_DIR2, HIGH);
    leftSpeed = -leftSpeed; // Convert to positive for PWM
  }
  analogWrite(LEFT_MOTOR_PWM, leftSpeed);

  // Set direction and speed for the right motor
  if (rightSpeed >= 0)
  {
    digitalWrite(RIGHT_MOTOR_DIR1, HIGH);
    digitalWrite(RIGHT_MOTOR_DIR2, LOW);
  }
  else
  {
    digitalWrite(RIGHT_MOTOR_DIR1, LOW);
    digitalWrite(RIGHT_MOTOR_DIR2, HIGH);
    rightSpeed = -rightSpeed; // Convert to positive for PWM
  }
  analogWrite(RIGHT_MOTOR_PWM, rightSpeed);
}
