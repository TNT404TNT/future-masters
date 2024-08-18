// Pin definitions
const int NUM_SENSORS = 7;
const int sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6};
const int leftMotorPin1 = 5;
const int leftMotorPin2 = 6;
const int rightMotorPin1 = 9;
const int rightMotorPin2 = 10;

// PID constants
float Kp = 1.0;
float Ki = 0.0;
float Kd = 0.0;

// PID variables
float error = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;

// Motor speed variables
int baseSpeed = 150;
int maxSpeed = 255;

void setup()
{
    Serial.begin(9600);

    // Set motor pins as outputs
    pinMode(leftMotorPin1, OUTPUT);
    pinMode(leftMotorPin2, OUTPUT);
    pinMode(rightMotorPin1, OUTPUT);
    pinMode(rightMotorPin2, OUTPUT);

    // Set sensor pins as inputs
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        pinMode(sensorPins[i], INPUT);
    }
}

void loop()
{
    // Read sensors and calculate error
    error = calculateError();

    // Calculate PID values
    integral += error;
    derivative = error - lastError;

    // Calculate motor speed adjustment
    float adjustment = Kp * error + Ki * integral + Kd * derivative;

    // Set motor speeds
    int leftSpeed = baseSpeed - adjustment;
    int rightSpeed = baseSpeed + adjustment;

    // Constrain motor speeds
    leftSpeed = constrain(leftSpeed, 0, maxSpeed);
    rightSpeed = constrain(rightSpeed, 0, maxSpeed);

    // Apply motor speeds
    setMotorSpeeds(leftSpeed, rightSpeed);

    // Update last error
    lastError = error;

    // Optional: Print debug information
    printDebugInfo();

    delay(10); // Short delay for stability
}

float calculateError()
{
    int sensorValues[NUM_SENSORS];
    int sum = 0;
    int weightedSum = 0;

    for (int i = 0; i < NUM_SENSORS; i++)
    {
        sensorValues[i] = analogRead(sensorPins[i]);
        sum += sensorValues[i];
        weightedSum += sensorValues[i] * (i - 3); // Center sensor has weight 0
    }

    if (sum > 0)
    {
        return (float)weightedSum / sum;
    }
    else
    {
        return 0; // All sensors reading black (off the line)
    }
}

void setMotorSpeeds(int leftSpeed, int rightSpeed)
{
    analogWrite(leftMotorPin1, leftSpeed);
    analogWrite(leftMotorPin2, 0);
    analogWrite(rightMotorPin1, rightSpeed);
    analogWrite(rightMotorPin2, 0);
}

void printDebugInfo()
{
    Serial.print("Error: ");
    Serial.print(error);
    Serial.print("\tAdjustment: ");
    Serial.print(Kp * error + Ki * integral + Kd * derivative);
    Serial.println();
}