// Define the analog pin numbers for the 5 IR sensors
const int sensorPins[5] = {A0, A1, A2, A3, A4};

// Variables to store the minimum and maximum values for each sensor
int sensorMin[5];
int sensorMax[5];

// Calibration settings
const int numReadings = 100; // Number of readings to take for calibration
const int calibratedRangeMin = 0; // Minimum value for the calibrated range
const int calibratedRangeMax = 1023; // Maximum value for the calibrated range

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Initialize sensorMin and sensorMax arrays
  for (int i = 0; i < 5; i++) {
    sensorMin[i] = 1023; // Set initial min to maximum possible value
    sensorMax[i] = 0;    // Set initial max to minimum possible value
  }
  
  // Perform calibration
  calibrateSensors();
  
  // After calibration, print the calibration results
  printCalibrationResults();
}

void loop() {
  // Read and print calibrated sensor values
  for (int i = 0; i < 5; i++) {
    int rawValue = analogRead(sensorPins[i]);
    int calibratedValue = map(rawValue, sensorMin[i], sensorMax[i], calibratedRangeMin, calibratedRangeMax);
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(calibratedValue);
  }
  
  delay(500); // Delay for readability
}

void calibrateSensors() {
  Serial.println("Calibrating sensors...");


  // Take readings and find min/max for each sensor
  for (int i = 0; i < numReadings * 5; i++) {
    for (int j = 0; j < 5; j++) {
      int value = analogRead(sensorPins[j]);
      if (value < sensorMin[j]) sensorMin[j] = value;
      if (value > sensorMax[j]) sensorMax[j] = value;
      Serial.print("reading sensor ");
      Serial.print(j);
      delay(100); // Small delay between readings
    }
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(" - Min: ");
    Serial.print(sensorMin[i]);
    Serial.print(", Max: ");
    Serial.println(sensorMax[i]);
  }
}

void printCalibrationResults() {
  Serial.println("Calibration results:");
  for (int i = 0; i < 5; i++) {
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(" - Min: ");
    Serial.print(sensorMin[i]);
    Serial.print(", Max: ");
    Serial.println(sensorMax[i]);
  }
}
