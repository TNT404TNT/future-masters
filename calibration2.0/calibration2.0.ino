#define NUM_SENSORS 5
#define readings 5000
const int SENSOR_PINS[NUM_SENSORS] = { A1, A2, A3, A4, A5 };
int sensorMin[NUM_SENSORS] = { 1023, 1023, 1023, 1023, 1023 };
int sensorMax[NUM_SENSORS] = { 0, 0, 0, 0, 0 };

void setup() {

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    pinMode(SENSOR_PINS[i], INPUT);
  }

  calibrateSensors();

  Serial.begin(9600);
}

void loop() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    Serial.println(readCalibrated(i));
  }
}

void calibrateSensors() {
  Serial.println("Starting calibration...");

  // Calibrate for 5 seconds
  for (int i = 0; i < readings; i++) {
    for (int j = 0; j < NUM_SENSORS; j++) {
      int value = analogRead(SENSOR_PINS[j]);
      sensorMin[j] = min(sensorMin[j], value);
      sensorMax[j] = max(sensorMax[j], value);
    }
    delay(1);
  }

  Serial.println("Calibration complete");
}

int readCalibrated(int sensorIndex) {
  int raw = analogRead(SENSOR_PINS[sensorIndex]);
  return map(raw, sensorMin[sensorIndex], sensorMax[sensorIndex], 0, 1023);
}