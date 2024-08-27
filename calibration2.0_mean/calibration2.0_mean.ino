#define NUM_SENSORS 5
#define readings 5000
const int SENSOR_PINS[NUM_SENSORS] = { A1, A2, A3, A4, A5 };
int sensorSum[NUM_SENSORS] = { 0, 0, 0, 0, 0 };
int sensorMean[NUM_SENSORS];

void setup() {

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    pinMode(SENSOR_PINS[i], INPUT);
  }

  calibrateSensors();

  Serial.begin(9600);
}

void loop() {
  /*im not sure what should be here so please help*/
}

void calibrateSensors() {
  Serial.println("Starting calibration...");

  // Calibrate for 5 seconds
  for (int i = 0; i < readings; i++) {
    for (int j = 0; j < NUM_SENSORS; j++) {
      sensorSum[j] += analogRead(SENSOR_PINS[j]);
    }
    delay(1);
  }

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    sensorMean[i] = sensorSum[i] / readings;
  }


  Serial.println("Calibration complete");
}

void readCalibrated() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    int raw = analogRead(SENSOR_PINS[i]); /*also please fix this*/
  }
}