#include <MAX30105.h>
#include <heartRate.h>
#include <spo2_algorithm.h>

// Initialize MAX30105 sensor
MAX30105 particleSensor;

// Variables for SpO2 and Heart Rate
uint32_t irBuffer[100];      // Infrared LED sensor data
uint32_t redBuffer[100];     // Red LED sensor data
int32_t bufferLength = 100;  // Number of samples in each reading

int32_t heartRate;           // Heart rate value
int32_t spo2;                // SpO2 value
int8_t validHeartRate;       // Indicator for valid heart rate
int8_t validSpO2;            // Indicator for valid SpO2

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);

  // Initialize the MAX30105 sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("Sensor error! Check wiring.");
    while (1); // Loop forever if sensor is not found
  }

  particleSensor.setup();     // Configure sensor with default settings
  Serial.println("Sensor initialized. Starting readings...");
}

void loop() {
  // Read data from the sensor into buffers
  for (int i = 0; i < bufferLength; i++) {
    while (!particleSensor.available()) // Wait until sensor has new data
      particleSensor.check();           // Check for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();        // Move to the next sample
  }

  // Calculate heart rate and SpO2 using algorithms
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSpO2, &heartRate, &validHeartRate);

  // Read temperature from the sensor
  float temperature = particleSensor.readTemperature();

  // Print results to the Serial Monitor
  if (validHeartRate) {
    Serial.print("Heart Rate: ");
    Serial.print(heartRate);
    Serial.print(" bpm");
  } else {
    Serial.print("Heart Rate: Invalid");
  }

  Serial.print(" | ");

  if (validSpO2) {
    Serial.print("SpO2: ");
    Serial.print(spo2);
    Serial.print("%");
  } else {
    Serial.print("SpO2: Invalid");
  }

  Serial.print(" | ");

  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");

  delay(1000); // Delay for readability
}