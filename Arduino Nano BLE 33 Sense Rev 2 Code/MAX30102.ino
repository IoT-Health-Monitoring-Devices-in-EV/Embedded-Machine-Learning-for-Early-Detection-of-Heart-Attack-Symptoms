#include <MAX30105.h>
#include <heartRate.h>
#include <spo2_algorithm.h>
#include <SparkFun_TMP117.h>
#include <SparkFun_TMP117_Registers.h>

// Initialize MAX30105 sensor
MAX30105 particleSensor;

// Initialize TMP117 sensor
TMP117 sensor;

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

  // Initialize MAX30105 sensor
  if (!particleSensor.begin()) {
    Serial.println("Sensor error! Check wiring.");
    while (1); // Loop forever if sensor is not found
  }
  particleSensor.setup(); // Configure sensor with default settings
  Serial.println("MAX30105 Initialized.");

  // Initialize TMP117 sensor
  if (!sensor.begin()) {
    Serial.println("TMP117 not detected! Check wiring.");
    while (1);
  }
  Serial.println("TMP117 Initialized. Starting readings...");
}

void loop() {
  // Read data from the MAX30105 sensor into buffers
  for (int i = 0; i < bufferLength; i++) {
    while (!particleSensor.available()) // Wait until sensor has new data
      particleSensor.check();           // Check for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();        // Move to the next sample
  }

  // Calculate heart rate and SpO2 using algorithms
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSpO2, &heartRate, &validHeartRate);

  // Read temperature from TMP117 sensor
  float temperature = sensor.readTempC();

  // Print results to the Serial Monitor
  Serial.print("Heart Rate: ");
  if (validHeartRate) Serial.print(heartRate);
  else Serial.print("Invalid");

  Serial.print(" bpm | SpO2: ");
  if (validSpO2) Serial.print(spo2);
  else Serial.print("Invalid");

  Serial.print("% | Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");

  delay(1000); // Delay for readability
}