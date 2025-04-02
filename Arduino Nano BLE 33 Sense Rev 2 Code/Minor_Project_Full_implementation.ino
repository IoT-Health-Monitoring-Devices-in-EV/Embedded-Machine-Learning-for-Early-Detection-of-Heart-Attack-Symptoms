#include <Wire.h>
#include <TensorFlowLite.h>
#include <tensorflow/lite/micro/tflite_bridge/micro_error_reporter.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/micro/all_ops_resolver.h>
#include <tensorflow/lite/schema/schema_generated.h>
#include <MAX30105.h>
#include <heartRate.h>
#include <spo2_algorithm.h>
#include <SparkFun_TMP117.h>
#include "model_data.h"  // Include trained TFLite model

// Sensor Initialization
MAX30105 particleSensor;
TMP117 tempSensor;
const int GSR_PIN = A0;

// TensorFlow Lite Variables
constexpr int tensorArenaSize = 20 * 1024;
uint8_t tensorArena[tensorArenaSize];

tflite::MicroErrorReporter errorReporter;
tflite::AllOpsResolver resolver;
const tflite::Model* model;
tflite::MicroInterpreter* interpreter;

void setup() {
    Serial.begin(115200);
    Wire.begin();

    // Initialize MAX30102 Sensor
    if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
        Serial.println("ERROR: MAX30102 Sensor Not Detected!");
        while (1);
    }
    particleSensor.setup();

    // Initialize TMP117 Sensor
    if (!tempSensor.begin()) {
        Serial.println("ERROR: TMP117 Sensor Not Detected!");
        while (1);
    }

    // Load TFLite Model
    model = tflite::GetModel(tflite_model);
    if (model->version() > TFLITE_SCHEMA_VERSION) {
        Serial.println("ERROR: Model schema version mismatch!");
        while (1);
    }

    static tflite::MicroInterpreter staticInterpreter(model, resolver, tensorArena, tensorArenaSize);
    interpreter = &staticInterpreter;

    if (interpreter->AllocateTensors() != kTfLiteOk) {
        Serial.println("ERROR: Tensor allocation failed!");
        while (1);
    }
    Serial.println("TFLite Model Loaded Successfully.");
}

void loop() {
    uint32_t irBuffer[100], redBuffer[100];
    int32_t bufferLength = 100;
    int32_t heartRate = 0, spo2 = 0;
    int8_t validHeartRate = 0, validSpO2 = 0;

    // Read MAX30102 Sensor Data
    for (int i = 0; i < bufferLength; i++) {
        while (!particleSensor.available()) particleSensor.check();
        redBuffer[i] = particleSensor.getRed();
        irBuffer[i] = particleSensor.getIR();
        particleSensor.nextSample();
    }

    // Detect Finger Placement
    long irSignal = irBuffer[bufferLength - 1];
    bool fingerDetected = (irSignal > 5000);

    if (fingerDetected) {
        maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSpO2, &heartRate, &validHeartRate);
    } else {
        heartRate = 0;
        spo2 = 0;
    }

    // Read TMP117 Temperature Sensor
    float temperature = fingerDetected ? tempSensor.readTempC() : 0;
    bool tempValid = (fingerDetected && !(isnan(temperature) || temperature < 10.0 || temperature > 50.0));
    if (!tempValid) temperature = 0;

    // Read GSR Sensor and Convert to Conductance
    int rawGsrValue = analogRead(GSR_PIN);
    float gsrValue = (rawGsrValue >= 1000 || rawGsrValue == 0) ? 0.0 : (1.0 / ((3.3 * 1000000.0 / ((float)rawGsrValue * (3.3 / 1023))) - 1000000.0)) * 1000000.0;

    // Prepare TensorFlow Lite Input
    TfLiteTensor* input = interpreter->input(0);
    if (input->type == kTfLiteFloat32 && input->dims->size == 2 && input->dims->data[1] == 4) {
        input->data.f[0] = (float)heartRate;
        input->data.f[1] = (float)spo2;
        input->data.f[2] = temperature;
        input->data.f[3] = gsrValue;
    } else {
        Serial.println("ERROR: Input tensor type mismatch!");
        return;
    }

    // Run Inference
    if (interpreter->Invoke() != kTfLiteOk) {
        Serial.println("ERROR: TFLite inference failed!");
        return;
    }

    // Get Output Tensor
    TfLiteTensor* output = interpreter->output(0);
    float heartAttackRisk = fingerDetected ? output->data.f[0] : 0.0;

    // Print Results
    Serial.print("HR: "); Serial.print(validHeartRate ? heartRate : 0);
    Serial.print(" bpm | SpO2: "); Serial.print(validSpO2 ? spo2 : 0);
    Serial.print("% | Temp: "); Serial.print(tempValid ? temperature : 0);
    Serial.print(" °C | GSR: "); Serial.print(gsrValue);
    Serial.print(" | Heart Attack Risk: "); Serial.println(heartAttackRisk);

    delay(5000);
}