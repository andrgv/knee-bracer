#include "ICM_20948.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TensorFlowLite.h>

#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

#ifndef MAIN_FUNCTIONS_H_
#define MAIN_FUNCTIONS_H_

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "tensorflow/lite/c/common.h"  // Ensure TfLiteStatus is declared

// Expose a C friendly interface for main functions.
#ifdef __cplusplus
extern "C" {
#endif

void setup();
void loop();
void scanI2CBus();
float scaleInput(float input, float mean_value, float std_dev);
// void initializeIMUs();
TfLiteStatus GetIMUData(tflite::ErrorReporter* error_reporter, int8_t* data);

#ifdef __cplusplus
}
#endif

#endif  // MAIN_FUNCTIONS_H_
