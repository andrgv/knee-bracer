#ifndef DETECTION_RESPONDER_H_
#define DETECTION_RESPONDER_H_

#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"


void RespondToDetection(tflite::ErrorReporter* error_reporter,
                        int8_t action_score);

#endif  // DETECTION_RESPONDER_H_
