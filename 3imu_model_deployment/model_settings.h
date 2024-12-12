#ifndef MODEL_SETTINGS_H_
#define MODEL_SETTINGS_H_

// All of these values are derived from the values used during model training,
// if you change your model you'll need to update these constants.

// Define the number of features based on your IMU data setup
constexpr int kNumFeatures = 24; // Adjust if different

// Define the sequence length based on your model training data
constexpr int kSequenceLength = 10;

constexpr int kInputSize = kNumFeatures * kSequenceLength;

constexpr int kCategoryCount = 2;
constexpr int kActionIndex = 1;
constexpr int kNoActionIndex = 0;
extern const char* kCategoryLabels[kCategoryCount];

#endif  // MODEL_SETTINGS_H_