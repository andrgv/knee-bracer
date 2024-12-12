#include "ICM_20948.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TensorFlowLite.h>

#include "main_functions.h"
#include "detection_responder.h"
#include "model_settings.h"
#include "knee_detect_model_data.h"  // Include the generated model data header
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

#define SERIAL_PORT Serial

#define WIRE_PORT Wire
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

ICM_20948_I2C myICM;
ICM_20948_I2C myICM2;
Adafruit_MPU6050 mpu1;

// float mean_value = 100.0;
// float std_dev = 50.0;

namespace {
  tflite::ErrorReporter* error_reporter = nullptr;
  const tflite::Model* model = nullptr;
  tflite::MicroInterpreter* interpreter = nullptr;
  TfLiteTensor* input = nullptr;
  constexpr int kTensorArenaSize = 80 * 1024;
  static uint8_t tensor_arena[kTensorArenaSize];
}

float scaleInput(float input, float mean_value, float std_dev) {
    return (input - mean_value) / std_dev;
}

void scanI2CBus() {
  Serial.println("Scanning I2C bus...");
  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found I2C device at address 0x");
      Serial.println(address, HEX);
      delay(10);
    }
  }
  Serial.println("Scan complete.");
}

TfLiteStatus GetIMUData(tflite::ErrorReporter* error_reporter, int8_t* data) {
  // first 9-axis imu
  // IMU1
  digitalWrite(0, HIGH);
  digitalWrite(1, LOW);
  if (myICM.dataReady())
  {
    myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
    // data[0] = static_cast<int8_t>(scaleInput(myICM.agmt.acc.axes.x, -918.3729855599648, 712.7876116336696));
    // data[1] = static_cast<int8_t>(scaleInput(myICM.agmt.acc.axes.y, -1324.5650209435628, 1312.3667604693424));
    // data[2] = static_cast<int8_t>(scaleInput(myICM.agmt.acc.axes.z, -1048.5565542328043, 1284.0994242487907));
    // data[3] = static_cast<int8_t>(scaleInput(myICM.agmt.gyr.axes.x, -1389.8803830467375, 1486.8277866703295));
    // data[4] = static_cast<int8_t>(scaleInput(myICM.agmt.gyr.axes.y, -1037.6879508377426, 1294.8867321382425));
    // data[5] = static_cast<int8_t>(scaleInput(myICM.agmt.gyr.axes.z, -1404.4687373236331, 1315.4601833839204));
    // data[6] = static_cast<int8_t>(scaleInput(myICM.agmt.mag.axes.x, -435.87374614197535, 1189.2772330672813));
    // data[7] = static_cast<int8_t>(scaleInput(myICM.agmt.mag.axes.y, -112.00339175485009, 1333.6872919307386));
    // data[8] = static_cast<int8_t>(scaleInput(myICM.agmt.mag.axes.z, -719.5155186287478, 1063.1199979551932));
    data[0] = static_cast<int8_t>(myICM.agmt.acc.axes.x);
    data[1] = static_cast<int8_t>(myICM.agmt.acc.axes.y);
    data[2] = static_cast<int8_t>(myICM.agmt.acc.axes.z);
    data[3] = static_cast<int8_t>(myICM.agmt.gyr.axes.x);
    data[4] = static_cast<int8_t>(myICM.agmt.gyr.axes.y);
    data[5] = static_cast<int8_t>(myICM.agmt.gyr.axes.z);
    data[6] = static_cast<int8_t>(myICM.agmt.mag.axes.x);
    data[7] = static_cast<int8_t>(myICM.agmt.mag.axes.y);
    data[8] = static_cast<int8_t>(myICM.agmt.mag.axes.z);
    delay(10);
  }
  else
  {
    SERIAL_PORT.println("Waiting for data 1");
    delay(10);
  }

  // IMU2
  digitalWrite(0, LOW);
  digitalWrite(1, HIGH);
  if (myICM2.dataReady())
  {
    myICM2.getAGMT();         // The values are only updated when you call 'getAGMT'
    // data[9] = static_cast<int8_t>(scaleInput(myICM2.agmt.acc.axes.x, -1387.445437059083, 1044.6432709054209));
    // data[10] = static_cast<int8_t>(scaleInput(myICM2.agmt.acc.axes.y, -896.6922530864198, 905.0048843497381));
    // data[11] = static_cast<int8_t>(scaleInput(myICM2.agmt.acc.axes.z, -375.08307760141093, 1195.12627668347));
    // data[12] = static_cast<int8_t>(scaleInput(myICM2.agmt.gyr.axes.x, -735.7913458994709, 1078.8032416465594));
    // data[13] = static_cast<int8_t>(scaleInput(myICM2.agmt.gyr.axes.y, -2318.806517857143, 1328.4820793060417));
    // data[14] = static_cast<int8_t>(scaleInput(myICM2.agmt.gyr.axes.z, -1240.3674327601411, 959.7441422702027));
    // data[15] = static_cast<int8_t>(scaleInput(myICM2.agmt.mag.axes.x, -1351.8139445546738, 1316.8114737714025));
    // data[16] = static_cast<int8_t>(scaleInput(myICM2.agmt.mag.axes.y, -550.8138222001764, 1289.466396064897));
    // data[17] = static_cast<int8_t>(scaleInput(myICM2.agmt.mag.axes.z, -1128.4108581349208, 1489.7779146337855));
    data[0] = static_cast<int8_t>(myICM2.agmt.acc.axes.x);
    data[1] = static_cast<int8_t>(myICM2.agmt.acc.axes.y);
    data[2] = static_cast<int8_t>(myICM2.agmt.acc.axes.z);
    data[3] = static_cast<int8_t>(myICM2.agmt.gyr.axes.x);
    data[4] = static_cast<int8_t>(myICM2.agmt.gyr.axes.y);
    data[5] = static_cast<int8_t>(myICM2.agmt.gyr.axes.z);
    data[6] = static_cast<int8_t>(myICM2.agmt.mag.axes.x);
    data[7] = static_cast<int8_t>(myICM2.agmt.mag.axes.y);
    data[8] = static_cast<int8_t>(myICM2.agmt.mag.axes.z);
    delay(10);
  }
  else
  {
    SERIAL_PORT.println("Waiting for data 2");
    delay(10);
  }
  
  // Read data from the first MPU-6050 sensor
  sensors_event_t a1, g1, temp1;
  mpu1.getEvent(&a1, &g1, &temp1);

  // Fill input tensor with IMU data (example using accelerometer and gyroscope data)
  // data[18] = static_cast<int8_t>(scaleInput(a1.acceleration.x, -1027.5913183421517, 1295.8785283520394));
  // data[19] = static_cast<int8_t>(scaleInput(a1.acceleration.y, -1415.3769505070547, 1315.2706888754694));
  // data[20] = static_cast<int8_t>(scaleInput(a1.acceleration.z, -413.8660157627866, 1045.591771424207));
  // data[21] = static_cast<int8_t>(scaleInput(g1.gyro.x, -112.10788525132277, 1029.1135002284143));
  // data[22] = static_cast<int8_t>(scaleInput(g1.gyro.y, -1291.448477733686, 849.5379457053515));
  // data[23] = static_cast<int8_t>(scaleInput(g1.gyro.z, -1714.4688855820104, 872.992594031797));
  data[18] = static_cast<int8_t>(a1.acceleration.x);
  data[19] = static_cast<int8_t>(a1.acceleration.y);
  data[20] = static_cast<int8_t>(a1.acceleration.z);
  data[21] = static_cast<int8_t>(g1.gyro.x);
  data[22] = static_cast<int8_t>(g1.gyro.y);
  data[23] = static_cast<int8_t>(g1.gyro.z);


  return kTfLiteOk;
}

void setup() {
  SERIAL_PORT.begin(115200);
  delay(1000);
  while (!SERIAL_PORT)
  {
  };
  Wire.begin();

  WIRE_PORT.begin();
  WIRE_PORT.setClock(100000);

  myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial
  myICM2.enableDebugging();
  bool initialized1 = false;
  bool initialized2 = false;

  while (!initialized1 || !initialized2)
  {

    myICM.begin(WIRE_PORT, AD0_VAL);
    myICM2.begin(WIRE_PORT, AD0_VAL);

    digitalWrite(0, HIGH);
    digitalWrite(1, LOW);
    SERIAL_PORT.print(F("Initialization of the sensor 1 returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized1 = true;
      delay(10);
    }

    digitalWrite(0, LOW);
    digitalWrite(1, HIGH);
    SERIAL_PORT.print(F("Initialization of the sensor 2 returned: "));
    SERIAL_PORT.println(myICM2.statusString());
    if (myICM2.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized2 = true;
      delay(10);
    }
  }


  digitalWrite(0, HIGH);
  digitalWrite(1, HIGH);

  scanI2CBus();

  if (!mpu1.begin(0x68)) {
    Serial.println("Could not find MPU-6050 sensor at address 0x68");
    while (1);

  }

  mpu1.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu1.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu1.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("MPU-6050 initialized successfully");

  // Set up TensorFlow Lite
  static tflite::MicroErrorReporter micro_error_reporter;
  error_reporter = &micro_error_reporter;

  model = tflite::GetModel(g_imu_detect_model_data);  // Ensure this matches your model variable name
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    TF_LITE_REPORT_ERROR(error_reporter,
                         "Model provided is schema version %d not equal "
                         "to supported version %d.",
                         model->version(), TFLITE_SCHEMA_VERSION);
    return;
  }

  static tflite::MicroMutableOpResolver<5> micro_op_resolver;
  micro_op_resolver.AddFullyConnected(); 
  micro_op_resolver.AddReshape();
  micro_op_resolver.AddSoftmax();

  static tflite::MicroInterpreter static_interpreter(
      model, micro_op_resolver, tensor_arena, kTensorArenaSize, error_reporter);
  interpreter = &static_interpreter;

  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk) {
    TF_LITE_REPORT_ERROR(error_reporter, "AllocateTensors() failed");
    return;
  }

  input = interpreter->input(0);
}

void loop() {
  if (GetIMUData(error_reporter, input->data.int8) != kTfLiteOk) {
    TF_LITE_REPORT_ERROR(error_reporter, "IMU data capture failed.");
    return;
  }

  // Print the array directly in the loop for debugging purposes
  for (size_t i = 0; i < input->bytes; i++) { 
    Serial.print(input->data.int8[i]); 
    if (i < input->bytes - 1) { Serial.print(", ");
    } 
  } 
  Serial.println();

  if (kTfLiteOk != interpreter->Invoke()) {
    TF_LITE_REPORT_ERROR(error_reporter, "Invoke failed.");
    return;
  }

  TfLiteTensor* output = interpreter->output(0);
  // Scores for both categories
  // int8_t category_0_score = output->data.int8[0];
  // int8_t category_1_score = output->data.int8[1];
  int8_t category_0_score = output->data.int8[0];
  int8_t category_1_score = output->data.int8[1];

  // Determine which category has the higher score
  Serial.print("Score for incorrect: ");
  Serial.print(category_0_score);
  Serial.print(". Score for correct: ");
  Serial.println(category_1_score);
  int action_score = (category_0_score > category_1_score) ? 0 : 1;
  RespondToDetection(error_reporter, action_score);

  delay(1000);
}
