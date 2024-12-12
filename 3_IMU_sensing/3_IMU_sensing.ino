/****************************************************************
 * Example1_Basics.ino
 * ICM 20948 Arduino Library Demo
 * Use the default configuration to stream 9-axis IMU data
 * Owen Lyke @ SparkFun Electronics
 * Original Creation Date: April 17 2019
 *
 * Please see License.md for the license information.
 *
 * Distributed as-is; no warranty is given.
 ***************************************************************/
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
//#define USE_SPI       // Uncomment this to use SPI

#define SERIAL_PORT Serial

#define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 2     // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

#ifdef USE_SPI
ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object
#else
ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object
ICM_20948_I2C myICM2; // Otherwise create an ICM_20948_I2C object
//ICM_20948_I2C myICM3; // Otherwise create an ICM_20948_I2C object
#endif

Adafruit_MPU6050 mpu1;



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




void setup()
{

  SERIAL_PORT.begin(115200);
  delay(1000);
  while (!SERIAL_PORT)
  {
  };
  Wire.begin();

#ifdef USE_SPI
  SPI_PORT.begin();
#else
  WIRE_PORT.begin();
  WIRE_PORT.setClock(100000);
#endif

  myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial
  myICM2.enableDebugging();
  bool initialized1 = false;
  bool initialized2 = false;

  while (!initialized1 || !initialized2)
  {

#ifdef USE_SPI
    myICM.begin(CS_PIN, SPI_PORT);
#else
    myICM.begin(WIRE_PORT, AD0_VAL);
    myICM2.begin(WIRE_PORT, AD0_VAL);
#endif

    digitalWrite(0, HIGH);
    digitalWrite(1, LOW);
    //digitalWrite(2, LOW);
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


}

void loop()
{
  // IMU1
  digitalWrite(0, HIGH);
  digitalWrite(1, HIGH);
  //digitalWrite(2, LOW);
  if (myICM.dataReady())
  {
    myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
                             //    printRawAGMT( myICM.agmt );     // Uncomment this to see the raw values, taken directly from the agmt structure
    SERIAL_PORT.print("IMU 1: ");
    printScaledAGMT(&myICM);
    // printRawAGMT(myICM.agmt); // This function takes into account the scale settings from when the measurement was made to calculate the values with units
  }
  else
  {
    SERIAL_PORT.println("Waiting for data 1");
    delay(10);
  }

  // IMU2
  digitalWrite(0, LOW);
  digitalWrite(1, HIGH);
  //digitalWrite(2, LOW);
  if (myICM2.dataReady())
  {
    myICM2.getAGMT();         // The values are only updated when you call 'getAGMT'
                             //    printRawAGMT( myICM.agmt );     // Uncomment this to see the raw values, taken directly from the agmt structure
    SERIAL_PORT.print("IMU 2: ");
    printScaledAGMT(&myICM2); // This function takes into account the scale settings from when the measurement was made to calculate the values with units
    // printRawAGMT(myICM2.agmt);
    delay(10);
  }
  else
  {
    SERIAL_PORT.println("Waiting for data 2");
    delay(500);
  }

  digitalWrite(0, HIGH);
  digitalWrite(1, HIGH);
  sensors_event_t a1, g1, temp1;
  mpu1.getEvent(&a1, &g1, &temp1);

  Serial.print("MPU-6050 1: ");
  Serial.print("Acc X: "); Serial.print(a1.acceleration.x); 
  Serial.print(" | Acc Y: "); Serial.print(a1.acceleration.y); 
  Serial.print(" | Acc Z: "); Serial.print(a1.acceleration.z);
  Serial.print(" | Gyro X: "); Serial.print(g1.gyro.x); 
  Serial.print(" | Gyro Y: "); Serial.print(g1.gyro.y); 
  Serial.print(" | Gyro Z: "); Serial.println(g1.gyro.z);
  delay(50);
}

// Below here are some helper functions to print the data nicely!

void printPaddedInt16b(int16_t val)
{
  if (val > 0)
  {
    SERIAL_PORT.print(" ");
    if (val < 10000)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 1000)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 100)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 10)
    {
      SERIAL_PORT.print("0");
    }
  }
  else
  {
    SERIAL_PORT.print("-");
    if (abs(val) < 10000)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 1000)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 100)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 10)
    {
      SERIAL_PORT.print("0");
    }
  }
  SERIAL_PORT.print(abs(val));
}

void printRawAGMT(ICM_20948_AGMT_t agmt)
{
  SERIAL_PORT.print("RAW. Acc [ ");
  printPaddedInt16b(agmt.acc.axes.x);
  SERIAL_PORT.print(" , ");
  printPaddedInt16b(agmt.acc.axes.y);
  SERIAL_PORT.print(" , ");
  printPaddedInt16b(agmt.acc.axes.z);
  SERIAL_PORT.print(" ], Gyr [ ");
  printPaddedInt16b(agmt.gyr.axes.x);
  SERIAL_PORT.print(" , ");
  printPaddedInt16b(agmt.gyr.axes.y);
  SERIAL_PORT.print(" , ");
  printPaddedInt16b(agmt.gyr.axes.z);
  SERIAL_PORT.print(" ], Mag [ ");
  printPaddedInt16b(agmt.mag.axes.x);
  SERIAL_PORT.print(" , ");
  printPaddedInt16b(agmt.mag.axes.y);
  SERIAL_PORT.print(" , ");
  printPaddedInt16b(agmt.mag.axes.z);
  SERIAL_PORT.print(" ], Tmp [ ");
  printPaddedInt16b(agmt.tmp.val);
  SERIAL_PORT.print(" ]");
}

void printFormattedFloat(float val, uint8_t leading, uint8_t decimals)
{
  float aval = abs(val);
  if (val < 0)
  {
    SERIAL_PORT.print("-");
  }
  else
  {
    SERIAL_PORT.print(" ");
  }
  for (uint8_t indi = 0; indi < leading; indi++)
  {
    uint32_t tenpow = 0;
    if (indi < (leading - 1))
    {
      tenpow = 1;
    }
    for (uint8_t c = 0; c < (leading - 1 - indi); c++)
    {
      tenpow *= 10;
    }
    if (aval < tenpow)
    {
      SERIAL_PORT.print("0");
    }
    else
    {
      break;
    }
  }
  if (val < 0)
  {
    SERIAL_PORT.print(-val, decimals);
  }
  else
  {
    SERIAL_PORT.print(val, decimals);
  }
}

#ifdef USE_SPI
void printScaledAGMT(ICM_20948_SPI *sensor)
{
#else
void printScaledAGMT(ICM_20948_I2C *sensor)
{
#endif
  SERIAL_PORT.print("Scaled. Acc (mg)[ ");
  printFormattedFloat(sensor->accX(), 5, 2);
  SERIAL_PORT.print(" , ");
  printFormattedFloat(sensor->accY(), 5, 2);
  SERIAL_PORT.print(" , ");
  printFormattedFloat(sensor->accZ(), 5, 2);
  SERIAL_PORT.print(" ], Gyr (DPS) [ ");
  printFormattedFloat(sensor->gyrX(), 5, 2);
  SERIAL_PORT.print(" , ");
  printFormattedFloat(sensor->gyrY(), 5, 2);
  SERIAL_PORT.print(" , ");
  printFormattedFloat(sensor->gyrZ(), 5, 2);
  SERIAL_PORT.print(" ], Mag (uT)[ ");
  printFormattedFloat(sensor->magX(), 5, 2);
  SERIAL_PORT.print(" , ");
  printFormattedFloat(sensor->magY(), 5, 2);
  SERIAL_PORT.print(" , ");
  printFormattedFloat(sensor->magZ(), 5, 2);
  SERIAL_PORT.print(" ], Tmp (C)[ ");
  printFormattedFloat(sensor->temp(), 5, 2);
  SERIAL_PORT.print(" ]");
}
