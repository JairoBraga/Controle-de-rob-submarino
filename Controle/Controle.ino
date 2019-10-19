#include "I2Cdev.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 accelgyro;
const int g_rate = 131;
const int a_rate = 16384;
int16_t ax, ay, az;
int16_t gx, gy, gz;
void setup() {
  // put your setup code here, to run once:

  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

     Serial.begin(38400);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    calibrateMPU();
}

void loop() {
  // put your main code here, to run repeatedly:
  
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Serial.print("a/g:\t");
  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.print(az); Serial.print("\t");
  Serial.print(gx/g_rate); Serial.print("\t");
  Serial.print(gy/g_rate); Serial.print("\t");
  Serial.println(gz/g_rate);
  delay(100);
}

void calibrateMPU(){
  accelgyro.CalibrateGyro(6);
  accelgyro.CalibrateAccel(6);
  Serial.println("Inciando calibração");
  accelgyro.PrintActiveOffsets();
}
