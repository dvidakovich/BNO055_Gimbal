#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 myIMU = Adafruit_BNO055(); // create an instance of the BNO055 library

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  myIMU.begin(); // turn on the IMU
  delay(1000); // always good to have a delay after turning something on
  int8_t temp=myIMU.getTemp(); // this is an easy way to store a number from -120 to 120.  This is a very compact data type.
  Serial.println(temp);
  myIMU.setExtCrystalUse(true);  // for better results we don't want to use the timing crystal on the chip itself we want to use the timer on the board
}

void loop() {
  // put your main code here, to run repeatedly:
  imu::Vector<3> acc =myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro =myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> mag =myIMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  Serial.print(acc.x());
  Serial.print(",");
  Serial.print(acc.y());
  Serial.print(",");
  Serial.print(acc.z());
  Serial.print(",");

  Serial.print(gyro.x());
  Serial.print(",");
  Serial.print(gyro.y());
  Serial.print(",");
  Serial.print(gyro.z());
  Serial.print(",");

  Serial.print(mag.x());
  Serial.print(",");
  Serial.print(mag.y());
  Serial.print(",");
  Serial.println(mag.z());

  delay(BNO055_SAMPLERATE_DELAY_MS);

}
