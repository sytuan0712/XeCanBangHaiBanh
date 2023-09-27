#include <Wire.h>
#include "Kalman.h"
#define RESTRICT_PITCH
Kalman kalmanX;
Kalman kalmanY;
int duong1 = 6;  
int am1 = 9;     
int duong2 = 10; 
int am2 = 11;
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
int16_t tempRaw;
float gyroXangle, gyroYangle;
float compAngleX, compAngleY;
float kalAngleX, kalAngleY;
uint32_t timer;
uint8_t i2cData[14];
float x, a, loi , vitri, deltaloi, iloi, lastloi;

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  TWBR = ((F_CPU / 400000L) - 16) / 2;

  i2cData[0] = 7;
  i2cData[1] = 0x00;
  i2cData[2] = 0x00;
  i2cData[3] = 0x00;
  while (i2cWrite(0x19, i2cData, 4, false));
  while (i2cWrite(0x6B, 0x01, true));

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68)
  {
    Serial.print(F("Error reading sensor"));
    while (1);
    pinMode (duong1, OUTPUT);
    pinMode (am1, OUTPUT);
    pinMode (duong2, OUTPUT);
    pinMode (am2, OUTPUT);
    vitri = 0;
    deltaloi = 0;
    lastloi = 0;
    iloi = 0;
  }
  delay(100);
  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];
#ifdef RESTRICT_PITCH
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif
  kalmanX.setAngle(roll);
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;
  timer = micros();
}

void loop()
{
  readIMU();
  if (x > 30 || x < -30)
  {
    PID(0, 0, 0, 0);
  }
  else
  {
    PID(x, 15 , 0.5, 4);
  }
  delay (10); 
}
void readIMU()
{
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];
  double dt = (double)(micros() - timer) / 1000000;
  timer = micros();
#ifdef RESTRICT_PITCH
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif
  double gyroXrate = gyroX / 131.0;
  double gyroYrate = gyroY / 131.0;
#ifdef RESTRICT_PITCH
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90))
  {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);
  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate;
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90))
  {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate;
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);
#endif
  gyroXangle += gyroXrate * dt;
  gyroYangle += gyroYrate * dt;
  gyroXangle += kalmanX.getRate() * dt;
  gyroYangle += kalmanY.getRate() * dt;
  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll;
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
  // Print Data 
  //Serial.print(kalAngleX);         
  x = kalAngleX;
}
/*

#include <SimpleKalmanFilter.h>
#include <Wire.h>
#include "i2c.h"
#define RESTRICT_PITCH
SimpleKalmanFilter simpleKalmanFilter(1, 1, 0.001);
 
uint8_t i2cData[14]; // Buffer for I2C data
 
int16_t accX;
float accX_kalman;
 
void setup()
{
    Serial.begin(9600);
    Wire.begin();
#if ARDUINO >= 157
    Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
    TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif
 
    i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
    i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
    i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
    i2cData[3] = 0x03; // Set Accelerometer Full Scale Range to ±16g
    while (i2cWrite(0x19, i2cData, 4, false))
        ; // Write to all four registers at once
    while (i2cWrite2(0x6B, 0x01, true))
        ; // PLL with X axis gyroscope reference and disable sleep mode
}
void loop()
{
 
    while (i2cRead(0x3B, i2cData, 14)) {
        ;
    }
    accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
    accX_kalman = simpleKalmanFilter.updateEstimate((float)accX);
    Serial.print(accX);
    Serial.print(",");
    Serial.print(accX_kalman, 3);
    Serial.println();
}



*/
float PID (float loi, float kp , float ki, float kd)
{
  long ketqua;
  deltaloi = loi - lastloi;
  iloi = + loi;
  if (iloi >= 50)iloi = 50;
  else if (iloi <= -50)iloi = -50;

  ketqua = kp * loi + ki * iloi + kd * deltaloi;
  lastloi = loi;

  if (ketqua >= 255) ketqua = 255;
  else if (ketqua <= -255) ketqua = -255;

  if (ketqua > 0)
  {
    analogWrite(duong1, 0);
    analogWrite(am1, ketqua);
    analogWrite(am2, 0);
    analogWrite(duong2, ketqua);
  }
  else
  {
    analogWrite(am1, 0);
    analogWrite(duong1, -ketqua);
    analogWrite(duong2, 0);
    analogWrite(am2, -ketqua);
  }
}
