#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
MPU6050 m;
float ypr[3];  
Quaternion q;           // [w, x, y, z]         quaternion container
uint16_t packetSize;    // expected DMP packet size(default = 42 bytes)
uint16_t fifoCount;     // count / bytes currently in FIFO
uint8_t fifoBuf[64]; // FIFO storage buffer
VectorFloat gravity;    // [x, y, z]            gravity vector

void setup() {
  Wire.begin();
  Serial.begin(115200);
  m.initialize();
  m.dmpInitialize();    
  m.setXAccelOffset(-3124);//get these values from calibration script
  m.setYAccelOffset(-1398);
  m.setZAccelOffset(5427);
  m.setXGyroOffset(12);
  m.setYGyroOffset(-36);
  m.setZGyroOffset(47);
  m.setDMPEnabled(true);
  packetSize = m.dmpGetFIFOPacketSize();
  
}

void loop() {
      while (fifoCount < packetSize) fifoCount = m.getFIFOCount();
      fifoCount = m.getFIFOCount();
      m.getFIFOBytes(fifoBuf, packetSize);
      fifoCount -= packetSize;
      m.dmpGetQuaternion(&q, fifoBuf);
      m.dmpGetGravity(&gravity, &q);
      m.dmpGetYawPitchRoll(ypr, &q, &gravity);
      //note the first value, which seems to correspond to xpos doesn't always zero at start.
      /*Serial.print("ypr\t");
      Serial.print(ypr[0] * 180/M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180/M_PI);
      Serial.print("\t");
      Serial.println(ypr[2] * 180/M_PI);*/
      // ** uncomment above for serial output **
}
