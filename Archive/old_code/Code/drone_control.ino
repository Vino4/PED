/*
@name: OSC Drone Control Code
@version: 0.3
@description: This code reads UDP packages over LAN and decodes them using OSC formatted messages of size 12 to control the quad-copter.
@project:Edison Drone
@team:Segmentation Fault
@authors:Matt Almenshad, David Zimmerly, Jacob Lin, Joseph McLaughlin
@instructor: Boyana Norris
@last-update:02/28/2016
*/

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
MPU6050 m;
float ypr[3];  
Quaternion q;           // [w, x, y, z]         quaternion container
uint16_t packetSize2;    // expected DMP packet size(default = 42 bytes)
uint16_t fifoCount;     // count / bytes currently in FIFO
uint8_t fifoBuf[64]; // FIFO storage buffer
VectorFloat gravity;    // [x, y, z]            gravity vector



#include <Servo.h> //to help operate the ESCs

#include <EthernetUdp.h>        // UDP library

#include <OSCDecode.h> // our decode OSC function


Servo escFR;   // create servo object to control front right ESC
Servo escFL;  // create servo object to control front left ESC
Servo escBR; // create servo object to control back right ESC
Servo escBL;// create servo object to control back left ESC

unsigned int localPort = 8888;      // local port to listen on

// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  //buffer to hold incoming packet of size UDP_TX_PACKET_MAX_SIZE

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;





void setup() {
  
  
  Serial.println("Initializing Udp..");
  Udp.begin(localPort); //initialize udp to listen to localport
  Serial.print("Udp started on port: ");
  Serial.println(localPort);

  //ESC calibration
  escFR.writeMicroseconds(2000); //set maximum front right ESC signal
  escFL.writeMicroseconds(2000); //set maximum front left ESC signal
  escBR.writeMicroseconds(2000); //set maximum back right ESC signal
  escBL.writeMicroseconds(2000); //set maximum back left ESC signal
  escFR.attach(3);  //the pin for the front right ESC control
  escFL.attach(5);  //the pin for the front left ESC control
  escBR.attach(6);  //the pin for the back right ESC control
  escBL.attach(9);  //the pin for the back left ESC control
  delay(2000);
  escFR.writeMicroseconds(700); //set minimum front right ESC signal
  escFL.writeMicroseconds(700); //set minimum front left ESC signal
  escBR.writeMicroseconds(700); //set minimum back right ESC signal
  escBL.writeMicroseconds(700); //set minimum back left ESC signal

  Wire.begin();
  Serial.begin(115200);
  m.initialize();
  m.dmpInitialize();    
  m.setXAccelOffset(-3240);//get these values from calibration script
  m.setYAccelOffset(-1327);
  m.setZAccelOffset(5428);
  m.setXGyroOffset(10);
  m.setYGyroOffset(-35);
  m.setZGyroOffset(52);
  m.setDMPEnabled(true);
  packetSize2 = m.dmpGetFIFOPacketSize();

}

void loop() {
  // if there's data available, read a packet
  int packetSize = Udp.parsePacket(); //when you parse a packet the function returns its size, save it
  if (packetSize) {//if there is data in the packet
    Serial.print("Received packet of size ");
    Serial.println(packetSize);//print the size

    // Decode packet and send it to motors
    Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);//read the packet of size UDP_TX_PACKET_MAX_SIZE into packet buffer
    Serial.println("Decoded Content:");
      int decodedVal = decodeValue(packetBuffer);
      Serial.print("Command:");
      Serial.println(packetBuffer[1]);
      Serial.print("Value:");
      Serial.println(decodedVal);
      switch(packetBuffer[1]){
        case 'f':
          escFR.writeMicroseconds(decodedVal); //set front right ESC signal
          escFL.writeMicroseconds(decodedVal); //set front left ESC signal
          escBR.writeMicroseconds(decodedVal); //set back right ESC signal
          escBL.writeMicroseconds(decodedVal); //set back left ESC signal
          Serial.print("Frequency was set to: ");
          Serial.println(decodedVal);
        break;

        case 'o':
          if (decodedVal == 1){
            escFR.writeMicroseconds(700); //set front right ESC signal
            escFL.writeMicroseconds(700); //set front left ESC signal
            escBR.writeMicroseconds(700); //set back right ESC signal
            escBL.writeMicroseconds(700); //set back left ESC signal
            Serial.println("Frequency was set to: 700");
          }
        break;

        case 'm':
            escFR.writeMicroseconds(2000); //set front right ESC signal
            escFL.writeMicroseconds(2000); //set front left ESC signal
            escBR.writeMicroseconds(2000); //set back right ESC signal
            escBL.writeMicroseconds(2000); //set back left ESC signal
            Serial.println("Frequency was set to: 2000");
        break;

        default:
        break;
      }
  }
  while (fifoCount < packetSize2) fifoCount = m.getFIFOCount();
   fifoCount = m.getFIFOCount();
   m.getFIFOBytes(fifoBuf, packetSize2);
   fifoCount -= packetSize2;
   m.dmpGetQuaternion(&q, fifoBuf);
   m.dmpGetGravity(&gravity, &q);
   m.dmpGetYawPitchRoll(ypr, &q, &gravity);
      //note the first value, which seems to correspond to xpos doesn't always zero at start.
   /*Serial.print("ypr\t");
   Serial.print(ypr[0] * 180/M_PI);
   Serial.print("\t");
   Serial.print(ypr[1] * 180/M_PI);
   Serial.print("\t");
   Serial.println(ypr[2] * 180/M_PI);*/ //uncomment for mpu ypr readings via serial
  //delay(10);

}
