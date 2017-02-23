/*
@name: OSC Drone Control Code
@version: 0.1
@description: This code reads UDP packages over LAN and decodes them using OSC formatted messages of size 12 to control the quad-copter.
@project:Edison Drone
@team:Segmentation Fault
@authors:Matt Almenshad, David Zimmerly, Jacob Lin, Joseph McLaughlin
@instructor: Boyana Norris
@last-update:02/24/2016
*/

#include <Servo.h> //to help operate the ESCs

#include <EthernetUdp.h>        // UDP library

#include <OSCDecode.h>

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
  Serial.begin(115200); //start serial at Puad 115200
  Serial.println("Initializing Udp..");
  Udp.begin(localPort); //initialize udp to listen to localport
  Serial.print("Udp started on port: ");
  Serial.println(localPort);

  escFR.writeMicroseconds(700); //set initial front right ESC signal
  escFL.writeMicroseconds(700); //set initial front left ESC signal
  escBR.writeMicroseconds(700); //set initial back right ESC signal
  escBL.writeMicroseconds(700); //set initial back left ESC signal
  escFR.attach(3);  //the pin for the front right ESC control
  escFL.attach(5);  //the pin for the front left ESC control
  escBR.attach(6);  //the pin for the back right ESC control
  escBL.attach(9);  //the pin for the back left ESC control

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
      escFR.writeMicroseconds(decodedVal); //set front right ESC signal
      escFL.writeMicroseconds(decodedVal); //set front left ESC signal
      escBR.writeMicroseconds(decodedVal); //set back right ESC signal
      escBL.writeMicroseconds(decodedVal); //set back left ESC signal

  }
  delay(10);

}
