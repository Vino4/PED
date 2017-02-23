/*
@name: UDP Reader
@version: 0.2
@description: This code reads UDP packets over Wlan and prints the content into Serial
*/
#include <EthernetUdp.h>         // UDP library from: bjoern@cs.stanford.edu 12/30/2008

unsigned int localPort = 8888;      // local port to listen on

// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  //buffer to hold incoming packet,
char  ReplyBuffer[] = "acknowledged";       // a string to send back

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

void setup() {
  Udp.begin(localPort); //initialize udp to listen to localport

  Serial.begin(115200); //start serial at Puad 115200
}

void loop() {
  // if there's data available, read a packet
  int packetSize = Udp.parsePacket(); //when you parse a packet the function returns its size, save it
  if (packetSize) {//if there is data in the packet
    Serial.print("Received packet of size ");
    Serial.println(packetSize);//print the size

    // read the packet into packetBufffer
    Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);//read the data into packet buffer
    Serial.println("Contents:");
    Serial.println(packetBuffer);//print the data
  }
  delay(10);
}
