/*
@name: OSCDecode.h
@version: 0.1
@description: Library file created to contain messages related to our control format.
@project:Edison Drone
@team:Segmentation Fault
@authors:Matt Almenshad, David Zimmerly, Jacob Lin, Joseph McLaughlin
@instructor: Boyana Norris
@last-update:02/24/2016
*/
#ifndef OSCDECODE_h
#define OSCDECODE_h

int decodeValue(char *packet){


    union {
        float f;
        uint8_t b[4];
    } u;
    char rData[4];
    rData[0] = packet[11];
    rData[1] = packet[10];
    rData[2] = packet[9];
    rData[3] = packet[8];
    memcpy(u.b, rData, 4);

    return (int) (u.f+0.5);

}

#endif /* END_OF_OSCDECODE_h */
