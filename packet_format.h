/*
 * packet_format.h
 *
 *  Created on: Jan 25, 2017
 *      Author: Brandon Klefman
 *
 *      Purpose: Define the structure and union for
 *      sending data over the UART to the ground
 *      station and back.
 */

#ifndef PACKET_FORMAT_H_
#define PACKET_FORMAT_H_

/*
 * Data Packet Structure sent by the microcontroller.
 */
struct upacket {
    float UTC; // Current UTC
    float lat;  // Latitude of platform.
    float lon;  // Longitude of platform.
    float alt;  // Altitude of platform.
    float accelX;    // Acceleration reading in X direction.
    float accelY;    // Acceleration reading in Y direction.
    float accelZ;    // Acceleration reading in Z direction.
    float velX;  // Velocity in X direction.
    float velY;  // Velocity in Y direction.
    float velZ;  // Velocity in Z direction.
    float posX;  // Position in X direction.
    float posY;  // Position in Y direction.
    float posZ;  // Position in Z direction.
    float roll;     // Current roll angle.
    float pitch;    // Current pitch angle.
    float yaw;      // Current yaw angle.
    char movement;  // Mode of Operation: Air or Ground? (byte[64])
    bool gndmtr1;   // Is ground motor 1 turned on? (byte[65])
    bool gndmtr2;   // Is ground motor 2 turned on?
    bool amtr1; // Is air motor 1 turned on?
    bool amtr2; // Is air motor 2 turned on?
    bool amtr3; // Is air motor 3 turned on?
    bool amtr4; // Is air motor 4 turned on?
    bool uS1;   // Obstacle detected on sensor 1?
    bool uS2;   // Obstacle detected on sensor 2?
    bool uS3;   // Obstacle detected on sensor 3?
    bool uS4;   // Obstacle detected on sensor 4?
    bool uS5;   // Obstacle detected on sensor 5?
    bool uS6;   // Obstacle detected on sensor 6?
    bool payBay;    // Has the payload been released?
}; // Size of the microcontroller packet is 78 bytes.

/*
 * Union to define the same memory space
 * for the microcontroller packet.
 */
typedef union {
    struct upacket pack;    // Structure as defined above.
    char str[78]; // character addressing of same memory.
} uTxPack;

/*
 * Ground Station packet format.
 */
//
// Target type packet. It will send just the
// location of the target.
typedef struct {
    char pad[3];
    char type; // Target or Control command? T or C?
    float tLat; // Target latitude.
    float tLong;    // Target longitude.
} tGSTPacket ; // Size of gstpacket is 9 bytes

//
// Control type packet. It will send
// information on how to operate in
// manual mode.
typedef struct  {
    char pad[3];
    char type;          // Target or Control command? T or C? (or anything else indicates bad target data)
    float throttle;   // Desired throttle level.
    float throttle2;    // Desired left wheel throttle level for ground travel.
    float roll;         // Desired roll angle.
    float pitch;        // Desired pitch angle.
    float yaw;          // Desired yaw angle.
    char flyordrive;    // Flying or driving?
    char fdConfirm;     // Confirmation of flying or driving.
    bool payloadRelease;    // Release the payload command.
    bool prConfirm;         // Confirmation to release payload command.
} tGSCPacket; // Size of gscpacket is 25 bytes.

//
// Associated Union for gscpacket
typedef union {
    tGSTPacket sTargetPacket;   // Structure as defined above.
    tGSCPacket sControlPacket;   // Structure as defined above.
    char ui8Data[1];
    char ui8DataTarget[sizeof(tGSTPacket)];  // Same memory byte addressable.
    char ui8DataControl[sizeof(tGSCPacket)];  // Same memory byte addressable.
} uRxPack;

#endif /* PACKET_FORMAT_H_ */
