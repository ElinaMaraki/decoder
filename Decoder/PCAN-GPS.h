#ifndef PCAN_GPS_H
#define PCAN_GPS_H
#include "CAN.h"
#include "Types.h"
#include <cinttypes>
#include <cstddef>
#include <iostream>

class PCAN_GPS {
public:
    // void Init(FlexCAN_T4_Base *CAN);
    bool Process(struct message msg);
    float AccelX();
    float AccelY();
    float AccelZ();
    float RotationX();
    float RotationY();
    float RotationZ();
    float Speed();
    PCAN_Longitude Long();
    PCAN_Latitude Lat();

private:
    // FlexCAN_T4_Base *_CAN = nullptr;
    // void SendCAN(CAN_message_t Message);
    float _Temperature = 0.0;
    PCAN_Acceleration _Acceleration;
    PCAN_MagneticField _MagneticField;
    PCAN_Rotation _Rotation;
    PCAN_GPSStatus _GPSStatus;
    float _Course = 0.0;
    float _Speed = 0.0;
    PCAN_Position _Position;
    PCAN_Delusions _Delusions;
    PCAN_IO _IO;
    PCAN_DateTime _DateTime_RTC;
    PCAN_DateTime _DateTime_GPS;
    template <typename T>
    T Parse(uint8_t *Buffer, bool Invert = false);
};

PCAN_GPS GPS1;
#endif
