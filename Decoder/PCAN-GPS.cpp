#include "PCAN-GPS.h"
#include "Global_definitions.h"

// void PCAN_GPS::Init(FlexCAN_T4_Base *CAN) { _CAN = CAN; }

bool PCAN_GPS::Process(struct message msg) {
    // Don't check if ID is outside useful range
    if (msg.id < CAN_PCAN_PCAN_GPS_MIN || msg.id > CAN_PCAN_PCAN_GPS_MAX) return false;
    switch (msg.id) {
        case CAN_PCAN_Acceleration:
            _Acceleration.X = (float)(Parse<int16_t>(&msg.buf[0]) * 3.91) / 1000.0f;
            _Acceleration.Y = (float)(Parse<int16_t>(&msg.buf[2]) * 3.91) / 1000.0f;
            _Acceleration.Z = (float)(Parse<int16_t>(&msg.buf[4]) * 3.91) / 1000.0f;
            _Temperature = (float)((msg.buf[6] * 0.5) + 24.0);
            _Acceleration.VerticalAxis = static_cast<PCAN_Axes>((msg.buf[7] & 0b11000000) >> 6);
            _Acceleration.Orientation = static_cast<PCAN_Orientation>(msg.buf[7] & 0b111111);
            return true;
        case CAN_PCAN_MagneticField:
            _MagneticField.X = (float)(Parse<int16_t>(&msg.buf[0]) * 0.3);
            _MagneticField.Y = (float)(Parse<int16_t>(&msg.buf[2]) * 0.3);
            _MagneticField.Z = (float)(Parse<int16_t>(&msg.buf[4]) * 0.3);
            return true;
        case CAN_PCAN_RotationA:
            _Rotation.X = Parse<float>(&msg.buf[0]);
            _Rotation.Y = Parse<float>(&msg.buf[4]);
            return true;
        case CAN_PCAN_RotationB:
            _Rotation.Z = Parse<float>(&msg.buf[0]);
            return true;
        case CAN_PCAN_GPS_Status:
            _GPSStatus.AntennaStatus = static_cast<PCAN_AntennaStatus>(msg.buf[0]);
            _GPSStatus.Satellites = msg.buf[1];
            _GPSStatus.NavigationMethod = static_cast<PCAN_NavigationMethod>(msg.buf[1]);
            return true;
        case CAN_PCAN_GPS_CourseSpeed:
            _Course = Parse<float>(&msg.buf[0]);
            _Speed = Parse<float>(&msg.buf[4]);
            return true;
        case CAN_PCAN_GPS_PositionLongitude:
            _Position.Longitude.Minutes = Parse<float>(&msg.buf[0]);
            _Position.Longitude.Degrees = Parse<uint16_t>(&msg.buf[4]);
            _Position.Longitude.Indicator = static_cast<PCAN_GPSIndicatorEW>(msg.buf[6]);
            return true;
        case CAN_PCAN_GPS_PositionLatitude:
            _Position.Latitude.Minutes = Parse<float>(&msg.buf[0]);
            _Position.Latitude.Degrees = Parse<uint16_t>(&msg.buf[4]);
            _Position.Latitude.Indicator = static_cast<PCAN_GPSIndicatorNS>(msg.buf[6]);
            return true;
        case CAN_PCAN_GPS_Altitude:
            _Position.Altitude = Parse<float>(&msg.buf[0]);
            return true;
        case CAN_PCAN_DelusionsA:
            _Delusions.PDOP = Parse<float>(&msg.buf[0]);
            _Delusions.HDOP = Parse<float>(&msg.buf[4]);
            return true;
        case CAN_PCAN_DelusionsB:
            _Delusions.VDOP = Parse<float>(&msg.buf[0]);
            return true;
        case CAN_PCAN_GPS_DateTime:
            _DateTime_GPS.Year = msg.buf[0] + 2000;
            _DateTime_GPS.Month = static_cast<PCAN_Months>(msg.buf[1]);
            _DateTime_GPS.DayOfMonth = msg.buf[2];
            _DateTime_GPS.Hour = msg.buf[3];
            _DateTime_GPS.Minute = msg.buf[4];
            _DateTime_GPS.Second = msg.buf[5];
            return true;
        case CAN_PCAN_IO:
            _IO.Input_1 = (msg.buf[0] & 0x01);
            _IO.Input_2 = ((msg.buf[0] >> 1) & 0x01);
            _IO.Output = ((msg.buf[0] >> 2) & 0x01);
            _IO.SD_Present = ((msg.buf[0] >> 3) & 0x01);
            _IO.GPS_PowerStatus = ((msg.buf[0] >> 4) & 0x01);
            _IO.DeviceID = ((msg.buf[0] >> 5) & 0x1F);
            return true;
        case CAN_PCAN_RTC_DateTime:
            _DateTime_GPS.Second = msg.buf[0];
            _DateTime_GPS.Minute = msg.buf[1];
            _DateTime_GPS.Hour = msg.buf[2];
            _DateTime_GPS.DayOfWeek = static_cast<PCAN_Days>(msg.buf[3]);
            _DateTime_GPS.DayOfMonth = msg.buf[4];
            _DateTime_GPS.Month = static_cast<PCAN_Months>(msg.buf[5]);
            _DateTime_GPS.Year = Parse<uint16_t>(&msg.buf[6]);
            return true;
    }
    return false;
}

// Return acceleration values in mG
float PCAN_GPS::AccelX() { return _Acceleration.X; }
float PCAN_GPS::AccelY() { return _Acceleration.Y; }
float PCAN_GPS::AccelZ() { return _Acceleration.Z; }

// Return rotation values in deg/s
float PCAN_GPS::RotationX() { return _Rotation.X; }
float PCAN_GPS::RotationY() { return _Rotation.Y; }
float PCAN_GPS::RotationZ() { return _Rotation.Z; }

// Return the current speed in km/h
float PCAN_GPS::Speed() { return _Speed; }

// Return the current positions in minutes and degrees
PCAN_Longitude PCAN_GPS::Long() { return _Position.Longitude; }
PCAN_Latitude PCAN_GPS::Lat() { return _Position.Latitude; }

template <typename T>
T PCAN_GPS::Parse(uint8_t *Buffer, bool Invert) {
    T Result;
    T *ResultPtr = &Result;
    if (Invert) {
        T Tmp;
        T *TmpPtr = &Tmp;
        memcpy(&Tmp, Buffer, sizeof(T));
        for (uint8_t i = 0; i < sizeof(T); i++) {
            ResultPtr[i] = TmpPtr[sizeof(T) - i - 1];
        }
    } else {
        memcpy(ResultPtr, Buffer, sizeof(T));
    }
    return Result;
}