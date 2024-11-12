#ifndef PCAN_GPS_TYPES_H
#define PCAN_GPS_TYPES_H

enum class PCAN_Days { Monday = 0, Tuesday, Wednesday, Thursday, Friday, Saturday, Sunday };

enum class PCAN_Months {
    January = 0,
    February,
    March,
    April,
    May,
    June,
    July,
    August,
    September,
    October,
    November,
    December
};

struct PCAN_DateTime {
    // Date Component
    uint16_t Year = 0;
    PCAN_Months Month;
    uint8_t DayOfMonth = 0;
    PCAN_Days DayOfWeek;
    uint8_t Hour = 0;
    uint8_t Minute = 0;
    uint8_t Second = 0;
};

enum class PCAN_Axes { Undefined = 0, X, Y, Z };

enum class PCAN_Orientation {
    Flat = 0,
    FlatUpsideDown,
    LandscapeLeft,
    LandscapeRight,
    Portrait,
    PortraitUpsideDown
};

struct PCAN_Acceleration {
    float X = 0.0;
    float Y = 0.0;
    float Z = 0.0;
    PCAN_Axes VerticalAxis;
    PCAN_Orientation Orientation;
};

struct PCAN_MagneticField {
    float X = 0.0;
    float Y = 0.0;
    float Z = 0.0;
};

struct PCAN_Rotation {
    float X = 0.0;
    float Y = 0.0;
    float Z = 0.0;
};

enum class PCAN_AntennaStatus { Init = 0, DontKnow, Ok, Short, Open };

enum class PCAN_NavigationMethod { Init = 0, None, TwoD, ThreeD };

struct PCAN_GPSStatus {
    PCAN_AntennaStatus AntennaStatus;
    uint8_t Satellites = 0;
    PCAN_NavigationMethod NavigationMethod;
};

enum class PCAN_GPSIndicatorEW { Init = 0, East = 69, West = 87 };

enum class PCAN_GPSIndicatorNS {
    Init = 0,
    North = 78,
    South = 83,
};

struct PCAN_Longitude {
    float Minutes = 0.0;
    float Degrees = 0.0;
    PCAN_GPSIndicatorEW Indicator;
};

struct PCAN_Latitude {
    float Minutes = 0.0;
    float Degrees = 0.0;
    PCAN_GPSIndicatorNS Indicator;
};

struct PCAN_Position {
    PCAN_Longitude Longitude;
    PCAN_Latitude Latitude;
    float Altitude = 0.0;
};

struct PCAN_Delusions {
    float PDOP = 0.0;
    float HDOP = 0.0;
    float VDOP = 0.0;
};

struct PCAN_IO {
    bool Input_1;
    bool Input_2;
    bool Output;
    bool SD_Present;
    bool GPS_PowerStatus;
    uint8_t DeviceID;
};

enum class PCAN_GyroScale { _250Ds = 0, _500Ds = 1, _2000Ds = 2 };

enum class PCAN_AccelScale { _2G = 1, _4G = 2, _8G = 3, _16G = 4 };

enum class PCAN_AccelCalibrationTarget { _0G = 0, _Plus1G = 1, _Minus1G = 2 };

struct PCAN_AccelCalibration {
    PCAN_AccelCalibrationTarget X;
    PCAN_AccelCalibrationTarget Y;
    PCAN_AccelCalibrationTarget Z;
    bool StartFastCalibration;
};

#endif