#include "Global_definitions.h"
#include "CAR.h"
#include "CAN.h"
#include "PCAN-GPS.h"
#include <iomanip>
#include <sstream>
#include <string>

std::string Del = ",";

std::string LoggedValues[] = {
    "vtime",                // 1
    "Th5",                  // 2
    "Th12",                 // 3
    "Thr",                  // 4
    "BT",                   // 5
    "BF",                   // 6
    "BR",                   // 7
    "SuFR",                 // 8
    "SuFL",                 // 9
    "SuRR",                 // 10
    "SuRL",                 // 11
    "TrFR",                 // 12
    "TrFL",                 // 13
    "TrRR",                 // 14
    "TrRL",                 // 15
    "In1",                  // 16
    "Out1",                 // 17
    "In2",                  // 18
    "Out2",                 // 19
    "SteeringAngle",        // 20
    "GPS1S",                // 21
    "GPS1Lat",              // 22
    "GPS1Long",             // 23
    "GPS1Ax",               // 24
    "GPS1Ay",               // 25
    "GPS1Az",               // 26
    "GPS1Gx",               // 27
    "GPS1Gy",               // 28
    "GPS1Gz",               // 29
    "RPMFR",                // 30
    "RPMFL",                // 31
    "RPMRR",                // 32
    "RPMRL",                // 33
    "MTEMPFR",              // 34
    "MTEMPFL",              // 35
    "MTEMPRR",              // 36
    "MTEMPRL",              // 37
    "IFR",                  // 38
    "IFL",                  // 39
    "IRR",                  // 40
    "IRL",                  // 41
    "CPFR",                 // 42
    "CPFL",                 // 43
    "CPRR",                 // 44
    "CPRL",                 // 45
    "DEM_TRQ_FR",           // 46
    "DEM_TRQ_FL",           // 47
    "DEM_TRQ_RR",           // 48
    "DEM_TRQ_RL",           // 49
    "ATFR",                 // 50
    "ATFL",                 // 51
    "ATRR",                 // 52
    "ATRL",                 // 53
    "SV",                   // 54
    "IVTV1",                // 55
    "IVTV2",                // 56
    "IVTV3",                // 57
    "IVTC",                 // 58
    "IVTTemp",              // 59
    "VMin",                 // 60
    "VMax",                 // 61
    "TMin",                 // 62
    "TMax",                 // 63
    "SOC",                  // 64
    "ERR_INF_FR",           // 65
    "ERR_INF_FL",           // 66
    "ERR_INF_RR",           // 67
    "ERR_INF_RL",           // 68
    "TV_STR_WHL_ANGLE",     // 69
    "TV_STR_WHL_RATE",      // 70
    "TV_DESIRED_YAW",       // 71
    "TV_TRQ_DEMAND",        // 72
    "TV_REG_TRQ_DEMAND",    // 73
    "TV_VEL_FROM_RPM",      // 74
    "TV_FR_TRQ",            // 75
    "TV_RR_TRQ",            // 76
    "TV_FL_TRQ",            // 77
    "TV_RL_TRQ",            // 78
    "TV_ERROR_OUT",         // 79
    "RTD",                  // 80 is RTD flag from ECU
    "BMSFlags",             // 81
    "LV_AMS_TotalVolt",     // 82
    "LV_AMS_TEMP_MAX",      // 83
    "LV_AMS_TEMP_MIN",      // 84
    "LV_AMS_CELL_MAX",      // 85
    "LV_AMS_CELL_MIN",      // 86
    "LV_AMS_WATT",          // 87
    "LV_AMS_STATUS",        // 88
    "Preset",               // 89
    "TV_State",             // 90
    "Pow_IVT",              // 91
    "Pow_FR",               // 92
    "Pow_FL",               // 93
    "Pow_RR",               // 94
    "Pow_RL",               // 95
    "Sess_Dist_RPM",        // 96
    "Sess_Dist_GPS",        // 97
    "DRS",                  // 98
    "Regen",                // 99
    "Vmin_Seg",             // 100
    "Vmin_Cell",            // 101
    "TMax_Seg",             // 102
    "TMax_Cell",            // 103
    "gkazofreno",           // 104
    "Trq_Err_FL",           // 105
    "Trq_Err_FR",           // 106
    "Trq_Err_RL",           // 107
    "Trq_Err_RR",           // 118
    "Apps_Implau",          // 109
    "Derating",             // 110
    "P_Limit",              // 111
    "P_Limit_SP",           // 112
    "P_Limit_Kp",           // 113
    "P_Limit_Ki",           // 114
    "TV_Kp",                // 115
    "Traction_Kp",          // 116
    "Bad_Cell",             // 117
    "Bad_Thermistor",       // 118
    "IVT_C_COUNTER",        // 119
    "IVT_E_COUNTER"         // 120

};

const uint8_t LoggedValueCounter = sizeof(LoggedValues) / sizeof(LoggedValues[0]);

bool IVT_Present() { return MyCar.IVTLastUpdate < 500; }

//SERIALIZATION

std::string SerializeBMSFlags() {
	std::string Flags = "";
	Flags += MyCar.SystemSafe ? "1" : "0";
	Flags += MyCar.OverVoltage ? "1" : "0";
	Flags += MyCar.UnderVoltage ? "1" : "0";
	Flags += MyCar.OverTempCells ? "1" : "0";
	Flags += MyCar.UnderTempCells ? "1" : "0";
	Flags += MyCar.AbnormalityError ? "1" : "0";
	Flags += MyCar.VoltageAbnormality ? "1" : "0";
	Flags += MyCar.TempAbnormality ? "1" : "0";
	Flags += MyCar.VoltageSamplingAbnormality ? "1" : "0";
	Flags += MyCar.TemperatureSamplingAbnormality ? "1" : "0";
	Flags += MyCar.OverCurrent? "1" : "0";
	Flags += MyCar.UnderCurrent? "1" : "0";
	Flags += IVT_Present() ? "0" : "1";
	Flags += MyCar.ShutdownStatus ? "1" : "0";
	Flags += MyCar.BMSShutdown ? "1" : "0";
	Flags += MyCar.IsPrecharged ? "1" : "0";
	return Flags;
}

std::string SerializeSensors() {
	std::string LogStr;
    LogStr +=  std::to_string(MyCar.LogTime) + Del; //1
    LogStr += std::to_string(MyCar.Throttle_5)  + Del ; //2 
    LogStr += std::to_string(MyCar.Throttle_12) + Del ; //3
    LogStr += std::to_string(MyCar.Throttle) + Del ; //4
    LogStr += std::to_string(MyCar.Brake_Travel) + Del; //5
    LogStr += std::to_string(MyCar.Brake_Front) + Del; //6
    LogStr += std::to_string(MyCar.Brake_Rear) + Del; //7
    //SuspensionFR
    std::stringstream sFR;
    sFR << std::fixed << std::setprecision(3) << MyCar.Suspension_Front_Right;
    std::string Susp_FR = sFR.str();
    LogStr += Susp_FR + Del; //8
    //SuspensionFL
    std::stringstream sFL;
    sFL << std::fixed << std::setprecision(3) << MyCar.Suspension_Front_Left;
    std::string Susp_FL = sFL.str();
    LogStr += Susp_FL + Del; //9
    //SuspensionRR
    std::stringstream sRR;
    sRR << std::fixed << std::setprecision(3) << MyCar.Suspension_Rear_Right;
    std::string Susp_RR = sRR.str();
    LogStr += Susp_RR + Del; //10
    //SuspensionRL
    std::stringstream sRL;
    sRL << std::fixed << std::setprecision(3) << MyCar.Suspension_Rear_Left;
    std::string Susp_RL = sRL.str();
    LogStr += Susp_RL + Del; //11
    LogStr += std::to_string(MyCar.Transmission_Front_Right) + Del ; //12
    LogStr += std::to_string(MyCar.Transmission_Front_Left) + Del; //13
    LogStr += std::to_string(MyCar.Transmission_Rear_Right) + Del; //14
    LogStr += std::to_string(MyCar.Transmission_Rear_Left) + Del; //15
    LogStr += std::to_string(MyCar.Coolant_In_1) + Del ; //16
    LogStr += std::to_string(MyCar.Coolant_Out_1) + Del; //17
    LogStr += std::to_string(MyCar.Coolant_In_2) + Del ; //18
    LogStr += std::to_string(MyCar.Coolant_Out_2) + Del; //19
    LogStr += std::to_string(MyCar.Steering_Wheel_Angle) + Del; //20
     //Speed
     std::stringstream GPS_S;
     GPS_S << std::fixed << std::setprecision(2) << GPS1.Speed();
     std::string Speed = GPS_S.str();
     LogStr += Speed + Del; //21
     //Latitude
     float latitude = GPS1.Lat().Degrees + GPS1.Lat().Minutes / 60.0f;
     LogStr += std::to_string(latitude) + Del; //22
     //Longitude
     float longitude = GPS1.Long().Degrees + GPS1.Long().Minutes / 60.0f;
     LogStr += std::to_string(longitude)+ Del; //23
     //AccelX
     std::stringstream AX;
     AX << std::fixed << std::setprecision(2) << GPS1.AccelX();
     std::string AccelX = AX.str();
     LogStr += AccelX + Del; //24
     //AccelY
     std::stringstream AY;
     AY << std::fixed << std::setprecision(2) << GPS1.AccelY();
     std::string AccelY = AY.str();
     LogStr += AccelY + Del; //25
     //AccelZ
     std::stringstream AZ;
     AZ << std::fixed << std::setprecision(2) << GPS1.AccelZ();
     std::string AccelZ = AZ.str();
     LogStr += AccelZ + Del; //26
     //RotationX
     std::stringstream RX;
     RX << std::fixed << std::setprecision(2) << GPS1.RotationX();
     std::string RotationX = RX.str();
     LogStr += RotationX + Del; //27
     //RotationY
     std::stringstream RY;
     RY << std::fixed << std::setprecision(2) << GPS1.RotationY();
     std::string RotationY = RY.str();
     LogStr += RotationY + Del; //28
     //RotationZ
     std::stringstream RZ;
     RZ << std::fixed << std::setprecision(2) << GPS1.RotationZ();
     std::string RotationZ = RZ.str();
     LogStr += RotationZ + Del; //29
    LogStr += std::to_string(MyCar.RPM_Front_Right) + Del ; //30
    LogStr += std::to_string(MyCar.RPM_Front_Left) + Del; //31
    LogStr += std::to_string(MyCar.RPM_Rear_Right) + Del ; //32
    LogStr += std::to_string(MyCar.RPM_Rear_Left) + Del; //33
    LogStr += std::to_string(MyCar.Motor_Temp_Front_Right) + Del; //34
    LogStr += std::to_string(MyCar.Motor_Temp_Front_Left) + Del; //35
    LogStr += std::to_string(MyCar.Motor_Temp_Rear_Right) + Del; //36
    LogStr += std::to_string(MyCar.Motor_Temp_Rear_Left) + Del; //37
    LogStr += std::to_string(MyCar.IGBT_Temp_Front_Right) + Del; //38
    LogStr += std::to_string(MyCar.IGBT_Temp_Front_Left) + Del; //39 
    LogStr += std::to_string(MyCar.IGBT_Temp_Rear_Right) + Del ; //40
    LogStr += std::to_string(MyCar.IGBT_Temp_Rear_Left) + Del; //41
    LogStr += std::to_string(MyCar.Plate_Temp_Front_Right) + Del ; //42
    LogStr +=std::to_string(MyCar.Plate_Temp_Front_Left) + Del; //43
    LogStr += std::to_string(MyCar.Plate_Temp_Rear_Right) + Del; //44
    LogStr +=std::to_string(MyCar.Plate_Temp_Rear_Left) + Del; //45
    LogStr += std::to_string(MyCar.Demanded_Front_Right) + Del ; //46
    LogStr += std::to_string(MyCar.Demanded_Front_Left) + Del; //47
    LogStr += std::to_string(MyCar.Demanded_Rear_Right) + Del ; //48
    LogStr += std::to_string(MyCar.Demanded_Rear_Left) + Del; //49
    LogStr += std::to_string(MyCar.Actual_Front_Right) + Del; //50
    LogStr += std::to_string(MyCar.Actual_Front_Left) + Del; //51
    LogStr += std::to_string(MyCar.Actual_Rear_Right) + Del; //52
    LogStr += std::to_string(MyCar.Actual_Rear_Left) + Del; //53
    LogStr += std::to_string(MyCar.StackVoltage) + Del; //54
    LogStr += std::to_string(MyCar.IVT_Voltage1) + Del; //55
    LogStr += std::to_string(MyCar.IVT_Voltage2) + Del; //56
    LogStr += std::to_string(MyCar.IVT_Voltage3) + Del; //57
    LogStr += std::to_string(MyCar.IVT_Current) + Del; //58
    LogStr += std::to_string(MyCar.IVT_Temperature) + Del; //59
    LogStr += std::to_string(MyCar.Voltage_BMS_Min) + Del ; //60
    LogStr += std::to_string(MyCar.Voltage_BMS_Max) + Del; //61
    LogStr += std::to_string(MyCar.Temp_BMS_Min) + Del; //62
    LogStr += std::to_string(MyCar.Temp_BMS_Max) + Del; //63
    LogStr += std::to_string(MyCar.SOC) + Del; //64
    LogStr += std::to_string(MyCar.Error_Info_Front_Right) + Del; //65
    LogStr += std::to_string(MyCar.Error_Info_Front_Left) + Del; //66
    LogStr += std::to_string(MyCar.Error_Info_Rear_Right) + Del; //67
    LogStr += std::to_string(MyCar.Error_Info_Rear_Left) + Del; //68
    LogStr += std::to_string(MyCar.Steering_Wheel_Angle) + Del; //69
    LogStr += std::to_string(MyCar.Steering_Wheel_Rate) + Del; //70
    LogStr += std::to_string(MyCar.Desired_Yaw_Rate)+ Del; //71
    LogStr += std::to_string(MyCar.Torque_Demand) + Del ; //72
    LogStr += std::to_string(MyCar.Regen_Torque_Demand )+ Del; //73
    LogStr += std::to_string(MyCar.Vehicle_Velocity_from_RPM) + Del; //74
    LogStr += std::to_string(MyCar.FR_Torque)+ Del; //75
    LogStr += std::to_string(MyCar.RR_Torque) + Del ; //76
    LogStr += std::to_string(MyCar.FL_Torque)+ Del; //77
    LogStr += std::to_string(MyCar.RL_Torque) + Del; //78
    LogStr += std::to_string(MyCar.TV_error) + Del; //79
    LogStr += std::to_string(MyCar.RTD) + Del; //80
    LogStr += SerializeBMSFlags() + Del; //81
    LogStr += std::to_string(MyCar.Voltage)+ Del; //82
    LogStr += std::to_string(MyCar.Max_temp)+ Del; //83
    LogStr += std::to_string(MyCar.Min_temp)+ Del; //84
    LogStr += std::to_string(MyCar.Max_Cell_Voltage)+ Del; //85
    LogStr += std::to_string(MyCar.Min_Cell_Voltage)+ Del; //86
    LogStr += std::to_string(MyCar.Watt_drown)+ Del; //87
    LogStr += std::to_string(MyCar.Status) + Del; //88
    LogStr += std::to_string((int)MyCar.Preset) + Del; //89
    LogStr += std::to_string((int)MyCar.TV) + Del; //90
    LogStr += std::to_string(MyCar.IVT_Power) + Del; //91
    LogStr += std::to_string(MyCar.Actual_Power_Front_Right) + Del; //92
    LogStr += std::to_string(MyCar.Actual_Power_Front_Left) + Del; //93
    LogStr += std::to_string(MyCar.Actual_Power_Rear_Right) + Del; //94
    LogStr += std::to_string(MyCar.Actual_Power_Rear_Left) + Del; //95
    LogStr += std::to_string(MyCar.Distance_Traveled_From_RPM) + Del; //96
    LogStr += std::to_string(MyCar.Distance_Traveled_From_GPS) + Del; //97
    LogStr += std::to_string((int)MyCar.DRS) + Del; //98
    LogStr += std::to_string((int)MyCar.Regen) + Del; //99
    LogStr += std::to_string(MyCar.Voltage_BMS_Min_Segment) + Del; //100
    LogStr += std::to_string(MyCar.Voltage_BMS_Min_Cell) + Del; //101
    LogStr += std::to_string(MyCar.Temp_BMS_Max_Segment) + Del; //102
    LogStr += std::to_string(MyCar.Temp_BMS_Max_Cell) +Del; //103
    LogStr += std::to_string((uint8_t)MyCar.gkazofreno) + Del; //104
    LogStr += std::to_string(abs(MyCar.Demanded_Front_Left - MyCar.Actual_Front_Left)) + Del; //105
    LogStr += std::to_string(abs(MyCar.Demanded_Front_Right - MyCar.Actual_Front_Right)) + Del; //106
    LogStr += std::to_string(abs(MyCar.Demanded_Rear_Left - MyCar.Actual_Rear_Left)) + Del; //107
    LogStr += std::to_string(abs(MyCar.Demanded_Rear_Right - MyCar.Actual_Rear_Right)) + Del; //108
    LogStr += std::to_string((uint8_t) (MyCar.Throttle>200) ) + Del; //109
    LogStr += std::to_string((uint8_t) (MyCar.Inverter_Derating) ) + Del; //110
    LogStr += std::to_string((uint8_t)MyCar.Power_Limit) + Del; //111
    LogStr += std::to_string(MyCar.powerlimitgains_setpoint) + Del; //112
    LogStr += std::to_string(MyCar.powerlimitgains_kp) + Del; //113
    LogStr += std::to_string(MyCar.powerlimitgains_ki) + Del; //114
    LogStr += std::to_string(MyCar.tvgains_kp) + Del; //115
    LogStr += std::to_string(MyCar.tvgains_kp_traction) + Del; //116
    LogStr += std::to_string(MyCar.Bad_Cell) + Del; //117
    LogStr += std::to_string(MyCar.Bad_Thermistor) + Del; //118
    LogStr += std::to_string(MyCar.IVT_CurrentCounter) + Del; //119
    LogStr += std::to_string(MyCar.IVT_EnergyCounter)+ Del; //120
    for (uint8_t Cell = 0; Cell < 144; Cell++) {
        std::stringstream CELL;
        CELL << std::fixed << std::setprecision(2) << MyCar.Voltages[Cell] / 10000.0F ;
        std::string cell = CELL.str();
        LogStr += cell + Del;
    }
    for (uint8_t Thermistor = 0; Thermistor < 59; Thermistor++) {
        LogStr += std::to_string(MyCar.Temperatures[Thermistor]) + Del;
    }
    // Serialize the last BMS temperature to avoid string ending in comma
    LogStr += std::to_string(MyCar.Temperatures[59]);
 return LogStr;
}

//END OF SERIALIZATION

void InitLogin(std::ofstream& LogFile){
    // Write first lines to the log file
    for (uint8_t Index = 0; Index <= LoggedValueCounter - 1; Index++) {
    LogFile << LoggedValues[Index] + Del;
    }
    // LogFile << LoggedValues[LoggedValueCounter - 1] << std::endl;
    for (uint8_t Index = 1; Index <= TOTAL_CELLS; Index++) {
        LogFile << "Cell #" + std::to_string(Index) + Del ;
    }
    for (uint8_t Index = 1; Index < TOTAL_THERMISTORS; Index++) {
        LogFile <<  "Therm #" + std::to_string(Index)  + Del ;
    }

    LogFile << "Therm #" + std::to_string(TOTAL_THERMISTORS) << std::endl ;

    LogFile.flush();

}
