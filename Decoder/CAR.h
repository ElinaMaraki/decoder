#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#ifndef CAR_H
#define CAR_H

#define TOTAL_CELLS 144
#define TOTAL_THERMISTORS 60

enum ECU_Mode {Testing=0, Acceleration, Skidpad, Autocross, Endurance};
enum ECU_TV_State { TV_Off=0, TV_Low, TV_Medium, TV_High, TV_Max};
enum ECU_TC_State { TC_Off=0, TC_Low, TC_Medium, TC_High, TC_Max};
enum ECU_Regen_State { REGEN_Off=0, REGEN_Low, REGEN_Medium, REGEN_High, REGEN_Max};
enum ECU_Power_Limit_State{Off=0 ,On};

struct My_Struct{
  unsigned long LogTime;

  //REAR NODE
  float Suspension_Rear_Right;
  float Suspension_Rear_Left; 
  float Transmission_Rear_Right; 
  float Transmission_Rear_Left; 
  float Coolant_In_1; 
  float Coolant_Out_1; 
  float Coolant_In_2; 
  float Coolant_Out_2;
    //RAW
  uint16_t RawSuspension_Rear_Left;   
  uint16_t RawSuspension_Rear_Right;   
  uint16_t RawTransmission_Rear_Left;   
  uint16_t RawTransmission_Rear_Right;   

  //FRONT NODE
  float Brake_Travel; 
  float Brake_Front; 
  float Brake_Rear; 
  float Brake_Bias; 
  float Throttle_12; 
  float Throttle_5; 
  float Throttle; 
  float Transmission_Front_Right; 
  float Transmission_Front_Left; 
  float Suspension_Front_Right; 
  float Suspension_Front_Left; 
    //RAW
  uint16_t RawThrottle1;   
  uint16_t RawThrottle2;   
  uint16_t RawBrakeTravel;    
  uint16_t RawBrakeFront;   
  uint16_t RawBrakeRear;   
  uint16_t RawSuspension_Front_Left;
  uint16_t RawSuspension_Front_Right;
  uint16_t RawTransmission_Front_Left;
  uint16_t RawTransmission_Front_Right;

  //ECU_Flags
  bool RTD;
  bool BrakeLight;
  bool Buzzer;
  bool EnablePumps;
  bool EnableFans;
  bool gkazofreno;

  //ECU TV
  int Steering_Wheel_Angle;
  float Desired_Yaw_Rate;
  float Torque_Demand;
  float Regen_Torque_Demand;
  float Vehicle_Velocity_from_RPM;
  float FL_Torque;
  float FR_Torque;
  float RL_Torque;
  float RR_Torque;
  float Steering_Wheel_Rate;
  uint16_t TV_error;
  uint16_t Steering_Wheel_Angle_Raw;

  //ECU
  float Demanded_Front_Right;
  float Demanded_Front_Left;
  float Demanded_Rear_Right;
  float Demanded_Rear_Left;

//These are commented out in logging
  float Regen_Demanded_Front_Right;
  float Regen_Demanded_Front_Left;
  float Regen_Demanded_Rear_Right;
  float Regen_Demanded_Rear_Left;

  enum ECU_Mode Preset;
  enum ECU_TV_State TV;
  enum ECU_TC_State TC;
  enum ECU_Regen_State Regen;
  enum ECU_Power_Limit_State Power_Limit;
  uint16_t left_steering_calibration;
  uint16_t right_steering_calibration;

//INVERTER
  int16_t RPM_Front_Right;
  int16_t RPM_Front_Left;
  int16_t RPM_Rear_Right;
  int16_t RPM_Rear_Left;
  //FROM HERE
  float Actual_Magnetizing_Current_Front_Right;
  float Actual_Magnetizing_Current_Front_Left;
  float Actual_Magnetizing_Current_Rear_Right;
  float Actual_Magnetizing_Current_Rear_Left;
  float Actual_Torque_Current_Front_Right;
  float Actual_Torque_Current_Front_Left;
  float Actual_Torque_Current_Rear_Right;
  float Actual_Torque_Current_Rear_Left;
  //TO HERE->COMMENTED OUT IN LOGGING
  float Plate_Temp_Front_Right;
  float Plate_Temp_Front_Left;
  float Plate_Temp_Rear_Right;
  float Plate_Temp_Rear_Left;
  float IGBT_Temp_Front_Right;
  float IGBT_Temp_Front_Left;
  float IGBT_Temp_Rear_Right;
  float IGBT_Temp_Rear_Left;
  float Motor_Temp_Front_Right;
  float Motor_Temp_Front_Left;
  float Motor_Temp_Rear_Right;
  float Motor_Temp_Rear_Left;
  uint16_t Error_Info_Front_Right;
  uint16_t Error_Info_Front_Left;
  uint16_t Error_Info_Rear_Right;
  uint16_t Error_Info_Rear_Left;
  float Actual_Front_Right;
  float Actual_Front_Left;
  float Actual_Rear_Right;
  float Actual_Rear_Left;
  float Actual_Power_Front_Right;
  float Actual_Power_Front_Left;
  float Actual_Power_Rear_Right;
  float Actual_Power_Rear_Left;
  float Speed_From_RPM;
  float Speed_From_GPS;
  float Distance_Traveled_From_RPM; 
  float Distance_Traveled_From_GPS;
  bool Inverter_Derating;
  

  //DRS
  bool DRS;
  
  //BMS
  uint16_t Voltages[144];//BMWS_Raw
  uint16_t Temperatures[60];//BMWS_Raw
  uint8_t CAN[7];//BMWS_Raw
  uint16_t StackVoltage;
  uint8_t SOC;
  float Current;
 
  bool OverVoltage;
  bool UnderVoltage; 
  bool OverTempCells; 
  bool OverTempDies;
  bool UnderTempCells; 
  bool UnderTempDies;
  bool VoltageAbnormality; 
  bool TempAbnormality; 
  bool SystemSafe; 
  bool OverCharge; 
  bool OverDischarge;
  bool AbnormalityError; 
  bool VoltageSamplingAbnormality; 
  bool TemperatureSamplingAbnormality; 
  bool OverCurrent;
  bool UnderCurrent; 
  bool IVTSamplingAbnormality;
  bool IVTVoltageFault;
  bool IVTCurrentFault; 
  bool VoltageMismatch;
  bool CurrentMismatch;
  bool ChargingStatus;
  bool ShutdownStatus;
  bool BMSShutdown;
  bool IsPrecharged;
  bool TestsPassed;

  uint8_t Bad_Cell;
  uint8_t Bad_Thermistor;
  
  //extremes
  float Voltage_BMS_Min;  // uint16
  uint8_t Voltage_BMS_Min_Segment;
  uint8_t Voltage_BMS_Min_Cell;
  uint8_t Temp_BMS_Max_Segment;
  uint8_t Temp_BMS_Max_Cell;
  float Voltage_BMS_Max;  // uint16
  float Voltage_BMS_Mean;
  float Temp_BMS_Min;  // uint16
  float Temp_BMS_Max;  // uint16
  float Temp_BMS_Mean;


  //LV_AMS
  float Voltage;
  float Max_temp;
  float Min_temp;
  float Max_Cell_Voltage;
  float Min_Cell_Voltage;
  float Watt_drown;
  int   Status;

  float powerlimitgains_kp;
  float powerlimitgains_ki;
  float powerlimitgains_setpoint;

  float tvgains_kp;
  float tvgains_kp_traction;
  uint8_t tvgains_model;

  float tractioncontrol_A;
  float tractioncontrol_B;

  //IVT

  float IVT_Current;
  float IVT_Voltage1;
  float IVT_Voltage2;
  float IVT_Voltage3;
  float IVT_Temperature;
  float IVT_Power;
  float IVT_CurrentCounter;
  float IVT_EnergyCounter;
  unsigned long IVTLastUpdate;

  unsigned long INVLastUpdate;
  unsigned long LogInterval;

};

#endif
