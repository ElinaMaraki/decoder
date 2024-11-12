#include "Global_definitions.h"
#include "CAR.h"
#include "CAN.h"
#include "Helper_functions.cpp"
#include "PCAN-GPS.h"
#include "PCAN-GPS.cpp"

//BMS
bool CANParse_BMSVoltages(struct message msg) {
    // If message ID corresponds to the BMS voltages range
    if (msg.id >= CAN_BMS_VOLTAGES_BASE && msg.id <= CAN_BMS_VOLTAGES_END) {
        // Calculate the cell's index based on the  ID
        uint8_t CellIndex = (msg.id - CAN_BMS_VOLTAGES_BASE) * 8;
        // Put all voltages of the buffer to the local struct
        for (uint8_t Index = 0; Index < msg.length; Index++) {
            MyCar.Voltages[CellIndex + Index] = DecompressVoltage(msg.buf[Index]);
        }
        return true;
    }
    return false;
}

bool CANParse_BMSTemperatures(struct message msg) {
    // If message ID corresponds to the BMS temperatures range
    if (msg.id >= CAN_BMS_TEMPERATURES_BASE && msg.id <= CAN_BMS_TEMPERATURES_END) {
        uint8_t CellIndex = (msg.id - CAN_BMS_TEMPERATURES_BASE) * 8;
        // Put all thermistors of the buffer to the local struct
        for (uint8_t Index = 0; Index < msg.length; Index++) {
            MyCar.Temperatures[CellIndex + Index] = DecompressTemperature(msg.buf[Index]);
        }
        return true;
    }
    return false;
}
//END OF BMS

//INVERTER
void CANParse_INVERTER(struct message msg) {

    bool actValues1Sent =
        msg.id == INVERTER_FR_ACTUAL_VALUES1 || msg.id == INVERTER_RR_ACTUAL_VALUES1 ||
        msg.id == INVERTER_FL_ACTUAL_VALUES1 || msg.id == INVERTER_RL_ACTUAL_VALUES1;
    bool actValues2Sent =
        msg.id == INVERTER_FR_ACTUAL_VALUES2 || msg.id == INVERTER_RR_ACTUAL_VALUES2 ||
        msg.id == INVERTER_FL_ACTUAL_VALUES2 || msg.id == INVERTER_RL_ACTUAL_VALUES2;
    bool specMessSent =
        msg.id == INVERTER_FR_SPECIAL_MESSAGE || msg.id == INVERTER_RR_SPECIAL_MESSAGE ||
        msg.id == INVERTER_FL_SPECIAL_MESSAGE || msg.id == INVERTER_RL_SPECIAL_MESSAGE;

    if (actValues1Sent) {
        int16_t speed;
        uint8_t d1, d2;
        float actTorqueCurr,actMagnCurr;

        // float RPM;
        // WARNING!: NOT SURE IF (float) CASTING WILL WORK
        // maybe change types to uint16_t ar Car.h
        // or transform it in other way
        // check link: https://forum.arduino.cc/t/convert-uint16-to-float32/179730

        //get last bit of d1
        MyCar.Inverter_Derating = (bool)(msg.buf[1]>>7 & 1);

        d1 = msg.buf[2];
        d2 = msg.buf[3];
        speed = ((int16_t)d2 << 8) | d1;
        d1 = msg.buf[4];
        d2 = msg.buf[5];
        actTorqueCurr = ((int16_t)d2 << 8) | d1;
        actTorqueCurr = (float)actTorqueCurr *107.3 / 16384;

        d1 = msg.buf[6];
        d2 = msg.buf[7];
        actMagnCurr = ((int16_t)d2 << 8) | d1;
        actMagnCurr = (float)actMagnCurr * 107.3/16384;



        switch ((int)msg.id){
        case INVERTER_FR_ACTUAL_VALUES1:
           MyCar.RPM_Front_Right = speed;
           MyCar.Actual_Magnetizing_Current_Front_Right = actMagnCurr;
           MyCar.Actual_Torque_Current_Front_Right = actTorqueCurr;
            break;
        case INVERTER_FL_ACTUAL_VALUES1:
           MyCar.RPM_Front_Left = speed;  // & 0x7FFF;
           MyCar.Actual_Magnetizing_Current_Front_Left = actMagnCurr;
           MyCar.Actual_Torque_Current_Front_Left = actTorqueCurr;
        case INVERTER_RR_ACTUAL_VALUES1:
           MyCar.RPM_Rear_Right = speed;
           MyCar.Actual_Magnetizing_Current_Rear_Right = actMagnCurr;
           MyCar.Actual_Torque_Current_Rear_Right = actTorqueCurr;
            break;
        case INVERTER_RL_ACTUAL_VALUES1:
           MyCar.RPM_Rear_Left = speed;  // 0x7FFF;
           MyCar.Actual_Magnetizing_Current_Rear_Left = actMagnCurr;
           MyCar.Actual_Torque_Current_Rear_Left = actTorqueCurr;
            break;
        }

    } else if (actValues2Sent) {
        uint8_t d1, d2;

        int16_t tempMotor, tempInverter, tempIGBT;
        uint16_t errorInfo;

        d1 = msg.buf[0];
        d2 = msg.buf[1];
        tempMotor = ((int16_t)d2 << 8) | d1;

        d1 = msg.buf[2];
        d2 = msg.buf[3];
        tempInverter = ((int16_t)d2 << 8) | d1;

        d1 = msg.buf[4];
        d2 = msg.buf[5];
        errorInfo = ((uint16_t)d2 << 8) | d1;

        d1 = msg.buf[6];
        d2 = msg.buf[7];
        tempIGBT = ((int16_t)d2 << 8) | d1;

        switch ((int)msg.id){
        case INVERTER_FR_ACTUAL_VALUES2:
           MyCar.Plate_Temp_Front_Right = (float)tempInverter * 0.1;  // unit: 0.1 °C
           MyCar.IGBT_Temp_Front_Right = (float)tempIGBT * 0.1;       // unit: 0.1 °C
           MyCar.Motor_Temp_Front_Right = (float)tempMotor * 0.1;     // unit: 0.1 °C
           MyCar.Error_Info_Front_Right = errorInfo;
            break;
        case INVERTER_FL_ACTUAL_VALUES2:
           MyCar.Plate_Temp_Front_Left = (float)tempInverter * 0.1;  // unit: 0.1 °C
           MyCar.IGBT_Temp_Front_Left = (float)tempIGBT * 0.1;       // unit: 0.1 °C
           MyCar.Motor_Temp_Front_Left = (float)tempMotor * 0.1;     // unit: 0.1 °C
           MyCar.Error_Info_Front_Left = errorInfo;
            break;
        case INVERTER_RR_ACTUAL_VALUES2:
           MyCar.Plate_Temp_Rear_Right = (float)tempInverter * 0.1;  // unit: 0.1 °C
           MyCar.IGBT_Temp_Rear_Right = (float)tempIGBT * 0.1;       // unit: 0.1 °C
           MyCar.Motor_Temp_Rear_Right = (float)tempMotor * 0.1;     // unit: 0.1 °C
           MyCar.Error_Info_Rear_Right = errorInfo;
            break;
        case INVERTER_RL_ACTUAL_VALUES2:
           MyCar.Plate_Temp_Rear_Left = (float)tempInverter * 0.1;  // unit: 0.1 °C
           MyCar.IGBT_Temp_Rear_Left = (float)tempIGBT * 0.1;       // unit: 0.1 °C
           MyCar.Motor_Temp_Rear_Left = (float)tempMotor * 0.1;     // unit: 0.1 °C
           MyCar.Error_Info_Rear_Left = errorInfo;
            break;
        }

    } else if (specMessSent) {
        // ta pairno apo inveter special messages
        int16_t val;
        uint8_t d1,d2,d3,d4;
        float actTorque,actPower;

        d1 = msg.buf[4];  // dont know the offset,Velou must define it
        d2 = msg.buf[5];  // dont know the offset,Velou must define it
        val = ((int16_t)d2 << 8) | d1;

        actTorque = ( (float)val ) / 1000.0 * 9.8;  // 9.8=Nominal Torque

        // read first 4 bytes into a float
        d1 = msg.buf[0];
        d2 = msg.buf[1];
        d3 = msg.buf[2];
        d4 = msg.buf[3];

        val =  ((int16_t)d4 << 24) | ((int16_t)d3 << 16) | ((int16_t)d2 << 8) | d1;
        actPower = ( (float)val ) / 1000.0;  // kW


        switch ((int)msg.id){
        case INVERTER_FR_SPECIAL_MESSAGE:
           MyCar.Actual_Front_Right = actTorque;
           MyCar.Actual_Power_Front_Right = actPower;
           //printf("actual fr= %f , actual power fr= %f\n",MyCar.Actual_Front_Right, MyCar.Actual_Power_Front_Right);
        break;
        case INVERTER_FL_SPECIAL_MESSAGE:
           MyCar.Actual_Front_Left = actTorque;
           MyCar.Actual_Power_Front_Left = actPower;
           //printf("actual fl= %f , actual power fl= %f\n",MyCar.Actual_Front_Left, MyCar.Actual_Power_Front_Left);
        break;
        case INVERTER_RR_SPECIAL_MESSAGE:
           MyCar.Actual_Rear_Right = actTorque;
           MyCar.Actual_Power_Rear_Right = actPower;
            //printf("actual fl= %f , actual power fl= %f\n",MyCar.Actual_Rear_Right, MyCar.Actual_Power_Rear_Right);
        break;
        case INVERTER_RL_SPECIAL_MESSAGE:
           MyCar.Actual_Rear_Left = actTorque;
           MyCar.Actual_Power_Rear_Left = actPower;
           //printf("actual fl= %f , actual power fl= %f\n",MyCar.Actual_Rear_Left, MyCar.Actual_Power_Rear_Left);
        break;
        }
    }

    // calculate RPMs and Distance Traveled
    float combinedRPM = (abs(MyCar.RPM_Rear_Left) + abs(MyCar.RPM_Rear_Right)) / 2.0;
    MyCar.Speed_From_RPM = 0.0059015*combinedRPM; // km/h
    MyCar.Speed_From_GPS = GPS1.Speed();
    //printf("GPS=%f\n", MyCar.Speed_From_GPS);
    float dt = MyCar.INVLastUpdate/3.6e6;
    //millis to hours 
    MyCar.Distance_Traveled_From_RPM +=MyCar.Speed_From_RPM * dt;
    MyCar.Distance_Traveled_From_GPS +=MyCar.Speed_From_GPS * dt;

    MyCar.INVLastUpdate = 0;
    //printf("Speed_From_GPS= %d ,Speed_From_RPM= %d, combinedRPM= %f\n",MyCar.RPM_Rear_Left , MyCar.RPM_Rear_Right, combinedRPM );
}
//END OF INVERTER

//IVT
void CANParse_IVT(struct message msg){
    MyCar.IVTLastUpdate=0;
    uint8_t MuxByte = msg.buf[0] >> 4;
    if (MuxByte == 0x00) {
        int32_t Value = (msg.buf[2] << 24) | (msg.buf[3] << 16) |
                        (msg.buf[4] << 8) | msg.buf[5];
        switch (msg.id){
        case CAN_IVT_CURRENT:
            MyCar.IVT_Current = (float) Value / 1000.0;
            break;
        case CAN_IVT_VOLTAGE:
            MyCar.IVT_Voltage1= (float) Value / 1000.0;
            break;
        case IVT_Msg_Result_U2: //added
            MyCar.IVT_Voltage2 = (float) Value / 1000.0;
            break;
        case IVT_Msg_Result_U3: //added
            MyCar.IVT_Voltage3= (float) Value / 1000.0;
            break;
        case IVT_Msg_Result_T: //added
            MyCar.IVT_Temperature = (float) Value / 10.0;
            break;
        case IVT_Msg_Result_W:
            MyCar.IVT_Power = (float) Value;
            break;
        case IVT_Msg_Result_As:
            MyCar.IVT_CurrentCounter = (float) Value / 3600.0;
            break;
        case IVT_Msg_Result_Wh: //added
            MyCar.IVT_EnergyCounter = (float) Value;
            break;
        }
    }
};
//END OF IVT

void Init_Decode(){
    // Ιnitialize struct values for ecu
    MyCar.Preset = Skidpad;
    MyCar.TV = TV_Off;
    MyCar.TC = TC_Off;
    MyCar.Regen = REGEN_Off;
    MyCar.Power_Limit = Off;
    MyCar.left_steering_calibration = 0;
    MyCar.right_steering_calibration = 0;
    // Εnd of values
    MyCar.INVLastUpdate=0;//might have issue with first inverter message
    MyCar.IVTLastUpdate=0;
    MyCar.LogTime =0;
    MyCar.LogInterval = 0;
}

void Decoder(struct message msg){
    MyCar.LogTime += msg.dt;
    MyCar.INVLastUpdate += msg.dt;
    MyCar.IVTLastUpdate +=msg.dt;
    MyCar.LogInterval +=msg.dt;
    CANParse_BMSVoltages(msg);
    CANParse_BMSTemperatures(msg);

    switch ((int)msg.id){
        //REAR NODE
        case CAN_BACK_NODE_SUSPENSION:
            memcpy(&MyCar.Suspension_Rear_Left, &msg.buf[0],
                   sizeof(MyCar.Suspension_Rear_Left));
            memcpy(&MyCar.Suspension_Rear_Right, &msg.buf[4],
                   sizeof(MyCar.Suspension_Rear_Right));
            break;
        case CAN_BACK_NODE_NTC:
            memcpy(&MyCar.Transmission_Rear_Left, &msg.buf[0],
                   sizeof(MyCar.Transmission_Rear_Left));
            memcpy(&MyCar.Transmission_Rear_Right, &msg.buf[4],
                   sizeof(MyCar.Transmission_Rear_Right));
            break;
        case CAN_BACK_NODE_SUSPENSION_RAW:
            memcpy(&MyCar.RawSuspension_Rear_Left,&msg.buf[0],
                   sizeof(MyCar.RawSuspension_Rear_Left));
            memcpy(&MyCar.RawSuspension_Rear_Right, &msg.buf[2],
                   sizeof(MyCar.RawSuspension_Rear_Right));
            break;
        case CAN_BACK_NODE_NTC_RAW:
            memcpy(&MyCar.RawTransmission_Rear_Left, &msg.buf[0], 2);
            memcpy(&MyCar.RawTransmission_Rear_Right, &msg.buf[2], 2);
            break;
        case CAN_BACK_NODE_COOLANT:
            uint16_t Coolant_In_1_temp, Coolant_Out_1_temp, Coolant_In_2_temp, Coolant_Out_2_temp;

            memcpy(&Coolant_In_1_temp, &msg.buf[0], 2);
            memcpy(&Coolant_Out_1_temp, &msg.buf[2], 2);
            memcpy(&Coolant_In_2_temp, &msg.buf[4], 2);
            memcpy(&Coolant_Out_2_temp, &msg.buf[6], 2);

            MyCar.Coolant_In_1 = DecompressTemperatureCoolant(Coolant_In_1_temp);
            MyCar.Coolant_Out_1 = DecompressTemperatureCoolant(Coolant_Out_1_temp);
            MyCar.Coolant_In_2 = DecompressTemperatureCoolant(Coolant_In_2_temp);
            MyCar.Coolant_Out_2 = DecompressTemperatureCoolant(Coolant_Out_2_temp);


            break;
            //FRONT NODE
        case CAN_FRONT_NODE_BRAKES:
            MyCar.Brake_Front = msg.buf[0];
            MyCar.Brake_Rear = msg.buf[1];
            MyCar.Brake_Travel = msg.buf[2] / 2.0f;
            MyCar.Brake_Bias = MyCar.Brake_Front / (MyCar.Brake_Front + MyCar.Brake_Rear) * 100.0f;
            break;
        case CAN_FRONT_NODE_TPS:
            MyCar.Throttle = msg.buf[0]/2.0;
            MyCar.Throttle_5 = msg.buf[1];
            MyCar.Throttle_12 = msg.buf[2];
            break;
        case CAN_FRONT_NODE_NTC:
            memcpy(&MyCar.Transmission_Front_Left, &msg.buf[0],
                   sizeof(MyCar.Transmission_Front_Left));
            memcpy(&MyCar.Transmission_Front_Right, &msg.buf[4],
                   sizeof(MyCar.Transmission_Front_Right));
            break;
        case CAN_FRONT_NODE_SUSPENSION:
            memcpy(&MyCar.Suspension_Front_Left, &msg.buf[0],
                   sizeof(MyCar.Suspension_Front_Left));
            memcpy(&MyCar.Suspension_Front_Right, &msg.buf[4],
                   sizeof(MyCar.Suspension_Front_Right));
            break;
        case CAN_FRONT_NODE_TPS_RAW:
            memcpy(&MyCar.RawThrottle1, &msg.buf[0], 2);
            memcpy(&MyCar.RawThrottle2, &msg.buf[2], 2);
            break;
        case CAN_FRONT_NODE_BRAKES_RAW:
            memcpy(&MyCar.RawBrakeTravel, &msg.buf[0], 2);
            memcpy(&MyCar.RawBrakeFront, &msg.buf[2], 2);
            memcpy(&MyCar.RawBrakeRear, &msg.buf[4], 2);
            break;
        case CAN_FRONT_NODE_NTC_RAW:
            memcpy(&MyCar.RawTransmission_Front_Left, &msg.buf[0], 2);
            memcpy(&MyCar.RawTransmission_Front_Right, &msg.buf[2], 2);
            break;
        case CAN_FRONT_NODE_SUSPENSION_RAW:
            memcpy(&MyCar.RawSuspension_Front_Left, &msg.buf[0], 2);
            memcpy(&MyCar.RawSuspension_Front_Right, &msg.buf[2], 2);
            break;
            //ECU
        case CAN_ECU_FLAGS:
            MyCar.RTD = bitRead(msg.buf[0], 0);
            MyCar.BrakeLight = bitRead(msg.buf[0], 1);
            MyCar.Buzzer = bitRead(msg.buf[0], 2);
            MyCar.EnablePumps = bitRead(msg.buf[0], 3);
            MyCar.EnableFans = bitRead(msg.buf[0], 4);
            MyCar.gkazofreno = bitRead(msg.buf[0], 5);
            break;
        case CAN_ECU_TV:
            MyCar.Steering_Wheel_Angle = ( (float)msg.buf[0] ) - 120;//float but STA int?
            MyCar.Desired_Yaw_Rate = ( (float) msg.buf[1] ) - 120;
            MyCar.Torque_Demand = ( (float) msg.buf[2] ) / 3.0;
            MyCar.Regen_Torque_Demand = ( (float) msg.buf[3]) * -1 / 3.0;
            MyCar.Vehicle_Velocity_from_RPM = ( (float)msg.buf[4] ) / 2.0;
            MyCar.FL_Torque =( (float)msg.buf[5] ) / 10.0;
            MyCar.FR_Torque =( (float)msg.buf[6] ) / 10.0;
            MyCar.RL_Torque =( (float)msg.buf[7] ) / 10.0;
            break;
        case CAN_ECU_TV2:
            MyCar.RR_Torque = (float)msg.buf[0] / 10.0;
            MyCar.TV_error = (float)msg.buf[1];
            MyCar.Steering_Wheel_Rate = ( ( (float) msg.buf[2] ) - 125 ) * 10;
            memcpy(&MyCar.Steering_Wheel_Angle_Raw, &msg.buf[3], 2);
            break;
        case INV_DATA_TO_LOGGER_TRQ_DEMANDS_CAN_ID:
            MyCar.Demanded_Front_Right = (float)msg.buf[0] / 12;
            MyCar.Demanded_Front_Left  = (float)msg.buf[1] / 12;
            MyCar.Demanded_Rear_Right  = (float)msg.buf[2] / 12;
            MyCar.Demanded_Rear_Left   = (float)msg.buf[3] / 12;
            MyCar.Regen_Demanded_Front_Right = (float)msg.buf[4] / 12;
            MyCar.Regen_Demanded_Front_Left  = (float)msg.buf[5] / 12;
            MyCar.Regen_Demanded_Rear_Right  = (float)msg.buf[6] / 12;
            MyCar.Regen_Demanded_Rear_Left   = (float)msg.buf[7] / 12;
            break;

        case CAN_ECU_DRIVER_GET_STATE:
            MyCar.Preset =static_cast<ECU_Mode>(msg.buf[0]);
            MyCar.TV = static_cast<ECU_TV_State>((msg.buf[1] & 0xF0) >> 4);
            MyCar.TC = static_cast<ECU_TC_State>(msg.buf[1] & 0x0F);
            MyCar.Regen = static_cast<ECU_Regen_State>(msg.buf[2]);
            MyCar.Power_Limit = static_cast<ECU_Power_Limit_State>(msg.buf[3]);
            memcpy(&MyCar.left_steering_calibration, &msg.buf[4], 2);
            memcpy(&MyCar.right_steering_calibration, &msg.buf[6], 2);
            break;

            //DRS
        case CAN_DRS_STATE:
            MyCar.DRS = (bool)msg.buf[0];
            break;

            //BMS
        case CAN_BMS_INFORMATION:
            // Store the incoming message buffer to the local struct
            for (uint8_t Index = 0; Index < msg.length; Index++) {
                MyCar.CAN[Index] = msg.buf[Index];
            }

            // Extract and put the total stack voltage into the local struct
            MyCar.StackVoltage = ((((uint16_t)(MyCar.CAN[0])) << 8) | MyCar.CAN[1]);

            // Extract and put the BMS flags into the local struct
            MyCar.OverVoltage = bitRead(MyCar.CAN[2], 7);
            MyCar.UnderVoltage = bitRead(MyCar.CAN[2], 6);
            MyCar.OverTempCells = bitRead(MyCar.CAN[2], 5);
            MyCar.OverTempDies = bitRead(MyCar.CAN[2], 4);
            MyCar.UnderTempCells = bitRead(MyCar.CAN[2], 3);
            MyCar.UnderTempDies = bitRead(MyCar.CAN[2], 2);
            MyCar.VoltageAbnormality = bitRead(MyCar.CAN[2], 1);
            MyCar.TempAbnormality = bitRead(MyCar.CAN[2], 0);

            MyCar.SystemSafe = bitRead(MyCar.CAN[3], 7);
            MyCar.OverCharge = bitRead(MyCar.CAN[3], 6);
            MyCar.OverDischarge = bitRead(MyCar.CAN[3], 5);
            MyCar.AbnormalityError = bitRead(MyCar.CAN[3], 4);
            MyCar.VoltageSamplingAbnormality = bitRead(MyCar.CAN[3], 3);
            MyCar.TemperatureSamplingAbnormality = bitRead(MyCar.CAN[3], 2);
            MyCar.OverCurrent = bitRead(MyCar.CAN[3], 1);
            MyCar.UnderCurrent = bitRead(MyCar.CAN[3], 0);

            MyCar.IVTSamplingAbnormality = bitRead(MyCar.CAN[4], 7);
            MyCar.IVTVoltageFault = bitRead(MyCar.CAN[4], 6);
            MyCar.IVTCurrentFault = bitRead(MyCar.CAN[4], 5);
            MyCar.VoltageMismatch = bitRead(MyCar.CAN[4], 4);
            MyCar.CurrentMismatch = bitRead(MyCar.CAN[4], 3);
            MyCar.ChargingStatus = bitRead(MyCar.CAN[4], 2);
            MyCar.ShutdownStatus = bitRead(MyCar.CAN[4], 1);
            MyCar.BMSShutdown = bitRead(MyCar.CAN[4], 0);

            MyCar.IsPrecharged = bitRead(MyCar.CAN[5], 1);
            MyCar.TestsPassed = bitRead(MyCar.CAN[5], 0);

            MyCar.SOC = MyCar.CAN[6];
            //printf("BMS\n Stack Voltage:%" PRIu16 "\nOver Voltage:%d \nUnder Voltage:%d \nOver Temp Cells:%d \nOver Temp Dies:%d \nUnder Temp Cells:%d \nUnder Temp Dies:%d \nVoltage Abnormality:%d \nTemp Abnormality:%d \nSystem Safe:%d \nOver Charge:%d \nOver Discharge:%d \nAbnormality Error:%d \nVolt Sampl Abnormality:%d \nTemp Samp Abnormality:%d \nOver Current:%d \nUnder Current:%d \nIVT Samp Abn:%d \nIVT Volt Fault:%d \nIVT Cur Fault:%d \nVoltage Missmatch:%d \nCurrent Missmatch:%d \nCharging Status:%d \nSD Status:%d \nBMS SD:%d \nIs Precharged:%d \nTest Passed:%d \n SOC:%" PRIu8 "\n", MyCar.StackVoltage , MyCar.OverVoltage,  MyCar.UnderVoltage,  MyCar.OverTempCells,  MyCar.OverTempDies,  MyCar.UnderTempCells, MyCar.UnderTempDies, MyCar.VoltageAbnormality, MyCar.TempAbnormality, MyCar.SystemSafe, MyCar.OverCharge, MyCar.OverDischarge, MyCar.AbnormalityError, MyCar.VoltageSamplingAbnormality, MyCar.TemperatureSamplingAbnormality, MyCar.OverCurrent, MyCar.UnderCurrent, MyCar.IVTSamplingAbnormality,  MyCar.IVTVoltageFault, MyCar.IVTCurrentFault, MyCar.VoltageMismatch, MyCar.CurrentMismatch, MyCar.ChargingStatus, MyCar.ShutdownStatus, MyCar.BMSShutdown, MyCar.IsPrecharged, MyCar.TestsPassed,MyCar.SOC);
            break;

        case CAN_BMS_EXTREMES:
        {
            MyCar.Temp_BMS_Mean = DecompressTemperature(msg.buf[3]);
            float min = 42000.0;
            float max = 17000.0;
            uint8_t min_index = 0;
            uint8_t max_index = 0;
            for (uint8_t i = 0; i < TOTAL_CELLS; i++) {
                if (MyCar.Voltages[i] < min) {
                    min = MyCar.Voltages[i];
                    MyCar.Voltage_BMS_Min = min / 10000.0;
                    min_index = i;
                }
                if (MyCar.Voltages[i] > max) {
                    max = MyCar.Voltages[i];
                    MyCar.Voltage_BMS_Max = max / 10000.0;
                    max_index = i;
                }
            }
            uint8_t segment = min_index / 12;
            uint8_t cell = min_index % 12;
            MyCar.Voltage_BMS_Min_Segment=segment;
            MyCar.Voltage_BMS_Min_Cell=cell;

            MyCar.Voltage_BMS_Mean = (MyCar.StackVoltage / 100.0f) / TOTAL_CELLS;
            min = 10000.0;
            max = 0.0;
            for (uint8_t i = 0; i < TOTAL_THERMISTORS; i++) {
                if (i == 36 || i == 43 || i == 55) continue;
                if (MyCar.Temperatures[i] < min) {
                    min = MyCar.Temperatures[i];
                    MyCar.Temp_BMS_Min = min;
                    min_index = i;
                }
                if (MyCar.Temperatures[i] > max) {
                    max = MyCar.Temperatures[i];
                    MyCar.Temp_BMS_Max = max;
                    max_index = i;
                }
            }
            segment = max_index / 12;
            cell = max_index % 12;
            MyCar.Temp_BMS_Max_Segment=segment;
            MyCar.Temp_BMS_Max_Cell=cell;
            //printf("TEMP BMS MIN:%f TEMP BMS MAX:%f\n",MyCar.Temp_BMS_Min, MyCar.Temp_BMS_Max);
            //printf("VOLT BMS MIN:%f VOLT BMS MAX:%f\n",MyCar.Voltage_BMS_Min, MyCar.Voltage_BMS_Max);
            //printf("Volt BMS MIN Seg:%" PRIu8 "\n Volt BMS MIN Cell:%" PRIu8 "\n",MyCar.Voltage_BMS_Min_Segment, MyCar.Voltage_BMS_Min_Cell);
            //printf("TEMP BMS MAX Seg:%" PRIu8 "\n TEMP BMS MAX Cell:%" PRIu8 "\n",MyCar.Temp_BMS_Max_Segment, MyCar.Temp_BMS_Max_Cell);

            break;
        }

        case CAN_BMS_BAD_CELL:
            MyCar.Bad_Cell = msg.buf[0];
            //printf("BMS Bad Cell:%" PRIu8 "\n",MyCar.Bad_Cell);
            break;

        case CAN_BMS_BAD_THERMISTOR:
            MyCar.Bad_Thermistor = msg.buf[0];
            //printf("BMS Bad Thermistor:%" PRIu8 "\n",MyCar.Bad_Thermistor);
            break;

        case CAN_BMS_CURRENTS:
            // Extract all the bytes from the message and store the current value
            // Car.BMS.Current = ((int32_t)(Msg.buf[0]) << 0) | ((int32_t)(Msg.buf[1]) << 8) |
            //                   ((int32_t)(Msg.buf[2] << 16)) | (int32_t)(Msg.buf[3] << 24);
            memcpy(&MyCar.Current, &msg.buf[0], sizeof(float));
            //printf("BMS Current:%f\n",MyCar.Current);
            break;

        case LV_BMS_CAN_ID:
        {
            uint8_t bitsperval[] ={10,8,8,11,11,10,3};
            float out[7];
            decodeMessage(msg.buf, bitsperval, 7, out);

            //transformations according to Logger Excel
            MyCar.Voltage  = out[0]/25.6 ;
            MyCar.Min_temp = out[1] ;
            MyCar.Max_temp = out[2] ;
            MyCar.Min_Cell_Voltage = out[3]/1024 + 2.5 ;
            MyCar.Max_Cell_Voltage = out[4]/1024 + 2.5 ;
            MyCar.Watt_drown = out[5]/1.7 ;
            MyCar.Status = out[6] ;
            //printf("LVS BMS:\n Voltage:%f\n Min Temp:%f\n Max Temp:%f\n Min Cell Voltage:%f\n Max Cell Voltage:%f\n Watt Drown:%f\n Status:%d\n",MyCar.Voltage, MyCar.Min_temp, MyCar.Max_temp, MyCar.Min_Cell_Voltage, MyCar.Max_Cell_Voltage, MyCar.Watt_drown, MyCar.Status);
            break;
        }
            //INVERTER
        case INVERTER_FR_ACTUAL_VALUES1:
        case INVERTER_FR_ACTUAL_VALUES2:
        case INVERTER_FR_SPECIAL_MESSAGE:
        case INVERTER_RR_ACTUAL_VALUES1:
        case INVERTER_RR_ACTUAL_VALUES2:
        case INVERTER_RR_SPECIAL_MESSAGE:
        case INVERTER_FL_ACTUAL_VALUES1:
        case INVERTER_FL_ACTUAL_VALUES2:
        case INVERTER_FL_SPECIAL_MESSAGE:
        case INVERTER_RL_ACTUAL_VALUES1:
        case INVERTER_RL_ACTUAL_VALUES2:
        case INVERTER_RL_SPECIAL_MESSAGE:
            CANParse_INVERTER(msg);
            break;
        case CAN_IVT_CURRENT:
        case CAN_IVT_VOLTAGE:
        case IVT_Msg_Result_U2:
        case IVT_Msg_Result_U3:
        case IVT_Msg_Result_T:
        case IVT_Msg_Result_W:
        case IVT_Msg_Result_As:
        case IVT_Msg_Result_Wh:
            CANParse_IVT(msg);
            break;
        case CAN_PCAN_Acceleration:
        case CAN_PCAN_MagneticField:
        case CAN_PCAN_RotationA:
        case CAN_PCAN_RotationB:
        case CAN_PCAN_GPS_Status:
        case CAN_PCAN_GPS_CourseSpeed:
        case CAN_PCAN_GPS_PositionLongitude:
        case CAN_PCAN_GPS_PositionLatitude:
        case CAN_PCAN_GPS_Altitude:
        case CAN_PCAN_DelusionsA:
        case CAN_PCAN_DelusionsB:
        case CAN_PCAN_GPS_DateTime:
        case CAN_PCAN_IO:
        case CAN_PCAN_RTC_DateTime:
            GPS1.Process(msg);
            break;

        case TUNING_CAN_ID:
            switch ((uint8_t)msg.buf[0]){
                case 0:
                    MyCar.powerlimitgains_kp = 0.1 * (float)(msg.buf[2] << 8 | msg.buf[1]);
                    MyCar.powerlimitgains_ki= 0.1 * (float)(msg.buf[4] << 8 | msg.buf[3]);
                    MyCar.powerlimitgains_setpoint = 10 * (float)(msg.buf[6] << 8 | msg.buf[5]);
                    break;
                case 1:
                    MyCar.tvgains_kp = 0.01 * (float)(msg.buf[2] << 8 | msg.buf[1]);
                    MyCar.tvgains_kp_traction = 0.01 * (float)(msg.buf[4] << 8 | msg.buf[3]);
                    break;
                case 2:
                    MyCar.tractioncontrol_A = 0.001 * (float)(msg.buf[1] << 8 | msg.buf[2]);
                    break;
                case 3:
                    MyCar.tractioncontrol_B = 0.001 * (float)(msg.buf[3] << 8 | msg.buf[4]);
                    break;
            }


            break;
    }
}