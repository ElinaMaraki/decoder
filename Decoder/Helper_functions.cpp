#include <cstdint>
#include <cstring>

uint8_t bitRead(uint8_t value, int bitNumber) {
    return (value >> bitNumber) & 1;
}

//COMPRESSION-DECOMPRESSION FUNCTIONS
//Voltage
uint16_t DecompressVoltage(uint8_t Voltage) {
	return (Voltage + 170) * 100;
}
//Temperature
float DecompressTemperature(uint8_t Temperature) {
	return (float)Temperature / 2.0F;
}

float DecompressTemperatureCoolant(uint16_t Temperature) {
    return (float)Temperature / 512.0f;
}
//END OF COMP-DEC.

//DECODE FUNCTIONS

float decode_uint64(uint64_t value,int startBit,int length){
	uint64_t  mask=0;
	uint8_t endBit = startBit + length -1;
  	for (uint8_t i=startBit; i<=endBit; i++){
  		mask |= (uint64_t) 1 << i;
	}
	uint64_t isolated = value & mask;
	float f = isolated>>startBit;
	return f;
}

void decodeMessage(uint8_t bytes[],uint8_t bitsPerVal[],size_t numVals,float out[]){
	uint64_t united;
	memcpy(&united,bytes,sizeof(united));

	uint8_t count=0;
	for (uint8_t i=0; i<numVals; i++){
			out[i]=decode_uint64(united,count,bitsPerVal[i]);
			count+=bitsPerVal[i];
	}
}
//END OF DECODE FUNC.
