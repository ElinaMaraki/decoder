#include <cstdio>
#include <iostream>
#include <fstream>
#include "Serialization.cpp"
#include "Decoder.cpp"
#include "Global_definitions.h"

struct My_Struct MyCar = {0};

int main(int argc, char *argv[]){
    Init_Decode();
    struct message msg = {0};


    // Extract command-line arguments
    const char *input_file_path = argv[1];
    int threshold_value = std::stoi(argv[2]);  // Convert string to integer
    std::string output_file_name = argv[3];

    std::ifstream input_file(input_file_path);

    if (!input_file.is_open()) {
        std::cerr << "Failed to open input file: " << input_file_path << "\n";
        return 1;
    }

    std::ofstream csvFile(output_file_name);
    if (!csvFile.is_open()) {
        std::cerr << "Failed to create output file: " << output_file_name << "\n";
        return 1;
    }

    FILE *fp;
    fp = fopen(input_file_path,"rb");

    InitLogin(csvFile);

    if (!fp) {
        std::cerr << "Failed to open input file: " << input_file_path << "\n";
        return 1;
    }

    while(!feof(fp)){
        fread(&(msg.dt),1,1,fp);
        //printf("time: %u \n", msg.dt);

        fread(&(msg.length),1,1,fp);
        //printf("length: %u \n", msg.length);

        fread(&(msg.id),4,1,fp);
       // printf("0x%x, " ,msg.id);
        
        fread(&(msg.buf[0]),msg.length,1,fp);
        
        //for (int i=0; i<msg.length; i++) {
        //printf("%X ", msg.buf[i]);
        //}
        //printf("\n");

        Decoder(msg);

        if (MyCar.LogInterval >= threshold_value){
            std::string LogStr = SerializeSensors();
            csvFile << LogStr << std::endl;
            MyCar.LogInterval=0;
	    }
    }
    fclose(fp);
    csvFile.close();

    return 0;
}
