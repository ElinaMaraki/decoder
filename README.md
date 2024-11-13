--File Contents:
Parser.cpp: main function 
Decoder.cpp: Decoder function, that based on recieved message's IDs decodes message and saves it to corresponding CAR.h variable
Helper_functions.h: Helper functions for decoding process
CAR.h: contains logging variables
CAN.h: contains CAN Bus messages' IDs
PCAN.h, PCAN.cpp, Types.h: Functions and Libraries for PCAN GPS decoding(used from previous logger)
Serialization.cpp: Function for variable serialization, needed for looging variables in a predetermined order
data.txt: File extracted for teensy board, contains decoded messages
---------------------------------------------------------------
Parser.exe: executable file if full Decoder used in GUI.
