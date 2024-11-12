#include <cstdio>
#include <cinttypes>

#ifndef GLOBALDEF_H
#define GLOBALDEF_H

struct message {
	uint8_t dt;
	uint32_t id;
	uint8_t length;
	uint8_t buf[8];
};

extern struct My_Struct MyCar;

#endif
