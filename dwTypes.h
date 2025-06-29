#ifndef DWTYPES_H
#define DWTYPES_H

#include <stdint.h>

typedef union {
	uint8_t raw[5];
	uint64_t full;
	struct {
		uint32_t low32;
		uint8_t high8;
	} __attribute__((packed));
	struct {
		uint8_t low8;
		uint32_t high32;
	} __attribute__((packed));
} dwTime_t;			// 8 byte

#endif