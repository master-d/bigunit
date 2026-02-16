#pragma once

#include <stdint.h>

// Ship bitmap and metadata
typedef struct ship_data {
	// Pointer to pixel data as a flat array. Index with: pixels[row * width + col]
	const uint32_t *pixels;
	uint16_t w;  // width in pixels (columns)
	uint16_t h; // height in pixels (rows)
} ship_data;

extern const ship_data pship_data;
