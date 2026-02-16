#include "data.h"

// Define and initialize the array
const uint32_t pship[10][10] = {
    {0,  0,  0,  0,  0xFF660000,  0xFF660000,  0,  0,  0,  0},
    {0,  0,  0,  0,  0xFF660000,  0xFF660000,  0,  0,  0,  0},
    {0,  0,  0,  0,  0xFF660000,  0xFF660000,  0,  0,  0,  0},
    {0,  0,  0,  0,  0xFF660000,  0xFF660000,  0,  0,  0,  0},
    {0,  0,  0,  0,  0xFF660000,  0xFF660000,  0,  0,  0,  0},
    {0,  0,  0,  0xFF660000,  0xFF660000,  0xFF660000,  0xFF660000,  0,  0,  0},
    {0,  0,  0xFF660000,  0xFF660000,  0xFF660000,  0xFF660000,  0xFF660000,  0xFF660000,  0,  0},
    {0,  0xFF660000,  0xFF660000,  0xFF660000,  0xFF660000,  0xFF660000,  0xFF660000,  0xFF660000,  0xFF660000,  0},
    {0xFF660000,  0xFF660000,  0xFF660000,  0xFF660000,  0xFF660000,  0xFF660000,  0xFF660000,  0xFF660000,  0xFF660000,  0xFF660000},
    {0,  0,  0,  0,  0xFF660000,  0xFF660000,  0,  0,  0,  0}
};

// Expose metadata
const ship_data pship_data = { .pixels = &pship[0][0], .w = 10, .h = 10 };