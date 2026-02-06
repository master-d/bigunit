#pragma once

#include <stdlib.h>
#include <box2d/box2d.h>

#define MAX_BODIES 1000

struct physics_data {
    b2WorldId worldId;
    b2BodyId* bodyIds; // Pointer to the dynamically allocated array
    int count;         // Number of bodies currently stored
    int capacity;      // current capacity of the bodyIds array
};

struct physics_data pd_init(b2WorldId worldId);
void pd_add_body(struct physics_data* pdata, b2BodyId id);
void pd_free(struct physics_data* pdata);