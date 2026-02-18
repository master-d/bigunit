#pragma once

#include <stdlib.h>
#include <box2d/box2d.h>
#include "data.h"

#define MAX_BODIES 1000

struct physics_data {
    b2WorldId worldId;
    struct pbody* bodies; // Pointer to the dynamically allocated array
    int count;         // Number of bodies currently stored
    int capacity;      // current capacity of the bodyIds array
};

struct pbody {
    b2BodyId id;
    bool welded; // whether this body is part of a welded group
};

b2BodyId createBox(b2WorldId worldId, float x, float y);
struct physics_data pd_init(b2WorldId worldId);
void pd_add_body(struct physics_data* pdata, struct pbody pbody);
void pd_create_welded_body(struct physics_data* pdata, const ship_data* wbody, float offsetX, float offsetY); // Create a welded body from the ship array
void pd_weld_bodies(struct physics_data* pdata, b2BodyId bodyA, b2BodyId bodyB, bool weldx);
void pd_free(struct physics_data* pdata);
void pd_cleanup(struct physics_data* pdata, uint32_t xres, uint32_t yres); // Remove bodies that are out of bounds