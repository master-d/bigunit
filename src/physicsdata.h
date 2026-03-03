#pragma once

#include <stdlib.h>
#include <box2d/box2d.h>
#include "data.h"

#define MAX_BODIES 1000

typedef struct {
    b2BodyId id;
    b2ShapeId* shapeIds;
    b2Vec2* shapeOffsets;  // Local position offset for each shape
    int shapeCount;
    int shapeCapacity;
    bool welded; // whether this body is part of a welded group
    bool breakable;
    bool controllable;
}  pbody;

typedef struct {
    b2WorldId worldId;
    pbody* bodies; // Pointer to the dynamically allocated array
    int count;         // Number of bodies currently stored
    int capacity;      // current capacity of the bodyIds array
} physics_data;


pbody pd_createBox(physics_data* pdata, float x, float y);
pbody pd_createPolygon(physics_data* pdata, const ship_data* polydata, float x, float y, bool controllable, bool breakable);

// Break off a single shape from a polygon body and make it independent.
// `normal` should point away from the remaining piece (i.e. the direction in
// which the new fragment should be pushed).  `speed` is the relative impact
// speed and is used to scale the impulse applied to the fragment.
b2BodyId pd_breakOffShape(physics_data* pdata, b2BodyId polygonBodyId, b2ShapeId shapeToBreak, b2Vec2 normal, float speed);
physics_data pd_init(b2WorldId worldId);
void pd_add_body(physics_data* pdata, pbody pbody);
void pd_free(physics_data* pdata);
void pd_cleanup(physics_data* pdata, uint32_t xres, uint32_t yres); // Remove bodies that are out of bounds