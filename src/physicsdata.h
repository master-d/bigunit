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
b2BodyId pd_breakOffShape(physics_data* pdata, b2BodyId polygonBodyId, b2ShapeId shapeToBreak);
physics_data pd_init(b2WorldId worldId);
void pd_add_body(physics_data* pdata, pbody pbody);
void pd_create_welded_body(physics_data* pdata, const ship_data* wbody, float offsetX, float offsetY); // Create a welded body from the ship array
void pd_weld_bodies(physics_data* pdata, b2BodyId bodyA, b2BodyId bodyB, bool weldx);
void pd_free(physics_data* pdata);
void pd_cleanup(physics_data* pdata, uint32_t xres, uint32_t yres); // Remove bodies that are out of bounds