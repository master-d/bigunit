#include "physicsdata.h"

// helper for building a body entry that has a single shape. Moves duplicated pbody
// initialization into one place so callers such as pd_createBox and
// pd_breakOffShape can reuse it.
static pbody pd_register_single_shape_body(physics_data* pdata,
                                           b2BodyId bodyId,
                                           b2ShapeId shapeId,
                                           bool controllable,
                                           bool breakable) {
    pbody body = {0};
    body.id = bodyId;
    body.controllable = controllable;
    body.breakable = breakable;
    body.shapeCount = 1;
    body.shapeCapacity = 1;
    body.shapeIds = malloc(sizeof(b2ShapeId));
    body.shapeOffsets = malloc(sizeof(b2Vec2));
    body.shapeIds[0] = shapeId;
    body.shapeOffsets[0] = (b2Vec2){0.0f, 0.0f};
    pd_add_body(pdata, body);
    return body;
}

pbody pd_createBox(physics_data* pdata, float x, float y) {
    // 2. Create a Dynamic Box
    b2BodyDef bodyDef = b2DefaultBodyDef();
    bodyDef.type = b2_dynamicBody;
    bodyDef.fixedRotation = false;
    bodyDef.position = (b2Vec2){x, y}; // Center-screen roughly
    b2BodyId bodyId = b2CreateBody(pdata->worldId, &bodyDef);

    b2Polygon box = b2MakeBox(0.5f, 0.5f); // 1m x 1m box (half-extents)
    b2ShapeDef shapeDef = b2DefaultShapeDef();
    shapeDef.density = 1.0f;
    // Enable contact events for this shape so the world will report collisions
    shapeDef.enableContactEvents = true;
    shapeDef.isSensor = false;
    b2ShapeId shapeId = b2CreatePolygonShape(bodyId, &shapeDef, &box);

    // Also enable contact events on the body (optional but ensures events are delivered)
    b2Body_EnableContactEvents(bodyId, true);

    // register and return the pbody
    return pd_register_single_shape_body(pdata, bodyId, shapeId, false, false);
}

pbody pd_createPolygon(physics_data* pdata, const ship_data* polydata, float x, float y, bool controllable, bool breakable) {
    b2BodyDef bodyDef = b2DefaultBodyDef();
    bodyDef.type = b2_dynamicBody;
    bodyDef.fixedRotation = false;
    bodyDef.position = (b2Vec2){x, y};
    b2BodyId bodyId = b2CreateBody(pdata->worldId, &bodyDef);
    b2Body_EnableContactEvents(bodyId, true);

    // Initialize shape array and offsets
    pbody outShapes = {0};
    outShapes.breakable = breakable;
    outShapes.controllable = controllable;
    outShapes.shapeIds = (b2ShapeId*)malloc(polydata->w * polydata->h * sizeof(b2ShapeId));
    outShapes.shapeOffsets = (b2Vec2*)malloc(polydata->w * polydata->h * sizeof(b2Vec2));
    outShapes.shapeCapacity = polydata->w * polydata->h;
    outShapes.shapeCount = 0;

    for (int px = 0; px < polydata->w; px++) {
        for (int py = 0; py < polydata->h; py++) {
            uint32_t pixel = polydata->pixels[py * polydata->w + px];
            if (pixel != 0) {
                // compute local offset in meters (polydata coords are in pixels)
                float localX = (px - polydata->w * 0.5f);
                float localY = (py - polydata->h * 0.5f);
                b2Polygon box = b2MakeBox(0.5f, 0.5f);
                
                // Translate the polygon vertices by the local offset
                for (int v = 0; v < box.count; v++) {
                    box.vertices[v].x += localX;
                    box.vertices[v].y += localY;
                }
                box.centroid = (b2Vec2){localX, localY};
                
                b2ShapeDef shapeDef = b2DefaultShapeDef();
                shapeDef.density = 1.0f;
                shapeDef.enableContactEvents = true;
                b2ShapeId shapeId = b2CreatePolygonShape(bodyId, &shapeDef, &box);
                outShapes.shapeIds[outShapes.shapeCount] = shapeId;
                outShapes.shapeOffsets[outShapes.shapeCount] = (b2Vec2){localX, localY};
                outShapes.shapeCount++;
            }
        }
    }

    outShapes.id = bodyId;
    pd_add_body(pdata, outShapes);
    return outShapes;
}

// Break off a single shape from a polygon body and make it independent.
// `normal` should point away from the remaining piece (i.e. the direction in
// which the new fragment should be pushed).  `speed` is the relative impact
// speed and is used to scale the impulse applied to the fragment.
b2BodyId pd_breakOffShape(physics_data* pdata, b2BodyId polygonBodyId, b2ShapeId shapeToBreak,
                           b2Vec2 normal, float speed) {
    // Get the original body's state
    b2Vec2 polygonPos = b2Body_GetPosition(polygonBodyId);
    b2Rot polygonRot = b2Body_GetRotation(polygonBodyId);
    b2Vec2 vel = b2Body_GetLinearVelocity(polygonBodyId);
    float angVel = b2Body_GetAngularVelocity(polygonBodyId);

    // Determine the local offset of the shape we're removing.
    b2Vec2 localOffset = {0,0};
    for (int i = 0; i < pdata->count; i++) {
        pbody* pb = &pdata->bodies[i];
        if (B2_ID_EQUALS(pb->id, polygonBodyId)) {
            for (int s = 0; s < pb->shapeCount; s++) {
                if (B2_ID_EQUALS(pb->shapeIds[s], shapeToBreak)) {
                    localOffset = pb->shapeOffsets[s];
                    break;
                }
            }
            break;
        }
    }

    // compute world position of the offset and use that for the new body
    b2Transform parentXf = b2Body_GetTransform(polygonBodyId);
    b2Vec2 worldPos = b2TransformPoint(parentXf, localOffset);

    b2BodyDef newBodyDef = b2DefaultBodyDef();
    newBodyDef.type = b2_dynamicBody;
    newBodyDef.position = worldPos;
    newBodyDef.rotation = polygonRot;
    b2BodyId newBodyId = b2CreateBody(pdata->worldId, &newBodyDef);
    b2Body_SetLinearVelocity(newBodyId, vel);
    b2Body_SetAngularVelocity(newBodyId, angVel);
    b2Body_EnableContactEvents(newBodyId, true);

    // Create the fragment shape on the new body
    b2Polygon box = b2MakeBox(0.5f, 0.5f);
    b2ShapeDef shapeDef = b2DefaultShapeDef();
    shapeDef.density = 1.0f;
    shapeDef.enableContactEvents = true;
    b2ShapeId newShape = b2CreatePolygonShape(newBodyId, &shapeDef, &box);

    // remove original shape from world and pdata
    b2DestroyShape(shapeToBreak, false);
    for (int i = 0; i < pdata->count; i++) {
        pbody* pb = &pdata->bodies[i];
        if (B2_ID_EQUALS(pb->id, polygonBodyId)) {
            for (int s = 0; s < pb->shapeCount; s++) {
                if (B2_ID_EQUALS(pb->shapeIds[s], shapeToBreak)) {
                    pb->shapeIds[s] = pb->shapeIds[pb->shapeCount - 1];
                    pb->shapeOffsets[s] = pb->shapeOffsets[pb->shapeCount - 1];
                    pb->shapeCount--;
                    break;
                }
            }
            break;
        }
    }

    pd_register_single_shape_body(pdata, newBodyId, newShape, false, false);

    // apply an outward impulse so the fragment doesn't stick in the parent
    b2Vec2 impulse = (b2Vec2){normal.x * speed, normal.y * speed};
    b2Body_ApplyLinearImpulse(newBodyId, impulse, worldPos, true);

    return newBodyId;
}


physics_data pd_init(b2WorldId worldId) {
    physics_data pdata;
    pdata.worldId = worldId;
    pdata.count = 0;
    pdata.capacity = 20; // Start small, grow as needed
    pdata.bodies = malloc(pdata.capacity * sizeof(pbody));
    return pdata;
}

void pd_add_body(physics_data* pdata, pbody body) {
    if (pdata->count == pdata->capacity) {
        // Double the capacity
        pdata->capacity *= 2;
        
        // Reallocate memory to the new size
        pbody* temp = (pbody*)realloc(pdata->bodies, pdata->capacity * sizeof(pbody));
        
        if (temp == NULL) {
            // Handle memory error here (e.g., exit or log)
            return;
        }
        pdata->bodies = temp;
    }

    // Add the new pbody and increment the count
    pdata->bodies[pdata->count] = body;
    pdata->count++;
}


void pd_cleanup(physics_data* pdata, uint32_t xres, uint32_t yres) {
    int i = 0;
    while (i < pdata->count) {
        b2BodyId bodyId = pdata->bodies[i].id;
        b2Vec2 pos = b2Body_GetPosition(bodyId);
        if (pos.x < -2 || pos.x > xres || pos.y < -2 || pos.y > yres) {
            // Free shape arrays before destroying body
            if (pdata->bodies[i].shapeIds) free(pdata->bodies[i].shapeIds);
            if (pdata->bodies[i].shapeOffsets) free(pdata->bodies[i].shapeOffsets);
            
            // Destroy the body in the Box2D world
            b2DestroyBody(bodyId);
            
            // Remove from array by swapping with last and reducing count
            pdata->bodies[i] = pdata->bodies[pdata->count - 1];
            pdata->count--;
        } else {
            i++;
        }
    }
}
void pd_free(physics_data* pdata) {
    // 1. (Optional) Loop through and b2DestroyBody() if needed
    
    // 2. Free the array memory
    free(pdata->bodies);
    pdata->bodies = NULL;
    pdata->count = 0;
    pdata->capacity = 0;
}