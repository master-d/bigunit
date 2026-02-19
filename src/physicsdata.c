#include "physicsdata.h"

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
    b2ShapeId shapeId = b2CreatePolygonShape(bodyId, &shapeDef, &box);    

    // Also enable contact events on the body (optional but ensures events are delivered)
    // b2Body_EnableContactEvents(bodyId, true);

    pbody body = {0};
    body.id = bodyId;
    body.breakable = false;
    body.controllable = false;
    body.shapeCount = 1;
    body.shapeCapacity = 1;
    body.shapeIds = malloc(sizeof(b2ShapeId));
    body.shapeOffsets = malloc(sizeof(b2Vec2));
    body.shapeIds[0] = shapeId;
    body.shapeOffsets[0] = (b2Vec2){x,y};
    pd_add_body(pdata, body);
    return body;
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
                float localX = px - polydata->w * 0.5f;
                float localY = py - polydata->h * 0.5f;
                b2Polygon box = b2MakeBox(0.5f, 0.5f);
                box.centroid = (b2Vec2){localX, localY};
                b2ShapeDef shapeDef = b2DefaultShapeDef();
                shapeDef.density = 1.0f;
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

// Break off a single shape from a polygon body and make it independent
b2BodyId pd_breakOffShape(physics_data* pdata, b2BodyId polygonBodyId, b2ShapeId shapeToBreak) {
    // Get the original body's state
    b2Vec2 polygonPos = b2Body_GetPosition(polygonBodyId);
    b2Rot polygonRot = b2Body_GetRotation(polygonBodyId);
    b2Vec2 vel = b2Body_GetLinearVelocity(polygonBodyId);
    float angVel = b2Body_GetAngularVelocity(polygonBodyId);

    // Create new independent body
    b2BodyDef newBodyDef = b2DefaultBodyDef();
    newBodyDef.type = b2_dynamicBody;
    newBodyDef.position = polygonPos;
    newBodyDef.rotation = polygonRot;
    b2BodyId newBodyId = b2CreateBody(pdata->worldId, &newBodyDef);
    b2Body_SetLinearVelocity(newBodyId, vel);
    b2Body_SetAngularVelocity(newBodyId, angVel);
    b2Body_EnableContactEvents(newBodyId, true);

    // Create matching shape on new body
    b2Polygon box = b2MakeBox(0.5f, 0.5f);
    b2ShapeDef shapeDef = b2DefaultShapeDef();
    shapeDef.density = 1.0f;
    b2CreatePolygonShape(newBodyId, &shapeDef, &box);

    // Destroy shape from original body
    b2DestroyShape(shapeToBreak, false);

    // Remove shape from physics_data array of the original body
    for (int i = 0; i < pdata->count; i++) {
        pbody* pb = &pdata->bodies[i];
        if (B2_ID_EQUALS(pb->id, polygonBodyId)) {
            // Find and remove this shape from the array
            for (int s = 0; s < pb->shapeCount; s++) {
                if (B2_ID_EQUALS(pb->shapeIds[s], shapeToBreak)) {
                    // Swap with last and shrink
                    pb->shapeIds[s] = pb->shapeIds[pb->shapeCount - 1];
                    pb->shapeOffsets[s] = pb->shapeOffsets[pb->shapeCount - 1];
                    pb->shapeCount--;
                    break;
                }
            }
            break;
        }
    }

    // Add to physics data as non-welded
    pd_add_body(pdata, (pbody){ .id = newBodyId, .welded = false, .controllable = false, .breakable = false });

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
void pd_create_welded_body(physics_data* pdata, const ship_data* wbody, float offsetX, float offsetY) {
    const uint16_t w = wbody->w;
    const uint16_t h = wbody->h;

    // Temporary grid mapping [row * w + col] -> b2BodyId (0 means no body)
    b2BodyId* grid = calloc(w * h, sizeof(b2BodyId));
    if (!grid) return; // allocation failed

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            uint32_t pixel = wbody->pixels[y * w + x];
            if (pixel != 0) {
                // Create a box at this position
                float posX = x + offsetX;
                float posY = y + offsetY;
                pbody body = pd_createBox(pdata, posX, posY);

                // store in grid for neighbor lookups
                grid[y * w + x] = body.id;

                // Weld to left neighbor if it exists
                if (x > 0) {
                    b2BodyId left = grid[y * w + (x - 1)];
                    if (b2Body_IsValid(left))
                        pd_weld_bodies(pdata, left, body.id, true);
                }

                // Weld to above neighbor if it exists
                if (y > 0) {
                    b2BodyId above = grid[(y - 1) * w + x];
                    if (b2Body_IsValid(above)) 
                        pd_weld_bodies(pdata, above, body.id, false);
                }
            }
        }
    }

    free(grid);
}

void pd_weld_bodies(physics_data* pdata, b2BodyId bodyIdA, b2BodyId bodyIdB, bool weldx) {
    b2WeldJointDef weldDef = b2DefaultWeldJointDef();
    weldDef.linearDampingRatio = 1.0f;
    weldDef.linearHertz = 0.0f;
    weldDef.angularDampingRatio = 1.0f;
    weldDef.angularHertz = 0.0f;
    weldDef.collideConnected = false;
    weldDef.bodyIdA = bodyIdA;
    weldDef.bodyIdB = bodyIdB;
    if (weldx) {
        weldDef.localAnchorA = (b2Vec2){1.0f, 0.0f};
        weldDef.localAnchorB = (b2Vec2){0.0f, 0.0f};
    } else {
        weldDef.localAnchorA = (b2Vec2){0.0f, 1.0f};
        weldDef.localAnchorB = (b2Vec2){0.0f, 0.0f};
    }
    b2CreateWeldJoint(pdata->worldId, &weldDef);
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