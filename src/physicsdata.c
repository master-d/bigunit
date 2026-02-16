#include "physicsdata.h"

b2BodyId createBox(b2WorldId worldId, float x, float y) {
        // 2. Create a Dynamic Box
    b2BodyDef bodyDef = b2DefaultBodyDef();
    bodyDef.type = b2_dynamicBody;
    bodyDef.fixedRotation = false;
    bodyDef.position = (b2Vec2){x, y}; // Center-screen roughly
    b2BodyId bodyId = b2CreateBody(worldId, &bodyDef);

    b2Polygon box = b2MakeBox(0.5f, 0.5f); // 1m x 1m box (half-extents)
    b2ShapeDef shapeDef = b2DefaultShapeDef();
    shapeDef.density = 1.0f;
    b2CreatePolygonShape(bodyId, &shapeDef, &box);    

    return bodyId;
}

struct physics_data pd_init(b2WorldId worldId) {
    struct physics_data pdata;
    pdata.worldId = worldId;
    pdata.count = 0;
    pdata.capacity = 20; // Start small, grow as needed
    pdata.bodyIds = malloc(pdata.capacity * sizeof(b2BodyId));
    return pdata;
}

void pd_add_body(struct physics_data* pdata, b2BodyId id) {
    if (pdata->count == pdata->capacity) {
        // Double the capacity
        pdata->capacity *= 2;
        
        // Reallocate memory to the new size
        b2BodyId* temp = (b2BodyId*)realloc(pdata->bodyIds, pdata->capacity * sizeof(b2BodyId));
        
        if (temp == NULL) {
            // Handle memory error here (e.g., exit or log)
            return;
        }
        pdata->bodyIds = temp;
    }

    // Add the new ID and increment the count
    pdata->bodyIds[pdata->count] = id;
    pdata->count++;
}
void pd_create_welded_body(struct physics_data* pdata, const ship_data* wbody, float offsetX, float offsetY) {
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
                b2BodyId bodyId = createBox(pdata->worldId, posX, posY);
                pd_add_body(pdata, bodyId);

                // store in grid for neighbor lookups
                grid[y * w + x] = bodyId;

                // Weld to left neighbor if it exists
                if (x > 0) {
                    b2BodyId left = grid[y * w + (x - 1)];
                    if (b2Body_IsValid(left))
                        pd_weld_bodies(pdata, left, bodyId, true);
                }

                // Weld to above neighbor if it exists
                if (y > 0) {
                    b2BodyId above = grid[(y - 1) * w + x];
                    if (b2Body_IsValid(above)) 
                        pd_weld_bodies(pdata, above, bodyId, false);
                }
            }
        }
    }

    free(grid);
}

void pd_weld_bodies(struct physics_data* pdata, b2BodyId bodyIdA, b2BodyId bodyIdB, bool weldx) {
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

void pd_cleanup(struct physics_data* pdata, uint32_t xres, uint32_t yres) {
    int i = 0;
    while (i < pdata->count) {
        b2BodyId bodyId = pdata->bodyIds[i];
        b2Vec2 pos = b2Body_GetPosition(bodyId);
        if (pos.x < -2 || pos.x > xres || pos.y < -2 || pos.y > yres) {
            // Destroy the body in the Box2D world
            b2DestroyBody(bodyId);
            // Remove from array by swapping with last and reducing count
            pdata->bodyIds[i] = pdata->bodyIds[pdata->count - 1];
            pdata->count--;
        } else {
            i++;
        }
    }
}
void pd_free(struct physics_data* pdata) {
    // 1. (Optional) Loop through and b2DestroyBody() if needed
    
    // 2. Free the array memory
    free(pdata->bodyIds);
    pdata->bodyIds = NULL;
    pdata->count = 0;
    pdata->capacity = 0;
}