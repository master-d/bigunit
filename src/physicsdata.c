#include "physicsdata.h"

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

void pd_free(struct physics_data* pdata) {
    // 1. (Optional) Loop through and b2DestroyBody() if needed
    
    // 2. Free the array memory
    free(pdata->bodyIds);
    pdata->bodyIds = NULL;
    pdata->count = 0;
    pdata->capacity = 0;
}