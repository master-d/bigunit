#include <SDL3/SDL.h>
#include <box2d/box2d.h>
#include "data.h"
#include "physicsdata.h"

#define MTP 10.0f // meters to pixels

b2WorldId createb2World() {
        // 1. Initialize Box2D World
    b2WorldDef worldDef = b2DefaultWorldDef();
    worldDef.gravity = (b2Vec2){0.0f, 0.8f}; // Gravity points "down" in SDL coordinates
    return b2CreateWorld(&worldDef);
}
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
struct physics_data initPhysicsData() {
    struct physics_data pdata = pd_init(createb2World());

    b2BodyDef groundBodyDef = b2DefaultBodyDef();
    groundBodyDef.position = (b2Vec2){0.0f, 80.0f}; // 10 meters below origin
    b2BodyId groundId = b2CreateBody(pdata.worldId, &groundBodyDef);
    b2Polygon groundPolygon = b2MakeBox(50.0f, 10.0f);
    b2ShapeDef groundShapeDef = b2DefaultShapeDef();
    b2CreatePolygonShape(groundId, &groundShapeDef, &groundPolygon);
    pd_add_body(&pdata, groundId);

    int size = 10;
    for (int y = 0; y < size; y++) {
        for (int x=0; x < size; x++) {
            if (pship[y][x] != 0) {
                // Create a box at this position
                float posX = x + 5.0f; // Offset to center
                float posY = y + 2.0f; // Offset to center
                b2BodyId bodyId = createBox(pdata.worldId, posX, posY);
                pd_add_body(&pdata, bodyId);
            }
        }
    }
    return pdata;
}

int main(int argc, char* argv[]) {
    // 1. Initialize SDL3 (specifically the video subsystem)
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        SDL_LogError(SDL_LOG_CATEGORY_ERROR, "Couldnt initialize SDL: %s\n", SDL_GetError());
        return 1;
    }
    SDL_SetHint(SDL_HINT_VIDEO_X11_NET_WM_BYPASS_COMPOSITOR, "0");
    // 2. Create an application window
    SDL_Window* window = SDL_CreateWindow("Bigunit the game",1024,768,0);
    // Create the renderer to handle the background
    SDL_Renderer* renderer = SDL_CreateRenderer(window, NULL);
    SDL_SetRenderVSync(renderer, 1);
    // Check that the window was successfully created
     if (!window || !renderer) {
        SDL_Log("Initialization Error: %s", SDL_GetError());
        SDL_Quit();
        return 1;
    }

    // Initialize Physics Data
    struct physics_data pdata = initPhysicsData();
    bool done = false;
    while (!done) {
        SDL_Event event;
        // Poll for events and handle them
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_EVENT_QUIT) {
                done = true;
            }
        }        // --- RENDER SECTION ---
        // 3. Step Physics (approx 60fps)
        b2World_Step(pdata.worldId, 1.0f / 60.0f, 4);
        SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "%d bodies size %d\n", pdata.count, pdata.capacity);


        // --- RENDER ---
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);

        // Get Box position and draw it
        for (int i = 0; i < pdata.count; i++) {
            b2BodyId bodyId = pdata.bodyIds[i];
            b2Vec2 pos = b2Body_GetPosition(bodyId);
            SDL_FRect rect = {
                (pos.x - 0.5f) * MTP, // Top-left X
                (pos.y - 0.5f) * MTP, // Top-left Y
                1.0f * MTP,           // Width
                1.0f * MTP            // Height
            };

            SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255); // Green Box
            SDL_RenderFillRect(renderer, &rect);
        }
        SDL_RenderPresent(renderer);
    }
    // 4. Close and destroy the window, and clean up SDL
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
