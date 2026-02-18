#include <SDL3/SDL.h>
#include <box2d/box2d.h>
#include "data.h"
#include "physicsdata.h"
#include <stdbool.h>
#include <math.h>
#include <pthread.h>
#include <stdlib.h>

// Persistent thread-pool implementation for Box2D tasks.
struct pool_job {
    b2TaskCallback* task;
    int start;
    int end;
    uint32_t workerIndex;
    void* taskContext;
    struct pool_job* next;
    struct pool_work* parent;
};

struct pool_work {
    int remaining;
    pthread_mutex_t mutex;
    pthread_cond_t cond;
};

struct thread_pool {
    pthread_t* threads;
    int threadCount;
    struct pool_job* head;
    struct pool_job* tail;
    pthread_mutex_t queue_mutex;
    pthread_cond_t queue_cond;
    bool running;
};

static struct thread_pool g_pool = {0};

static void pool_push_job(struct pool_job* job) {
    pthread_mutex_lock(&g_pool.queue_mutex);
    job->next = NULL;
    if (g_pool.tail) g_pool.tail->next = job;
    else g_pool.head = job;
    g_pool.tail = job;
    pthread_cond_signal(&g_pool.queue_cond);
    pthread_mutex_unlock(&g_pool.queue_mutex);
}

static struct pool_job* pool_pop_job(void) {
    struct pool_job* job = NULL;
    pthread_mutex_lock(&g_pool.queue_mutex);
    while (g_pool.running && g_pool.head == NULL) {
        pthread_cond_wait(&g_pool.queue_cond, &g_pool.queue_mutex);
    }
    if (!g_pool.running) {
        pthread_mutex_unlock(&g_pool.queue_mutex);
        return NULL;
    }
    job = g_pool.head;
    if (job) {
        g_pool.head = job->next;
        if (!g_pool.head) g_pool.tail = NULL;
    }
    pthread_mutex_unlock(&g_pool.queue_mutex);
    return job;
}

static void* pool_worker(void* vp) {
    (void)vp;
    while (g_pool.running) {
        struct pool_job* job = pool_pop_job();
        if (!job) break;
        job->task(job->start, job->end, job->workerIndex, job->taskContext);
        // signal completion
        pthread_mutex_lock(&job->parent->mutex);
        job->parent->remaining -= 1;
        if (job->parent->remaining == 0) pthread_cond_signal(&job->parent->cond);
        pthread_mutex_unlock(&job->parent->mutex);
        free(job);
    }
    return NULL;
}

static void pool_init(int threads) {
    if (g_pool.running) return;
    g_pool.threadCount = threads > 0 ? threads : 1;
    g_pool.threads = malloc(sizeof(pthread_t) * g_pool.threadCount);
    pthread_mutex_init(&g_pool.queue_mutex, NULL);
    pthread_cond_init(&g_pool.queue_cond, NULL);
    g_pool.head = g_pool.tail = NULL;
    g_pool.running = true;
    for (int i = 0; i < g_pool.threadCount; i++) pthread_create(&g_pool.threads[i], NULL, pool_worker, NULL);
}

static void pool_shutdown(void) {
    if (!g_pool.running) return;
    pthread_mutex_lock(&g_pool.queue_mutex);
    g_pool.running = false;
    pthread_cond_broadcast(&g_pool.queue_cond);
    pthread_mutex_unlock(&g_pool.queue_mutex);
    for (int i = 0; i < g_pool.threadCount; i++) pthread_join(g_pool.threads[i], NULL);
    free(g_pool.threads);
    g_pool.threads = NULL;
    g_pool.threadCount = 0;
    pthread_mutex_destroy(&g_pool.queue_mutex);
    pthread_cond_destroy(&g_pool.queue_cond);
}

// Box2D enqueue/finish callbacks using persistent thread-pool
static void* box2d_enqueue_task(b2TaskCallback* task, int itemCount, int minRange, void* taskContext, void* userContext) {
    int workerCount = userContext ? *(int*)userContext : g_pool.threadCount;
    if (!g_pool.running || workerCount <= 1 || itemCount <= 1) {
        task(0, itemCount, 0, taskContext);
        return NULL;
    }
    int threads = workerCount;
    if (threads > itemCount) threads = itemCount;

    struct pool_work* work = malloc(sizeof(struct pool_work));
    pthread_mutex_init(&work->mutex, NULL);
    pthread_cond_init(&work->cond, NULL);
    work->remaining = threads;

    for (int t = 0; t < threads; t++) {
        int start = (itemCount * t) / threads;
        int end = (itemCount * (t + 1)) / threads;
        struct pool_job* job = malloc(sizeof(struct pool_job));
        job->task = task;
        job->start = start;
        job->end = end;
        job->workerIndex = (uint32_t)t;
        job->taskContext = taskContext;
        job->parent = work;
        job->next = NULL;
        pool_push_job(job);
    }
    return work;
}

static void box2d_finish_task(void* userTask, void* userContext) {
    (void)userContext;
    if (!userTask) return;
    struct pool_work* work = (struct pool_work*)userTask;
    pthread_mutex_lock(&work->mutex);
    while (work->remaining > 0) pthread_cond_wait(&work->cond, &work->mutex);
    pthread_mutex_unlock(&work->mutex);
    pthread_mutex_destroy(&work->mutex);
    pthread_cond_destroy(&work->cond);
    free(work);
}

#define MTP 10.0f // meters to pixels
#define XRES 1920
#define YRES 1080
#define XGRAVITY 0.0f
#define YGRAVITY 0.0f

b2WorldId createb2World() {
        // 1. Initialize Box2D World
    static int g_worker_count = 8; // number of threads for Box2D
    // initialize persistent pool
    pool_init(g_worker_count);

    b2WorldDef worldDef = b2DefaultWorldDef();
    worldDef.gravity = (b2Vec2){XGRAVITY, YGRAVITY};
    worldDef.workerCount = g_worker_count;
    worldDef.enqueueTask = box2d_enqueue_task;
    worldDef.finishTask = box2d_finish_task;
    worldDef.userTaskContext = &g_worker_count;

    b2WorldId worldId = b2CreateWorld(&worldDef);
    b2World_SetHitEventThreshold(worldId, 0.0f); 

    return worldId;
}

struct physics_data initPhysicsData() {
    struct physics_data pdata = pd_init(createb2World());

    // b2BodyDef groundBodyDef = b2DefaultBodyDef();
    // groundBodyDef.position = (b2Vec2){0.0f, 80.0f}; // 10 meters below origin
    // b2BodyId groundId = b2CreateBody(pdata.worldId, &groundBodyDef);
    // b2Polygon groundPolygon = b2MakeBox(50.0f, 10.0f);
    // b2ShapeDef groundShapeDef = b2DefaultShapeDef();
    // b2CreatePolygonShape(groundId, &groundShapeDef, &groundPolygon);
    // pd_add_body(&pdata, groundId);

    pd_create_welded_body(&pdata, &pship_data, 50.0f, 50.0f);
    return pdata;
}

int main(int argc, char* argv[]) {
    // 1. Initialize SDL3 (specifically the video subsystem)
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        SDL_LogError(SDL_LOG_CATEGORY_ERROR, "Couldn't initialize SDL: %s\n", SDL_GetError());
        return 1;
    }
    SDL_SetHint(SDL_HINT_VIDEO_X11_NET_WM_BYPASS_COMPOSITOR, "0");
    // 2. Create an application window
    SDL_Window* window = SDL_CreateWindow("Bigunit the game",XRES,YRES,0);
    // Create the renderer to handle the background
    SDL_Renderer* renderer = SDL_CreateRenderer(window, NULL);
    if (renderer) SDL_SetRenderVSync(renderer, 1);
    // Check that the window was successfully created
     if (!window || !renderer) {
        SDL_Log("Initialization Error: %s", SDL_GetError());
        SDL_Quit();
        return 1;
    }

    // Create a 1x1 white texture we can scale and rotate when rendering boxes
    SDL_Texture* boxTex = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_STATIC, 1, 1);
    if (boxTex) {
        Uint32 white = 0xFFFFFFFFu;
        SDL_UpdateTexture(boxTex, NULL, &white, sizeof(white));
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
            } else if (event.type == SDL_EVENT_KEY_DOWN) {
                switch (event.key.key) {
                case SDLK_ESCAPE:
                    done = true;
                    break;
                case SDLK_W: {
                    // Move ship up - find first welded body
                    for (int i = 0; i < pdata.count; i++) {
                        if (pdata.bodies[i].welded) {
                            b2Body_ApplyForceToCenter(pdata.bodies[i].id, (b2Vec2){0.0f, -1000.0f}, true);
                            break;
                        }
                    }
                    break;
                }
                case SDLK_S: {
                    // Move ship down - find first welded body
                    for (int i = 0; i < pdata.count; i++) {
                        if (pdata.bodies[i].welded) {
                            b2Body_ApplyForceToCenter(pdata.bodies[i].id, (b2Vec2){0.0f, 1000.0f}, true);
                            break;
                        }
                    }
                    break;
                }
                case SDLK_A: {
                    // Move ship left - find first welded body
                    for (int i = 0; i < pdata.count; i++) {
                        if (pdata.bodies[i].welded) {
                            b2Body_ApplyForceToCenter(pdata.bodies[i].id, (b2Vec2){-1000.0f, 0.0f}, true);
                            break;
                        }
                    }
                    break;
                }
                case SDLK_D: {
                    // Move ship right - find first welded body
                    for (int i = 0; i < pdata.count; i++) {
                        if (pdata.bodies[i].welded) {
                            b2Body_ApplyForceToCenter(pdata.bodies[i].id, (b2Vec2){1000.0f, 0.0f}, true);
                            break;
                        }
                    }
                    break;
                }
                case SDLK_SPACE: {
                    // Add a new box at random position near the top
                    float x = (rand() % 80) + 10; // Random x between 10 and 90
                    float y = 10.0f; // Start near the top
                    b2BodyId bodyId = createBox(pdata.worldId, x, y);
                    pd_add_body(&pdata, (struct pbody){ .id = bodyId, .welded = false });
                    b2Body_ApplyForceToCenter(bodyId, (b2Vec2){(rand() % 10000) - 5000, (rand() % 10000) - 5000}, true); // Apply an initial upward force
                    break;
                }
                default:
                    break;
                }
            }
        }
        // 3. Step Physics (approx 60fps)
        b2World_Step(pdata.worldId, 1.0f / 60.0f, 1);
        // SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "%d bodies size %d\n", pdata.count, pdata.capacity);


        // clear screen to black
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);

        // Render each body on the main thread (single-threaded)
        pd_cleanup(&pdata, XRES/MTP, YRES/MTP); // Remove bodies that are out of bounds
        int count = pdata.count;
        // SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "%d bodies", count);

        // 6. Check for collisions/contacts ONCE per frame (before rendering)
        b2ContactEvents contactEvents = b2World_GetContactEvents(pdata.worldId); 
        
        // Use BEGIN contact events (which ARE firing)
        for (int j = 0; j < contactEvents.beginCount; j++) {
            b2ContactBeginTouchEvent* ev = &contactEvents.beginEvents[j];
            b2ShapeId shapeA = ev->shapeIdA;
            b2ShapeId shapeB = ev->shapeIdB;
            
            b2BodyId bodyA = b2Shape_GetBody(shapeA);
            b2BodyId bodyB = b2Shape_GetBody(shapeB);
            
            // Get relative velocity at contact point for impact speed
            b2Vec2 velA = b2Body_GetLinearVelocity(bodyA);
            b2Vec2 velB = b2Body_GetLinearVelocity(bodyB);
            b2Vec2 relVel = (b2Vec2){velA.x - velB.x, velA.y - velB.y};
            float approachSpeed = (float)sqrt(relVel.x * relVel.x + relVel.y * relVel.y);
            
            // SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "BEGIN EVENT: relative speed %.2f", approachSpeed);
            
            // Check if either body is in our welded set
            for (int i = 0; i < count; i++) {
                struct pbody pb = pdata.bodies[i];
                if (!pb.welded) continue;  // Only care about welded bodies
                
                if (B2_ID_EQUALS(bodyA, pb.id) || B2_ID_EQUALS(bodyB, pb.id)) {
                    int jointCount = b2Body_GetJointCount(pb.id);
                    SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "Welded body contact with speed %.2f, joints: %d", approachSpeed, jointCount);
                    
                    if (approachSpeed > 5.0f && jointCount > 0) {
                        // Remove weld joints on contact
                        b2JointId* joints = (b2JointId*)malloc(jointCount * sizeof(b2JointId));
                        int actualCount = b2Body_GetJoints(pb.id, joints, jointCount);
                        for (int k = 0; k < actualCount; k++) {
                            b2DestroyJoint(joints[k]);
                        }
                        free(joints);
                        SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "Destroyed %d weld joints", actualCount);
                    }
                    break;
                }
            }
        }

        for (int i = 0; i < count; i++) {
            struct pbody pb = pdata.bodies[i];
            b2Vec2 pos = b2Body_GetPosition(pb.id);

            b2Transform xf = b2Body_GetTransform(pb.id);
            float ang = b2Rot_GetAngle(xf.q);
            float w = MTP;
            float h = MTP;
            SDL_FRect dst = { pos.x * MTP - w * 0.5f, pos.y * MTP - h * 0.5f, w, h };
            double angle_deg = ang * 180.0 / M_PI;
            if (boxTex) {
                SDL_SetTextureColorMod(boxTex, 255, 255, 0);
                SDL_FPoint center = { dst.w * 0.5f, dst.h * 0.5f };
                SDL_RenderTextureRotated(renderer, boxTex, NULL, &dst, angle_deg, &center, SDL_FLIP_NONE);
            } else {
                SDL_SetRenderDrawColor(renderer, 255, 255, 0, 255);
                SDL_RenderFillRect(renderer, &dst);
            }
        }

        SDL_RenderPresent(renderer);
    }
    // 4. Close and destroy the window, and clean up SDL
    pool_shutdown();
    pd_free(&pdata);
    if (boxTex) SDL_DestroyTexture(boxTex);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
