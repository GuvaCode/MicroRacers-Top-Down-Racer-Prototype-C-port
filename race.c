/*******************************************************************************
  MicroRacers (Top-Down Racer Prototype) — C port
********************************************************************************
  Physics model — 1:1 with the original SkiaMicroRacers:
    - Single scalar Speed per car.
    - Car always moves exactly along its heading (no slip, no drift).
    - Steering is proportional to Speed (no turning in place).
    - Reverse works by making Speed negative via the brake key.
    - Surface (asphalt/grass) modifies Accel and Friction only.

  Angle handling:
    - Car.Angle is always kept in [0, 360).

  Camera:
    - raylib Camera2D + GetWorldToScreen2D.
    - Target smoothly follows the player.

  Track generation:
    - Star-shaped closed curve in polar coordinates:
        R(θ) = BaseR * (1 + Σ Ai·cos(Ki·θ + Pi)), integer Ki.
    - Regenerated until the max turn between adjacent segments is
      below MAX_ALLOWED_TURN.

  AI navigation:
    - Waypoints followed in order.
    - A waypoint is "passed" when the car is close enough OR has
      projected ahead of it along the track.
    - Fallback: force-switch after AI_WP_TIMEOUT seconds.
    - Speed-dependent switch radius prevents overshoot.
    - Look-ahead braking slows the AI into sharp corners.

  Rendering:
    - Each car: coloured body quad + black rectangular cabin.
    - Track: shoulder + asphalt + solid centre line + chequered start.

  Author of original code in Pascal: Lara Miriam Tamy Reschke
  Rewrite to raylib and C code Vadim Gunko @GuvaCode
  License: MIT
*******************************************************************************/

#include "raylib.h"
#include "raymath.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/* ============================ Constants ============================ */

#define CAR_SIZE        25.0f
#define CAR_VISUAL_LEN  30.0f
#define CAR_VISUAL_W    20.0f

/* Difficulty scaling */
#define LEVEL_STEP      0.06f
#define LEVEL_MAX_MUL   1.60f
#define PLAYER_BONUS    5.0f

/* Base AI (level 1) — slower than the player */
#define AI_BASE_SPEED   270.0f
#define AI_BASE_ACCEL   420.0f

/* AI waypoint following */
#define AI_WP_MIN_RADIUS    70.0f
#define AI_WP_SPEED_FACTOR  0.30f
#define AI_OFFSET_LIMIT     15.0f
#define AI_WP_TIMEOUT       2.0f

/* AI look-ahead braking */
#define AI_CORNER_START     15.0f
#define AI_CORNER_FULL      90.0f
#define AI_CORNER_MIN_MUL   0.50f

/* Track visual widths */
#define SHOULDER_EXTRA      40.0f
#define CENTRE_LINE_W       6.0f

/* Chequered start/finish strip */
#define FINISH_CELLS        12
#define FINISH_ROWS         3

/* Track generator */
#define NUM_POINTS          32
#define BASE_RADIUS         720.0f
#define WORLD_CENTER_X      1500.0f
#define WORLD_CENTER_Y      1500.0f
#define MAX_ALLOWED_TURN    24.0f
#define MAX_GEN_ATTEMPTS    80

/* ============================ Types ============================ */

typedef enum {
    GS_READY,
    GS_COUNTDOWN,
    GS_RACING,
    GS_FINISHED
} GameState;

typedef struct {
    float X, Y;
    float Angle;
} TrackPoint;

typedef struct {
    int    TargetIndex;
    float  X, Y;
    float  Angle;
    float  Speed;
    int    LapsCompleted;
    char   Name[32];
    Color  Color;
    bool   IsPlayer;
    bool   CanCountLap;
    bool   Finished;
    int    FinalRank;
    float  Offset;
    float  WaypointTimer;
} Car;

/* ============================ Globals ============================ */

#define MAX_CARS 4

static Car        gCars[MAX_CARS];
static int        gCarCount = 0;
static Car       *gPlayerCar = NULL;

static TrackPoint gTrackPoints[NUM_POINTS];
static int        gTrackCount = NUM_POINTS;
static float      gTrackWidth = 180.0f;
static int        gStartIndex = 0;

static GameState  gGameState = GS_READY;
static float      gCountdownTimer = 0.0f;
static float      gRaceTimer = 0.0f;
static int        gTotalLaps = 3;
static int        gLevel = 1;
static int        gWins = 0;

static Camera2D   gCamera;

/* input flags */
static bool gKeyLeft = false;
static bool gKeyRight = false;
static bool gKeyUp = false;
static bool gKeyDown = false;

/* ============================ Helpers ============================ */

static float NormalizeAngle360(float a)
{
    while (a < 0.0f)    a += 360.0f;
    while (a >= 360.0f) a -= 360.0f;
    return a;
}

static float LevelMul(void)
{
    float m = 1.0f + (float)(gLevel - 1) * LEVEL_STEP;
    if (m > LEVEL_MAX_MUL) m = LEVEL_MAX_MUL;
    return m;
}

static float RandFloat(void)
{
    return (float)rand() / (float)RAND_MAX;
}

/* ============================ Car ============================ */

static void CarReset(Car *c, float ax, float ay, float aangle)
{
    c->X = ax;
    c->Y = ay;
    c->Angle = NormalizeAngle360(aangle);
    c->Speed = 0.0f;
    c->LapsCompleted = 0;
    c->CanCountLap = false;
    c->Finished = false;
    c->FinalRank = 0;
    c->WaypointTimer = 0.0f;
}

/* ============================ Track generation ============================ */

static void GenerateTrack(void)
{
    const int K1 = 2, K2 = 3, K3 = 5;
    int   attempt = 0;
    bool  ok = false;
    float noiseA1, noiseA2, noiseA3;
    float noiseP1, noiseP2, noiseP3;
    int   i;

    while (!ok && attempt < MAX_GEN_ATTEMPTS)
    {
        attempt++;
        ok = true;

        noiseA1 = 0.07f + RandFloat() * 0.05f;
        noiseA2 = 0.04f + RandFloat() * 0.03f;
        noiseA3 = 0.02f + RandFloat() * 0.02f;

        noiseP1 = RandFloat() * 2.0f * PI;
        noiseP2 = RandFloat() * 2.0f * PI;
        noiseP3 = RandFloat() * 2.0f * PI;

        for (i = 0; i < NUM_POINTS; i++)
        {
            float theta = 2.0f * PI * (float)i / (float)NUM_POINTS;
            float r = BASE_RADIUS * (1.0f
                + noiseA1 * cosf(K1 * theta + noiseP1)
                + noiseA2 * cosf(K2 * theta + noiseP2)
                + noiseA3 * cosf(K3 * theta + noiseP3));

            gTrackPoints[i].X = WORLD_CENTER_X + cosf(theta) * r;
            gTrackPoints[i].Y = WORLD_CENTER_Y + sinf(theta) * r;
        }

        for (i = 0; i < NUM_POINTS; i++)
        {
            int n = (i + 1) % NUM_POINTS;
            Vector2 d = Vector2Subtract(
                (Vector2){ gTrackPoints[n].X, gTrackPoints[n].Y },
                (Vector2){ gTrackPoints[i].X, gTrackPoints[i].Y });
            gTrackPoints[i].Angle = RAD2DEG * atan2f(d.y, d.x);
        }

        for (i = 0; i < NUM_POINTS; i++)
        {
            float aA = gTrackPoints[i].Angle;
            float aB = gTrackPoints[(i + 1) % NUM_POINTS].Angle;
            float diff = fabsf(aB - aA);
            if (diff > 180.0f) diff = 360.0f - diff;

            if (diff > MAX_ALLOWED_TURN)
            {
                ok = false;
                break;
            }
        }
    }

    /* Find the straightest segment to use as start */
    {
        int   bestIdx = 0;
        float bestTurn = 99999.0f;

        for (i = 0; i < NUM_POINTS; i++)
        {
            float aA = gTrackPoints[i].Angle;
            float aB = gTrackPoints[(i + 1) % NUM_POINTS].Angle;
            float diff = fabsf(aB - aA);
            if (diff > 180.0f) diff = 360.0f - diff;

            if (diff < bestTurn)
            {
                bestTurn = diff;
                bestIdx = i;
            }
        }
        gStartIndex = bestIdx;
    }
}

/* ============================ Race init ============================ */

static void InitRace(void)
{
    int i;

    gTrackWidth = 180.0f;
    GenerateTrack();

    gCarCount = 0;

    TrackPoint startPt = gTrackPoints[gStartIndex];
    float startAng = startPt.Angle;

    float dirX  = cosf(DEG2RAD * startAng);
    float dirY  = sinf(DEG2RAD * startAng);
    float perpX = -dirY;
    float perpY =  dirX;

    Color colors[4];
    colors[0] = (Color){ 255,   0,   0, 255 }; /* player  — red    */
    colors[1] = (Color){   0,   0, 255, 255 }; /* AI 1    — blue   */
    colors[2] = (Color){   0, 255,   0, 255 }; /* AI 2    — green  */
    colors[3] = (Color){ 255, 255,   0, 255 }; /* AI 3    — yellow */

    for (i = 0; i < 4; i++)
    {
        Car *c = &gCars[gCarCount++];
        memset(c, 0, sizeof(Car));

        if (i == 0) {
            strcpy(c->Name, "Player");
            c->IsPlayer = true;
        } else {
            snprintf(c->Name, sizeof(c->Name), "AI %d", i);
            c->IsPlayer = false;
        }
        c->Color = colors[i];

        float alongOffset   = -i * 45.0f;
        float lateralOffset = ((i % 2) * 40.0f) - 20.0f;

        CarReset(c,
            startPt.X + dirX * alongOffset + perpX * lateralOffset,
            startPt.Y + dirY * alongOffset + perpY * lateralOffset,
            startAng);

        c->Offset = ((i * 40) % 100) - 50;
        if (c->Offset >  AI_OFFSET_LIMIT) c->Offset =  AI_OFFSET_LIMIT;
        if (c->Offset < -AI_OFFSET_LIMIT) c->Offset = -AI_OFFSET_LIMIT;

        c->TargetIndex = (gStartIndex + 1) % NUM_POINTS;
        c->WaypointTimer = 0.0f;
    }

    gPlayerCar = &gCars[0];
    gCamera.target = (Vector2){ gPlayerCar->X, gPlayerCar->Y };
    gCamera.rotation = 0.0f;
    gCamera.zoom = 1.0f;

    gGameState = GS_READY;
    gCountdownTimer = 0.0f;
    gRaceTimer = 0.0f;
}

/* ============================ Track test ============================ */

static bool IsOnTrack(float x, float y, float *distFromCenter)
{
    int i;
    float minDist = 99999.0f;

    for (i = 0; i < gTrackCount; i++)
    {
        float p1x = gTrackPoints[i].X;
        float p1y = gTrackPoints[i].Y;
        int   n   = (i + 1) % gTrackCount;
        float p2x = gTrackPoints[n].X;
        float p2y = gTrackPoints[n].Y;

        float lineX = p2x - p1x;
        float lineY = p2y - p1y;
        float pointX = x - p1x;
        float pointY = y - p1y;

        float lineLen = Vector2Length((Vector2){ lineX, lineY });
        if (lineLen > 0.0f)
        {
            float projT = (pointX * lineX + pointY * lineY) / (lineLen * lineLen);
            if (projT < 0.0f) projT = 0.0f;
            if (projT > 1.0f) projT = 1.0f;
            float projX = p1x + projT * lineX;
            float projY = p1y + projT * lineY;

            float d = Vector2Distance((Vector2){ x, y }, (Vector2){ projX, projY });
            if (d < minDist) minDist = d;
        }
    }

    *distFromCenter = minDist;
    return (minDist <= gTrackWidth / 2.0f);
}

/* ============================ Lap counting ============================ */

static void CheckRaceFinish(void);

static void CheckLap(Car *car)
{
    TrackPoint startPt = gTrackPoints[gStartIndex];

    float distToStart = Vector2Distance(
        (Vector2){ car->X, car->Y },
        (Vector2){ startPt.X, startPt.Y });

    if (distToStart < 100.0f)
    {
        if (!car->CanCountLap)
        {
            float angleDiff = fabsf(car->Angle - startPt.Angle);
            if (angleDiff > 180.0f) angleDiff = 360.0f - angleDiff;

            if (angleDiff < 70.0f)
            {
                if (car->LapsCompleted == 0)
                    car->LapsCompleted = 1;
                else
                    car->LapsCompleted++;

                car->CanCountLap = true;

                if (car->LapsCompleted > gTotalLaps && !car->Finished)
                {
                    car->Finished = true;

                    car->FinalRank = 1;
                    for (int i = 0; i < gCarCount; i++)
                        if (&gCars[i] != car && gCars[i].Finished)
                            car->FinalRank++;

                    CheckRaceFinish();
                }
            }
        }
    }
    else
    {
        if (distToStart > 170.0f)
            car->CanCountLap = false;
    }
}

static void CheckRaceFinish(void)
{
    int finishedCount = 0;
    for (int i = 0; i < gCarCount; i++)
        if (gCars[i].Finished)
            finishedCount++;

    if (finishedCount >= gCarCount)
    {
        if (gPlayerCar->FinalRank == 1)
            gWins++;

        gGameState = GS_FINISHED;
    }
}

/* ============================ Player physics ============================ */

static void UpdateCar(Car *car, float dt)
{
    bool left  = gKeyLeft;
    bool right = gKeyRight;
    bool up    = gKeyUp;
    bool down  = gKeyDown;

    if (gGameState == GS_COUNTDOWN)
    {
        up = down = left = right = false;
    }

    float distFromCenter;
    bool onTrack = IsOnTrack(car->X, car->Y, &distFromCenter);

    float accel = 350.0f;
    if (!onTrack) accel = 100.0f;
    float friction = 2.0f;
    if (!onTrack) friction = 5.0f;

    float maxSpeed = 300.0f + (float)(gLevel - 1) * PLAYER_BONUS;

    if (up)
    {
        car->Speed += accel * dt;
        if (car->Speed > maxSpeed) car->Speed = maxSpeed;
    }
    else if (down)
    {
        car->Speed -= accel * dt;
        if (car->Speed < -150.0f) car->Speed = -150.0f;
    }
    else
    {
        car->Speed -= car->Speed * friction * dt;
    }

    if (fabsf(car->Speed) > 5.0f)
    {
        float turnSpeed = 120.0f * (car->Speed / maxSpeed);
        if (left)  car->Angle -= turnSpeed * dt;
        if (right) car->Angle += turnSpeed * dt;
        car->Angle = NormalizeAngle360(car->Angle);
    }

    float rad = DEG2RAD * car->Angle;
    float vx = cosf(rad) * car->Speed * dt;
    float vy = sinf(rad) * car->Speed * dt;
    car->X += vx;
    car->Y += vy;

    CheckLap(car);
}

/* ============================ AI physics ============================ */

static void UpdateAI(Car *car, float dt)
{
    if (gGameState != GS_RACING)
        return;

    TrackPoint targetPt = gTrackPoints[car->TargetIndex];
    TrackPoint nextPt   = gTrackPoints[(car->TargetIndex + 1) % gTrackCount];
    TrackPoint farPt    = gTrackPoints[(car->TargetIndex + 2) % gTrackCount];

    float angleDelta = fabsf(targetPt.Angle - nextPt.Angle);
    if (angleDelta > 180.0f) angleDelta = 360.0f - angleDelta;

    float useOffset;
    if (angleDelta > 12.0f) useOffset = 0.0f;
    else                    useOffset = car->Offset;

    float perpX = -sinf(DEG2RAD * targetPt.Angle);
    float perpY =  cosf(DEG2RAD * targetPt.Angle);
    float offsetX = perpX * useOffset;
    float offsetY = perpY * useOffset;

    float distToTarget = Vector2Distance(
        (Vector2){ car->X, car->Y },
        (Vector2){ targetPt.X + offsetX, targetPt.Y + offsetY });

    float plainDist = Vector2Distance(
        (Vector2){ car->X, car->Y },
        (Vector2){ targetPt.X, targetPt.Y });

    Vector2 seg = Vector2Subtract(
        (Vector2){ nextPt.X, nextPt.Y },
        (Vector2){ targetPt.X, targetPt.Y });

    float dot = Vector2DotProduct(
        Vector2Subtract(
            (Vector2){ car->X, car->Y },
            (Vector2){ targetPt.X, targetPt.Y }),
        seg);

    float switchRadius = AI_WP_MIN_RADIUS + fabsf(car->Speed) * AI_WP_SPEED_FACTOR;

    car->WaypointTimer += dt;

    bool switched = false;

    if (plainDist < 10.0f || plainDist < switchRadius || dot > 0.0f)
        switched = true;

    if (car->WaypointTimer > AI_WP_TIMEOUT)
        switched = true;

    if (switched)
    {
        car->TargetIndex = (car->TargetIndex + 1) % gTrackCount;
        car->WaypointTimer = 0.0f;

        targetPt = gTrackPoints[car->TargetIndex];
        nextPt   = gTrackPoints[(car->TargetIndex + 1) % gTrackCount];
        farPt    = gTrackPoints[(car->TargetIndex + 2) % gTrackCount];

        angleDelta = fabsf(targetPt.Angle - nextPt.Angle);
        if (angleDelta > 180.0f) angleDelta = 360.0f - angleDelta;

        if (angleDelta > 12.0f) useOffset = 0.0f;
        else                    useOffset = car->Offset;

        offsetX = -sinf(DEG2RAD * targetPt.Angle) * useOffset;
        offsetY =  cosf(DEG2RAD * targetPt.Angle) * useOffset;

        distToTarget = Vector2Distance(
            (Vector2){ car->X, car->Y },
            (Vector2){ targetPt.X + offsetX, targetPt.Y + offsetY });
    }

    float desiredAngle = RAD2DEG * atan2f(
        (targetPt.Y + offsetY) - car->Y,
        (targetPt.X + offsetX) - car->X);
    desiredAngle = NormalizeAngle360(desiredAngle);

    float angleDiff = desiredAngle - car->Angle;
    if (angleDiff >  180.0f) angleDiff -= 360.0f;
    if (angleDiff < -180.0f) angleDiff += 360.0f;

    float distFromCenter;
    bool onTrack = IsOnTrack(car->X, car->Y, &distFromCenter);

    int lapDiff = gPlayerCar->LapsCompleted - car->LapsCompleted;
    float rubberMul = 1.0f;
    if (lapDiff >= 2)       rubberMul = 1.12f;
    else if (lapDiff <= -2) rubberMul = 0.92f;

    float levelFactor = LevelMul();

    float lookDelta = fabsf(targetPt.Angle - nextPt.Angle);
    if (lookDelta > 180.0f) lookDelta = 360.0f - lookDelta;

    float farDelta = fabsf(nextPt.Angle - farPt.Angle);
    if (farDelta > 180.0f) farDelta = 360.0f - farDelta;

    if (farDelta > lookDelta) lookDelta = farDelta;

    float cornerMul = 1.0f;
    if (lookDelta > AI_CORNER_START)
    {
        cornerMul = 1.0f - (lookDelta - AI_CORNER_START) /
                           (AI_CORNER_FULL - AI_CORNER_START);
        if (cornerMul < AI_CORNER_MIN_MUL) cornerMul = AI_CORNER_MIN_MUL;
    }

    float accel = AI_BASE_ACCEL * rubberMul * levelFactor;
    if (!onTrack) accel *= 0.4f;
    if (fabsf(angleDiff) > 60.0f) accel *= 0.6f;
    accel *= cornerMul;

    float maxAI = AI_BASE_SPEED * rubberMul * levelFactor * cornerMul;
    car->Speed += accel * dt;
    if (car->Speed > maxAI) car->Speed = maxAI;

    if (fabsf(car->Speed) > 5.0f)
    {
        float turnSpeed = 150.0f * (car->Speed / 320.0f) * (0.7f + 0.3f * levelFactor);
        if (angleDiff < 0.0f)
        {
            float step = fminf(fabsf(angleDiff), turnSpeed * dt);
            car->Angle -= step;
        }
        else
        {
            float step = fminf(angleDiff, turnSpeed * dt);
            car->Angle += step;
        }
        car->Angle = NormalizeAngle360(car->Angle);
    }

    float rad = DEG2RAD * car->Angle;
    float vx = cosf(rad) * car->Speed * dt;
    float vy = sinf(rad) * car->Speed * dt;
    car->X += vx;
    car->Y += vy;

    CheckLap(car);
}

/* ============================ Collisions ============================ */

static void ResolveCollisions(void)
{
    for (int i = 0; i < gCarCount; i++)
    {
        for (int j = i + 1; j < gCarCount; j++)
        {
            Car *a = &gCars[i];
            Car *b = &gCars[j];

            float dx = b->X - a->X;
            float dy = b->Y - a->Y;
            float dist = Vector2Length((Vector2){ dx, dy });

            if (dist < CAR_SIZE)
            {
                if (dist == 0.0f)
                {
                    dist = 0.1f;
                    dx = 1.0f;
                    dy = 0.0f;
                }

                float overlap = CAR_SIZE - dist;
                float pushX = (dx / dist) * (overlap / 2.0f);
                float pushY = (dy / dist) * (overlap / 2.0f);

                a->X -= pushX;
                a->Y -= pushY;
                b->X += pushX;
                b->Y += pushY;

                float speedDiff = (a->Speed - b->Speed) * 0.5f;
                a->Speed -= speedDiff * 0.5f;
                b->Speed += speedDiff * 0.5f;

                a->Angle = NormalizeAngle360(a->Angle - (dx / dist) * 5.0f);
                b->Angle = NormalizeAngle360(b->Angle + (dx / dist) * 5.0f);
            }
        }
    }
}

/* ============================ Input ============================ */

static void StartCountdown(void)
{
    gGameState = GS_COUNTDOWN;
    gCountdownTimer = 3.0f;
    gRaceTimer = 0.0f;
}

static void HandleInput(void)
{
    gKeyLeft  = IsKeyDown(KEY_LEFT);
    gKeyRight = IsKeyDown(KEY_RIGHT);
    gKeyUp    = IsKeyDown(KEY_UP);
    gKeyDown  = IsKeyDown(KEY_DOWN);

    if (gGameState == GS_READY && IsKeyPressed(KEY_ENTER))
        StartCountdown();

    if (gGameState == GS_READY && IsKeyPressed(KEY_R))
        InitRace();

    if (gGameState == GS_FINISHED && IsKeyPressed(KEY_ENTER))
    {
        if (gPlayerCar->FinalRank == 1)
            gLevel++;

        InitRace();
        StartCountdown();
    }
}

/* ============================ Update ============================ */

static void Update(float dt)
{
    if (gGameState == GS_COUNTDOWN)
    {
        gCountdownTimer -= dt;
        if (gCountdownTimer <= 0.0f)
        {
            gCountdownTimer = 0.0f;
            gGameState = GS_RACING;
        }
    }

    if (gGameState == GS_RACING)
        gRaceTimer += dt;

    if (gGameState == GS_RACING || gGameState == GS_COUNTDOWN)
    {
        for (int i = 0; i < gCarCount; i++)
        {
            Car *c = &gCars[i];
            if (!c->Finished)
            {
                if (c->IsPlayer) UpdateCar(c, dt);
                else             UpdateAI(c, dt);
            }
        }
        ResolveCollisions();
    }

    gCamera.offset = (Vector2){ GetScreenWidth() / 2.0f,
                                GetScreenHeight() / 2.0f };

    gCamera.target.x += (gPlayerCar->X - gCamera.target.x) * 5.0f * dt;
    gCamera.target.y += (gPlayerCar->Y - gCamera.target.y) * 5.0f * dt;

    gCamera.rotation = 0.0f;
    gCamera.zoom = 1.0f;
}

/* ============================ Draw ============================ */

static void DrawCarBody(const Car *c)
{
    Vector2 p = GetWorldToScreen2D((Vector2){ c->X, c->Y }, gCamera);
    float h = CAR_VISUAL_LEN / 2.0f;
    float w = CAR_VISUAL_W   / 2.0f;
    float rad = DEG2RAD * c->Angle;
    float cosA = cosf(rad);
    float sinA = sinf(rad);

    Color carCol = c->Color;
    Color cabCol = (Color){ 0, 0, 0, 255 };

    /* Body quad */
    Vector2 A, B, D, E;
    A.x = p.x + h * cosA - w * sinA;
    A.y = p.y + h * sinA + w * cosA;

    B.x = p.x + h * cosA + w * sinA;
    B.y = p.y + h * sinA - w * cosA;

    D.x = p.x - h * cosA + w * sinA;
    D.y = p.y - h * sinA - w * cosA;

    E.x = p.x - h * cosA - w * sinA;
    E.y = p.y - h * sinA + w * cosA;

    DrawTriangle(A, B, D, carCol);
    DrawTriangle(A, D, E, carCol);

    /* Cabin */
    float cabFwd  = h * 0.30f;
    float cabBack = h * 0.05f;
    float cabHalfW = w * 0.65f;
    float cabHalfH = h * 0.20f;

    Vector2 CA, CB, CC, CD;
    CA.x = p.x + (cabFwd + cabHalfH) * cosA - cabHalfW * sinA;
    CA.y = p.y + (cabFwd + cabHalfH) * sinA + cabHalfW * cosA;

    CB.x = p.x + (cabFwd + cabHalfH) * cosA + cabHalfW * sinA;
    CB.y = p.y + (cabFwd + cabHalfH) * sinA - cabHalfW * cosA;

    CC.x = p.x + (cabFwd - cabBack) * cosA + cabHalfW * sinA;
    CC.y = p.y + (cabFwd - cabBack) * sinA - cabHalfW * cosA;

    CD.x = p.x + (cabFwd - cabBack) * cosA - cabHalfW * sinA;
    CD.y = p.y + (cabFwd - cabBack) * sinA + cabHalfW * cosA;

    DrawTriangle(CA, CB, CC, cabCol);
    DrawTriangle(CA, CC, CD, cabCol);
}

static void Draw(void)
{
    BeginDrawing();
    ClearBackground((Color){ 46, 125, 50, 255 });

    int screenW = GetScreenWidth();
    int screenH = GetScreenHeight();

    /* Track: shoulder */
    for (int i = 0; i < gTrackCount; i++)
    {
        TrackPoint p1 = gTrackPoints[i];
        TrackPoint p2 = gTrackPoints[(i + 1) % gTrackCount];
        DrawLineEx(
            GetWorldToScreen2D((Vector2){ p1.X, p1.Y }, gCamera),
            GetWorldToScreen2D((Vector2){ p2.X, p2.Y }, gCamera),
            gTrackWidth + SHOULDER_EXTRA,
            (Color){ 120, 100, 60, 255 });
    }
    for (int i = 0; i < gTrackCount; i++)
    {
        DrawCircleV(
            GetWorldToScreen2D((Vector2){ gTrackPoints[i].X, gTrackPoints[i].Y }, gCamera),
            (gTrackWidth + SHOULDER_EXTRA) / 2.0f,
            (Color){ 120, 100, 60, 255 });
    }

    /* Track: asphalt */
    for (int i = 0; i < gTrackCount; i++)
    {
        TrackPoint p1 = gTrackPoints[i];
        TrackPoint p2 = gTrackPoints[(i + 1) % gTrackCount];
        DrawLineEx(
            GetWorldToScreen2D((Vector2){ p1.X, p1.Y }, gCamera),
            GetWorldToScreen2D((Vector2){ p2.X, p2.Y }, gCamera),
            gTrackWidth,
            (Color){ 58, 58, 58, 255 });
    }
    for (int i = 0; i < gTrackCount; i++)
    {
        DrawCircleV(
            GetWorldToScreen2D((Vector2){ gTrackPoints[i].X, gTrackPoints[i].Y }, gCamera),
            gTrackWidth / 2.0f,
            (Color){ 58, 58, 58, 255 });
    }

    /* Track: centre line */
    for (int i = 0; i < gTrackCount; i++)
    {
        TrackPoint p1 = gTrackPoints[i];
        TrackPoint p2 = gTrackPoints[(i + 1) % gTrackCount];
        DrawLineEx(
            GetWorldToScreen2D((Vector2){ p1.X, p1.Y }, gCamera),
            GetWorldToScreen2D((Vector2){ p2.X, p2.Y }, gCamera),
            CENTRE_LINE_W,
            (Color){ 255, 255, 255, 255 });
    }

    /* Chequered start/finish strip */
    {
        TrackPoint startP = gTrackPoints[gStartIndex];
        float sfAng = DEG2RAD * startP.Angle;

        float sfNX = -sinf(sfAng);
        float sfNY =  cosf(sfAng);
        float sfTX =  cosf(sfAng);
        float sfTY =  sinf(sfAng);

        float cellW = gTrackWidth / FINISH_CELLS;
        float cellH = cellW;
        float halfW = cellW / 2.0f;
        float halfH = cellH / 2.0f;

        int colFrom = -(FINISH_CELLS / 2);
        int colTo   = colFrom + FINISH_CELLS - 1;
        int rowFrom = -(FINISH_ROWS / 2);
        int rowTo   = rowFrom + FINISH_ROWS - 1;

        for (int col = colFrom; col <= colTo; col++)
        {
            for (int row = rowFrom; row <= rowTo; row++)
            {
                float cxw = startP.X
                          + sfNX * (col + 0.5f) * cellW
                          + sfTX * (row + 0.5f) * cellH;
                float cyw = startP.Y
                          + sfNY * (col + 0.5f) * cellW
                          + sfTY * (row + 0.5f) * cellH;

                Vector2 v0, v1, v2, v3;
                v0.x = cxw + sfNX * (-halfW) + sfTX * (-halfH);
                v0.y = cyw + sfNY * (-halfW) + sfTY * (-halfH);

                v1.x = cxw + sfNX * ( halfW) + sfTX * (-halfH);
                v1.y = cyw + sfNY * ( halfW) + sfTY * (-halfH);

                v2.x = cxw + sfNX * ( halfW) + sfTX * ( halfH);
                v2.y = cyw + sfNY * ( halfW) + sfTY * ( halfH);

                v3.x = cxw + sfNX * (-halfW) + sfTX * ( halfH);
                v3.y = cyw + sfNY * (-halfW) + sfTY * ( halfH);

                v0 = GetWorldToScreen2D(v0, gCamera);
                v1 = GetWorldToScreen2D(v1, gCamera);
                v2 = GetWorldToScreen2D(v2, gCamera);
                v3 = GetWorldToScreen2D(v3, gCamera);

                Color cellCol;
                if ((col + row) & 1)
                    cellCol = (Color){ 20, 20, 20, 255 };
                else
                    cellCol = (Color){ 245, 245, 245, 255 };

                DrawTriangle(v0, v1, v2, cellCol);
                DrawTriangle(v0, v2, v3, cellCol);
            }
        }
    }

    /* Cars */
    for (int i = 0; i < gCarCount; i++)
        DrawCarBody(&gCars[i]);

    /* HUD */
    switch (gGameState)
    {
        case GS_READY:
            DrawText("RAY MICRO RACERS", 40, 40, 40, (Color){ 255, 255, 255, 255 });
            DrawText(TextFormat("Level %d   Wins %d", gLevel, gWins),
                     40, 100, 24, (Color){ 255, 230, 60, 255 });
            DrawText("Press ENTER to start", 40, 140, 20, (Color){ 255, 255, 255, 255 });
            DrawText("Press R to generate a new track", 40, 170, 20, (Color){ 200, 220, 255, 255 });
            break;

        case GS_COUNTDOWN:
        {
            int countdownInt = (int)ceilf(gCountdownTimer);
            if (countdownInt < 0) countdownInt = 0;

            DrawText(TextFormat("%d", countdownInt),
                     screenW / 2 - 25, screenH / 2 - 60,
                     120, (Color){ 255, 255, 255, 255 });
            DrawText(TextFormat("LEVEL %d", gLevel), 40, 40, 30,
                     (Color){ 255, 220, 0, 255 });
            break;
        }

        case GS_RACING:
            DrawText(TextFormat("Lap: %d / %d", gPlayerCar->LapsCompleted, gTotalLaps),
                     10, 10, 20, (Color){ 255, 255, 255, 255 });
            DrawText(TextFormat("Time: %.2f", gRaceTimer),
                     10, 40, 20, (Color){ 255, 255, 255, 255 });
            DrawText(TextFormat("Level: %d   Wins: %d", gLevel, gWins),
                     10, 70, 20, (Color){ 200, 220, 255, 255 });
            break;

        case GS_FINISHED:
            if (gPlayerCar->FinalRank == 1)
            {
                DrawText("YOU WIN!", 40, 40, 48, (Color){ 255, 230, 60, 255 });
                DrawText(TextFormat("Level %d cleared. Next: %d", gLevel, gLevel + 1),
                         40, 110, 24, (Color){ 255, 255, 255, 255 });
            }
            else
            {
                DrawText("YOU LOST", 40, 40, 48, (Color){ 255, 80, 80, 255 });
                DrawText(TextFormat("Finished %d of %d", gPlayerCar->FinalRank, gCarCount),
                         40, 110, 24, (Color){ 255, 255, 255, 255 });
            }
            DrawText(TextFormat("Wins total: %d", gWins),
                     40, 150, 20, (Color){ 200, 220, 255, 255 });
            DrawText("Press ENTER to continue", 40, 190, 20,
                     (Color){ 255, 255, 255, 255 });
            break;
    }

    DrawFPS(screenW - 90, 10);

    EndDrawing();
}

/* ============================ main ============================ */

int main(void)
{
    const int screenWidth  = 800;
    const int screenHeight = 600;

    InitWindow(screenWidth, screenHeight, "Micro Racers (raylib C)");
    SetWindowState(FLAG_MSAA_4X_HINT);
    SetTargetFPS(60);
    SetTraceLogLevel(LOG_WARNING);

    gCamera.offset = (Vector2){ screenWidth / 2.0f, screenHeight / 2.0f };
    gCamera.target = (Vector2){ 0, 0 };
    gCamera.rotation = 0.0f;
    gCamera.zoom = 1.0f;

    InitRace();
    gCamera.target = (Vector2){ gPlayerCar->X, gPlayerCar->Y };

    double lastTime = GetTime();

    while (!WindowShouldClose())
    {
        double now = GetTime();
        float delta = (float)(now - lastTime);
        lastTime = now;

        if (delta > 0.1f) delta = 0.1f;

        HandleInput();
        Update(delta);
        Draw();
    }

    CloseWindow();
    return 0;
}
