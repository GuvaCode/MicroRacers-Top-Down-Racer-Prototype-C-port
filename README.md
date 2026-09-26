**Micro Racers — Top-Down Racing in C + raylib**

Ported a racing prototype from Pascal/Skia to pure C + raylib. Single file, no dependencies besides raylib.

**What's inside:**

- Physics 1:1 with the original — a single scalar speed, the car always moves strictly along its heading (no slip, no drift), steering is proportional to speed
- I also added procedural track generation — a star-shaped curve in polar coordinates, regenerated until the max turn between adjacent segments is below the allowed threshold
- No sprites or external assets — everything is drawn procedurally with raylib primitives (lines, circles, triangles)
- AI opponents — follow waypoints with look-ahead braking into corners, plus a rubber-band system based on lap gap
- Laps, checkpoints, chequered start/finish strip, 3 laps per race
- Difficulty progression — each win raises the level and the AI gets faster
- Controls: arrow keys, Enter to start, R to generate a new track

**Stack:** C99, raylib, raymath
**License:** MIT
**Original Pascal author:** Lara Miriam Tamy Reschke
**C/raylib port:** Vadim Gunko (@GuvaCode)

Code is in a single file, builds in a couple of commands. Happy to share the repo or a binary if anyone's interested.

#raylib #gamedev #c #racing

