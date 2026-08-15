# Velocity Rush — 3D Racing Game

A 3D racing game built with **C++ and GLUT/OpenGL** (fixed-function pipeline), written as a single `main.cpp`. Drive through checkpoints across three hand-built tracks, avoid obstacles, and race the clock. Built as a portfolio project.

![status](https://img.shields.io/badge/status-in--development-yellow)

## Features
- Real 3D driving physics: acceleration, braking, friction, and speed-sensitive steering with a minimum-response floor so low-speed turning never feels dead
- Off-road penalty — stray from the track and the car drags to a crawl
- Obstacle collision with push-back
- Dynamic lighting: a fixed "sun" light plus a spotlight that follows the car as headlights
- Boxy Land Cruiser-style SUV: two-tone painted body, roof rack, front grille and bumpers, lit headlights/taillights, and a rear-mounted spare tire
- Procedurally generated textures (grass ground, asphalt road, two-tone SUV paint) — no external image files needed
- Distance fog and a gradient sky background for atmosphere
- HUD with translucent panels, a color-coded speed bar, and a checkpoint progress bar
- Checkpoint/lap system with a live timer
- Three levels of increasing difficulty (wide oval → switchback → tight night course)
- Third-person chase camera

## Controls
| Key | Action |
|---|---|
| `W` / `↑` | Accelerate |
| `S` / `↓` | Brake / reverse |
| `A` / `←` | Steer left |
| `D` / `→` | Steer right |
| `Enter` | Start / advance to next level |
| `R` | Restart current level |
| `Esc` | Quit |

## Building it

### Code::Blocks (Windows or Linux)
1. Install **freeglut** (Windows: drop `freeglut.h`, `.lib`, `.dll` into your MinGW `include`/`lib`/`bin` folders — many guides call this "GLUT for Code::Blocks"; Linux: `sudo apt install freeglut3-dev`).
2. Create a new **Empty Project**, add `main.cpp` to it.
3. Project → Build options → **Linker settings** → add these libraries:
   - `glut32` (Windows) or `glut` (Linux)
   - `glu32` / `GLU`
   - `opengl32` / `GL`
4. Project → Build options → **Compiler settings** → make sure C++11 (or later) is enabled (`-std=c++11`).
5. Build and run (`F9`).

### Command line (Linux/macOS, useful for quick checks)
```bash
sudo apt install freeglut3-dev libglu1-mesa-dev libgl1-mesa-dev   # once
g++ -std=c++11 main.cpp -o velocity_rush -lglut -lGLU -lGL
./velocity_rush
```

## Code structure
Everything lives in `main.cpp`, organized into numbered sections so it's easy to navigate as it grows:

1. **Math helpers** — `Vec3` and distance functions
2. **Game data structures** — `Obstacle`, `Level`, `Car`, `GameState`
3. **Global state** — car, current level, timers, input flags
4. **Level data** — the three hardcoded tracks (waypoints + obstacles)
5. **Physics** — `updateCar()`, run every frame
6. **Camera & lighting** — chase cam + sun/headlight setup
7. **Drawing** — ground, road ribbon, checkpoint gates, obstacles, car
8. **HUD** — 2D text overlay (menu, timer, checkpoint count)
9. **GLUT callbacks** — display/reshape/keyboard/timer
10. **main()** — window + callback setup

## Roadmap
- [x] Textured ground, road, and car instead of flat colors
- [x] Distance fog + gradient sky
- [x] HUD panels with a speed bar and progress bar
- [ ] Lap-based looping tracks (currently point-to-point)
- [ ] Simple particle effects for tire skid / dust
- [ ] Sound (engine hum, collision thud) — GLUT has no audio API, so this needs an external lib or platform sound calls
- [ ] Minimap HUD
- [ ] Ghost/best-time replay

## License
MIT — do whatever you'd like with it.
