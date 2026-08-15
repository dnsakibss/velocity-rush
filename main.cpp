/* ================================================================
   VELOCITY RUSH — a 3D racing game
   ----------------------------------------------------------------
   Engine   : GLUT / OpenGL fixed-function pipeline
   Language : C++ (needs C++11 for the vector initializer lists)
   Layout   : single-file architecture (main.cpp), organized into
              numbered sections so it stays readable as it grows.

   CONTROLS
     W / Up Arrow     accelerate
     S / Down Arrow   brake / reverse
     A / Left Arrow   steer left
     D / Right Arrow  steer right
     ENTER            start game / advance to next level
     R                restart current level
     ESC              quit
   ================================================================ */

#include <GL/glut.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cctype>
#include <vector>
#include <string>

// ----------------------------------------------------------------
// SECTION 1 — MATH HELPERS
// Everything below is standard "gameplay math": vectors, distances,
// and one function (pointSegmentDistance) that answers "how far is
// the car from the road?" which drives the off-road physics.
// ----------------------------------------------------------------
const float PI = 3.14159265358979f;
const float DEG2RAD = PI / 180.0f;

struct Vec3 {
    float x, y, z;
    Vec3(float x_ = 0, float y_ = 0, float z_ = 0) : x(x_), y(y_), z(z_) {}
    Vec3 operator+(const Vec3& o) const { return Vec3(x + o.x, y + o.y, z + o.z); }
    Vec3 operator-(const Vec3& o) const { return Vec3(x - o.x, y - o.y, z - o.z); }
    Vec3 operator*(float s) const { return Vec3(x * s, y * s, z * s); }
    float length() const { return sqrtf(x * x + y * y + z * z); }
    Vec3 normalized() const {
        float len = length();
        if (len < 0.0001f) return Vec3(0, 0, 0);
        return Vec3(x / len, y / len, z / len);
    }
};

float dist(const Vec3& a, const Vec3& b) { return (a - b).length(); }

// Closest distance from point p to line segment [a,b], measured on the
// ground (XZ) plane. Used to detect whether the car has left the road.
float pointSegmentDistance(const Vec3& p, const Vec3& a, const Vec3& b) {
    Vec3 ab = b - a;
    float len2 = ab.x * ab.x + ab.z * ab.z;
    if (len2 < 0.0001f) return dist(p, a);
    float t = ((p.x - a.x) * ab.x + (p.z - a.z) * ab.z) / len2;
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    Vec3 closest(a.x + ab.x * t, 0, a.z + ab.z * t);
    return dist(Vec3(p.x, 0, p.z), closest);
}

// ----------------------------------------------------------------
// SECTION 1B — PROCEDURAL TEXTURES
// Rather than loading external image files (which would mean extra
// assets to ship alongside main.cpp and extra Code::Blocks setup),
// each texture is built pixel-by-pixel in code, then uploaded to the
// GPU once at startup. Same GPU-side result, zero external files.
// ----------------------------------------------------------------
GLuint grassTexture = 0;
GLuint asphaltTexture = 0;
GLuint suvBodyTexture = 0;

unsigned char clampByte(int v) {
    if (v < 0) return 0;
    if (v > 255) return 255;
    return (unsigned char)v;
}

// Uploads an RGB pixel buffer as a repeating, mipmapped texture and
// returns its GPU texture id.
GLuint uploadTexture(unsigned char* pixels, int size) {
    GLuint id;
    glGenTextures(1, &id);
    glBindTexture(GL_TEXTURE_2D, id);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGB, size, size, GL_RGB, GL_UNSIGNED_BYTE, pixels);
    return id;
}

// Mottled green with a faint patchwork pattern, like turf seen from a car.
GLuint createGrassTexture(int size = 128) {
    unsigned char* pix = new unsigned char[size * size * 3];
    for (int y = 0; y < size; y++) {
        for (int x = 0; x < size; x++) {
            int n = (rand() % 40) - 20;
            int patch = ((x / 6) + (y / 6)) % 2;
            int idx = (y * size + x) * 3;
            pix[idx + 0] = clampByte(55 + patch * 8 + n);
            pix[idx + 1] = clampByte(130 + patch * 15 + n);
            pix[idx + 2] = clampByte(55 + patch * 8 + n);
        }
    }
    GLuint id = uploadTexture(pix, size);
    delete[] pix;
    return id;
}

// Grainy dark gray, like tarmac.
GLuint createAsphaltTexture(int size = 128) {
    unsigned char* pix = new unsigned char[size * size * 3];
    for (int y = 0; y < size; y++) {
        for (int x = 0; x < size; x++) {
            int n = (rand() % 30) - 15;
            int base = 60 + n;
            int idx = (y * size + x) * 3;
            pix[idx + 0] = clampByte(base);
            pix[idx + 1] = clampByte(base);
            pix[idx + 2] = clampByte(base + 3);
        }
    }
    GLuint id = uploadTexture(pix, size);
    delete[] pix;
    return id;
}

// Two-tone SUV paint: a dark "cladding" band low on the body (like the
// protective black plastic on the lower doors/bumpers of a real SUV),
// with lighter paint above. Mapped onto the chassis box in drawCar().
GLuint createSUVBodyTexture(int size = 128) {
    unsigned char* pix = new unsigned char[size * size * 3];
    for (int y = 0; y < size; y++) {
        float t = (float)y / (float)(size - 1); // 0 = bottom of the panel, 1 = top
        bool cladding = t < 0.32f;
        for (int x = 0; x < size; x++) {
            int n = (rand() % 14) - 7;
            int idx = (y * size + x) * 3;
            if (cladding) {
                pix[idx + 0] = clampByte(35 + n);
                pix[idx + 1] = clampByte(35 + n);
                pix[idx + 2] = clampByte(38 + n);
            } else {
                pix[idx + 0] = clampByte(225 + n);
                pix[idx + 1] = clampByte(222 + n);
                pix[idx + 2] = clampByte(210 + n);
            }
        }
    }
    GLuint id = uploadTexture(pix, size);
    delete[] pix;
    return id;
}

// ----------------------------------------------------------------
// SECTION 2 — GAME DATA STRUCTURES
// ----------------------------------------------------------------
struct Obstacle {
    Vec3 position;
    float radius;
};

struct Level {
    std::string name;
    std::vector<Vec3> waypoints;   // road centerline; [0] = start, last = finish
    std::vector<Obstacle> obstacles;
    float trackWidth;
    Vec3 roadColor;
};

struct Car {
    Vec3 position;
    float heading;   // degrees, 0 = facing +Z
    float speed;      // units/sec, positive = forward, negative = reverse
    float radius;      // collision radius
};

enum GameState { MENU, PLAYING, LEVEL_COMPLETE, ALL_COMPLETE };

// ----------------------------------------------------------------
// SECTION 3 — GLOBAL STATE
// ----------------------------------------------------------------
int windowWidth = 1000, windowHeight = 700;

std::vector<Level> levels;
int currentLevelIndex = 0;
Car car;
int nextCheckpoint = 1; // index into waypoints; 0 is the start line

GameState gameState = MENU;
bool keyDown[256];
bool specialKeyDown[128];

float levelTimer = 0.0f;
float lastLevelTime = 0.0f;
int prevTimeMs = 0;

GLUquadric* quadric = NULL;

// Tunable physics constants — change these to make the game feel
// different (arcade-y vs. heavier / more "sim").
const float ACCELERATION = 30.0f;
const float BRAKE_DECEL  = 40.0f;
const float FRICTION     = 12.0f;
const float OFFROAD_DRAG = 46.0f;
const float MAX_SPEED    = 46.0f;
const float MAX_REVERSE  = -18.0f;
const float TURN_RATE    = 170.0f; // degrees/sec at full speed

// ----------------------------------------------------------------
// SECTION 4 — LEVEL DATA
// Each level is a hand-authored path of waypoints. Consecutive
// waypoints become road segments; every waypoint after the first is
// a checkpoint the player must reach in order.
// ----------------------------------------------------------------
void buildLevels() {
    levels.clear();

    Level l1;
    l1.name = "Sunrise Oval";
    l1.trackWidth = 12.0f;
    l1.roadColor = Vec3(0.25f, 0.25f, 0.28f);
    l1.waypoints = {
        Vec3(0, 0, 0), Vec3(0, 0, 60), Vec3(30, 0, 90),
        Vec3(70, 0, 90), Vec3(100, 0, 60), Vec3(100, 0, 0),
        Vec3(70, 0, -30), Vec3(30, 0, -30), Vec3(0, 0, 0)
    };
    l1.obstacles = {
        {Vec3(15, 0, 40), 3.0f}, {Vec3(85, 0, 30), 3.0f}, {Vec3(50, 0, -20), 3.0f}
    };
    levels.push_back(l1);

    Level l2;
    l2.name = "Canyon Switchback";
    l2.trackWidth = 9.0f;
    l2.roadColor = Vec3(0.30f, 0.20f, 0.18f);
    l2.waypoints = {
        Vec3(0, 0, 0), Vec3(40, 0, 10), Vec3(50, 0, 50), Vec3(20, 0, 80),
        Vec3(-20, 0, 80), Vec3(-40, 0, 50), Vec3(-30, 0, 10), Vec3(0, 0, -20)
    };
    l2.obstacles = {
        {Vec3(35, 0, 25), 2.5f}, {Vec3(10, 0, 70), 2.5f}, {Vec3(-30, 0, 65), 2.5f},
        {Vec3(-15, 0, 20), 2.5f}, {Vec3(15, 0, -5), 2.5f}
    };
    levels.push_back(l2);

    Level l3;
    l3.name = "Night Rush";
    l3.trackWidth = 7.0f;
    l3.roadColor = Vec3(0.18f, 0.18f, 0.22f);
    l3.waypoints = {
        Vec3(0, 0, 0), Vec3(20, 0, 20), Vec3(15, 0, 55), Vec3(40, 0, 75),
        Vec3(70, 0, 60), Vec3(65, 0, 25), Vec3(90, 0, 5), Vec3(60, 0, -25),
        Vec3(20, 0, -15), Vec3(0, 0, -40)
    };
    l3.obstacles = {
        {Vec3(18, 0, 35), 2.0f}, {Vec3(45, 0, 65), 2.0f}, {Vec3(68, 0, 40), 2.0f},
        {Vec3(80, 0, 10), 2.0f}, {Vec3(75, 0, -10), 2.0f}, {Vec3(40, 0, -18), 2.0f},
        {Vec3(10, 0, -25), 2.0f}
    };
    levels.push_back(l3);
}

void startLevel(int index) {
    currentLevelIndex = index;
    Level& lvl = levels[index];
    car.position = lvl.waypoints[0];
    Vec3 dir = (lvl.waypoints[1] - lvl.waypoints[0]).normalized();
    car.heading = atan2f(dir.x, dir.z) / DEG2RAD;
    car.speed = 0.0f;
    car.radius = 1.5f;
    nextCheckpoint = 1;
    levelTimer = 0.0f;
    gameState = PLAYING;
}

// ----------------------------------------------------------------
// SECTION 5 — PHYSICS
// Runs every frame. Handles acceleration/braking/friction, an
// off-road drag penalty, steering, obstacle collision, and
// checkpoint progress.
// ----------------------------------------------------------------
void updateCar(float dt) {
    Level& lvl = levels[currentLevelIndex];

    bool accel = keyDown['w'] || specialKeyDown[GLUT_KEY_UP];
    bool brake = keyDown['s'] || specialKeyDown[GLUT_KEY_DOWN];

    if (accel) car.speed += ACCELERATION * dt;
    else if (brake) car.speed -= BRAKE_DECEL * dt;
    else {
        if (car.speed > 0) car.speed -= FRICTION * dt;
        else if (car.speed < 0) car.speed += FRICTION * dt;
        if (fabs(car.speed) < 0.05f) car.speed = 0;
    }

    // off-road check: distance from car to the nearest road segment
    float minD = 1e9f;
    for (size_t i = 0; i + 1 < lvl.waypoints.size(); i++) {
        float d = pointSegmentDistance(car.position, lvl.waypoints[i], lvl.waypoints[i + 1]);
        if (d < minD) minD = d;
    }
    if (minD > lvl.trackWidth * 0.5f) {
        if (car.speed > 0) car.speed -= OFFROAD_DRAG * dt;
        else if (car.speed < 0) car.speed += OFFROAD_DRAG * dt;
        if (fabs(car.speed) < 0.05f) car.speed = 0;
    }

    if (car.speed > MAX_SPEED) car.speed = MAX_SPEED;
    if (car.speed < MAX_REVERSE) car.speed = MAX_REVERSE;

    bool left  = keyDown['a'] || specialKeyDown[GLUT_KEY_LEFT];
    bool right = keyDown['d'] || specialKeyDown[GLUT_KEY_RIGHT];

    // Steering strength scales with speed (fast = sharper turns), but is
    // floored at 35% so tapping A/D at low speed still visibly responds
    // instead of feeling dead. Sign still flips in reverse, like a real car.
    float speedMag = fabs(car.speed) / MAX_SPEED;
    if (speedMag < 0.35f) speedMag = 0.35f;
    float speedFactor = (car.speed < 0 ? -1.0f : 1.0f) * speedMag;
    // NOTE: with forward = (sin(heading), 0, cos(heading)) and a right-handed
    // coordinate system, INCREASING heading actually swings the car toward
    // world -X, which is the driver's left, not right. So "turn right" has
    // to DECREASE heading, and "turn left" has to INCREASE it — this looks
    // backwards at first glance but it's what makes A/D feel correct.
    if (left)  car.heading += TURN_RATE * dt * speedFactor;
    if (right) car.heading -= TURN_RATE * dt * speedFactor;

    float rad = car.heading * DEG2RAD;
    Vec3 forward(sinf(rad), 0, cosf(rad));
    Vec3 newPos = car.position + forward * (car.speed * dt);

    // obstacle collision: push the car back out along the hit normal
    for (size_t i = 0; i < lvl.obstacles.size(); i++) {
        Obstacle& ob = lvl.obstacles[i];
        float d = dist(newPos, ob.position);
        float minDist = car.radius + ob.radius;
        if (d < minDist && d > 0.0001f) {
            Vec3 pushDir = (newPos - ob.position).normalized();
            newPos = ob.position + pushDir * minDist;
            car.speed *= 0.4f;
        }
    }
    car.position = newPos;

    if (nextCheckpoint < (int)lvl.waypoints.size()) {
        float d = dist(car.position, lvl.waypoints[nextCheckpoint]);
        if (d < lvl.trackWidth) {
            nextCheckpoint++;
            if (nextCheckpoint >= (int)lvl.waypoints.size()) {
                lastLevelTime = levelTimer;
                gameState = LEVEL_COMPLETE;
            }
        }
    }

    levelTimer += dt;
}

// ----------------------------------------------------------------
// SECTION 6 — CAMERA & LIGHTING
// ----------------------------------------------------------------
void applyCamera() {
    float rad = car.heading * DEG2RAD;
    Vec3 forward(sinf(rad), 0, cosf(rad));
    Vec3 eye = car.position - forward * 14.0f + Vec3(0, 7.0f, 0);
    Vec3 center = car.position + forward * 8.0f + Vec3(0, 1.5f, 0);
    gluLookAt(eye.x, eye.y, eye.z, center.x, center.y, center.z, 0, 1, 0);
}

void updateHeadlights() {
    float rad = car.heading * DEG2RAD;
    Vec3 forward(sinf(rad), 0, cosf(rad));
    Vec3 pos = car.position + forward * 1.5f + Vec3(0, 1.0f, 0);
    GLfloat lp[] = { pos.x, pos.y, pos.z, 1.0f };
    GLfloat ld[] = { forward.x, forward.y, forward.z };
    glLightfv(GL_LIGHT1, GL_POSITION, lp);
    glLightfv(GL_LIGHT1, GL_SPOT_DIRECTION, ld);
}

void initGL() {
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_NORMALIZE); // keeps lighting correct even after glScalef
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glShadeModel(GL_SMOOTH);
    glClearColor(0.55f, 0.75f, 0.95f, 1.0f);

    // LIGHT0 = the "sun" — fixed over the whole scene
    GLfloat sunPos[]     = { 60.0f, 100.0f, 40.0f, 1.0f };
    GLfloat sunAmbient[] = { 0.35f, 0.35f, 0.38f, 1.0f };
    GLfloat sunDiffuse[] = { 0.85f, 0.82f, 0.70f, 1.0f };
    glLightfv(GL_LIGHT0, GL_POSITION, sunPos);
    glLightfv(GL_LIGHT0, GL_AMBIENT,  sunAmbient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  sunDiffuse);

    // LIGHT1 = the car's headlights — a spotlight, repositioned every frame
    GLfloat hlDiffuse[] = { 1.0f, 1.0f, 0.85f, 1.0f };
    glLightfv(GL_LIGHT1, GL_DIFFUSE, hlDiffuse);
    glLightf(GL_LIGHT1, GL_SPOT_CUTOFF, 25.0f);
    glLightf(GL_LIGHT1, GL_SPOT_EXPONENT, 12.0f);
    glLightf(GL_LIGHT1, GL_CONSTANT_ATTENUATION, 1.0f);
    glLightf(GL_LIGHT1, GL_LINEAR_ATTENUATION, 0.02f);

    // Distance fog: makes the far edges of the track fade into the sky
    // instead of stopping abruptly, which reads as much more "finished."
    GLfloat fogColor[] = { 0.55f, 0.75f, 0.95f, 1.0f };
    glEnable(GL_FOG);
    glFogi(GL_FOG_MODE, GL_LINEAR);
    glFogfv(GL_FOG_COLOR, fogColor);
    glFogf(GL_FOG_START, 80.0f);
    glFogf(GL_FOG_END, 260.0f);
    glHint(GL_FOG_HINT, GL_NICEST);

    glTexEnvi(GL_TEXTURE_2D, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    srand(1337); // fixed seed so the generated textures look the same every run
    grassTexture = createGrassTexture();
    asphaltTexture = createAsphaltTexture();
    suvBodyTexture = createSUVBodyTexture();

    quadric = gluNewQuadric();
}

// ----------------------------------------------------------------
// SECTION 7 — DRAWING
// ----------------------------------------------------------------
void drawGround() {
    glColor3f(1, 1, 1); // white so the texture's own colors show through unmodified
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, grassTexture);
    glNormal3f(0, 1, 0);
    const float tile = 8.0f; // world units per texture repeat
    glBegin(GL_QUADS);
        glTexCoord2f(-300 / tile, -300 / tile); glVertex3f(-300, -0.05f, -300);
        glTexCoord2f(-300 / tile,  300 / tile); glVertex3f(-300, -0.05f,  300);
        glTexCoord2f( 300 / tile,  300 / tile); glVertex3f( 300, -0.05f,  300);
        glTexCoord2f( 300 / tile, -300 / tile); glVertex3f( 300, -0.05f, -300);
    glEnd();
    glDisable(GL_TEXTURE_2D);
}

void drawRoad(Level& lvl) {
    glColor3f(1, 1, 1);
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, asphaltTexture);
    glNormal3f(0, 1, 0);
    const float tile = 6.0f; // world units per texture repeat, along the road's length
    float vCoord = 0.0f;      // running length so the texture tiles seamlessly segment to segment
    glBegin(GL_QUADS);
    for (size_t i = 0; i + 1 < lvl.waypoints.size(); i++) {
        Vec3 a = lvl.waypoints[i];
        Vec3 b = lvl.waypoints[i + 1];
        Vec3 dir = (b - a).normalized();
        Vec3 perp(-dir.z, 0, dir.x); // rotate direction 90 degrees in the XZ plane
        Vec3 half = perp * (lvl.trackWidth * 0.5f);
        Vec3 a1 = a + half, a2 = a - half;
        Vec3 b1 = b + half, b2 = b - half;

        float v0 = vCoord;
        float v1 = vCoord + dist(a, b) / tile;
        vCoord = v1;

        glTexCoord2f(0, v0); glVertex3f(a1.x, 0.01f, a1.z);
        glTexCoord2f(1, v0); glVertex3f(a2.x, 0.01f, a2.z);
        glTexCoord2f(1, v1); glVertex3f(b2.x, 0.01f, b2.z);
        glTexCoord2f(0, v1); glVertex3f(b1.x, 0.01f, b1.z);
    }
    glEnd();
    glDisable(GL_TEXTURE_2D);
}

void drawCheckpointGates(Level& lvl) {
    for (size_t i = 1; i < lvl.waypoints.size(); i++) {
        Vec3 p = lvl.waypoints[i];
        bool passed = (int)i < nextCheckpoint;
        bool isFinish = (i == lvl.waypoints.size() - 1);
        if (passed) glColor3f(0.2f, 0.9f, 0.2f);
        else if (isFinish) glColor3f(0.95f, 0.85f, 0.1f);
        else glColor3f(0.9f, 0.3f, 0.2f);

        glPushMatrix();
        glTranslatef(p.x, 0, p.z);
        glPushMatrix(); glTranslatef(-lvl.trackWidth * 0.5f, 2.0f, 0); glScalef(0.3f, 4.0f, 0.3f); glutSolidCube(1.0); glPopMatrix();
        glPushMatrix(); glTranslatef( lvl.trackWidth * 0.5f, 2.0f, 0); glScalef(0.3f, 4.0f, 0.3f); glutSolidCube(1.0); glPopMatrix();
        glPushMatrix(); glTranslatef(0, 4.0f, 0); glScalef(lvl.trackWidth, 0.3f, 0.3f); glutSolidCube(1.0); glPopMatrix();
        glPopMatrix();
    }
}

void drawObstacles(Level& lvl) {
    glColor3f(0.9f, 0.45f, 0.05f);
    for (size_t i = 0; i < lvl.obstacles.size(); i++) {
        Obstacle& ob = lvl.obstacles[i];
        glPushMatrix();
        glTranslatef(ob.position.x, 0, ob.position.z);
        glRotatef(-90, 1, 0, 0);
        gluCylinder(quadric, ob.radius, 0.1f, ob.radius * 2.2f, 12, 4);
        glPopMatrix();
    }
}

// Draws a w*h*d box centered at the origin, with correct per-face
// normals AND 0..1 texture coordinates on every face. glutSolidCube
// doesn't expose texture coordinates, so the car body needs this
// instead in order to show the paint/stripe texture.
void drawTexturedBox(float w, float h, float d) {
    float x = w * 0.5f, y = h * 0.5f, z = d * 0.5f;
    glBegin(GL_QUADS);
        // front (+z)
        glNormal3f(0, 0, 1);
        glTexCoord2f(0, 0); glVertex3f(-x, -y, z);
        glTexCoord2f(1, 0); glVertex3f( x, -y, z);
        glTexCoord2f(1, 1); glVertex3f( x,  y, z);
        glTexCoord2f(0, 1); glVertex3f(-x,  y, z);
        // back (-z)
        glNormal3f(0, 0, -1);
        glTexCoord2f(0, 0); glVertex3f( x, -y, -z);
        glTexCoord2f(1, 0); glVertex3f(-x, -y, -z);
        glTexCoord2f(1, 1); glVertex3f(-x,  y, -z);
        glTexCoord2f(0, 1); glVertex3f( x,  y, -z);
        // left (-x)
        glNormal3f(-1, 0, 0);
        glTexCoord2f(0, 0); glVertex3f(-x, -y, -z);
        glTexCoord2f(1, 0); glVertex3f(-x, -y,  z);
        glTexCoord2f(1, 1); glVertex3f(-x,  y,  z);
        glTexCoord2f(0, 1); glVertex3f(-x,  y, -z);
        // right (+x)
        glNormal3f(1, 0, 0);
        glTexCoord2f(0, 0); glVertex3f(x, -y,  z);
        glTexCoord2f(1, 0); glVertex3f(x, -y, -z);
        glTexCoord2f(1, 1); glVertex3f(x,  y, -z);
        glTexCoord2f(0, 1); glVertex3f(x,  y,  z);
        // top (+y)
        glNormal3f(0, 1, 0);
        glTexCoord2f(0, 0); glVertex3f(-x, y,  z);
        glTexCoord2f(1, 0); glVertex3f( x, y,  z);
        glTexCoord2f(1, 1); glVertex3f( x, y, -z);
        glTexCoord2f(0, 1); glVertex3f(-x, y, -z);
        // bottom (-y)
        glNormal3f(0, -1, 0);
        glTexCoord2f(0, 0); glVertex3f(-x, -y, -z);
        glTexCoord2f(1, 0); glVertex3f( x, -y, -z);
        glTexCoord2f(1, 1); glVertex3f( x, -y,  z);
        glTexCoord2f(0, 1); glVertex3f(-x, -y,  z);
    glEnd();
}

// A wheel with a hubcap on both faces so it looks right from either side
// without needing to reason about which way is "outward."
void drawWheel(float radius, float width) {
    glPushMatrix();
    glTranslatef(0, 0, -width * 0.5f);
    glColor3f(0.05f, 0.05f, 0.05f);
    gluCylinder(quadric, radius, radius, width, 14, 2);
    glColor3f(0.72f, 0.72f, 0.76f);
    gluDisk(quadric, 0, radius * 0.55f, 14, 1);
    glPushMatrix();
    glTranslatef(0, 0, width);
    gluDisk(quadric, 0, radius * 0.55f, 14, 1);
    glPopMatrix();
    glPopMatrix();
}

void drawCar() {
    glPushMatrix();
    glTranslatef(car.position.x, 0, car.position.z);
    glRotatef(car.heading, 0, 1, 0);

    const float wheelRadius = 0.45f;
    const float wheelWidth  = 0.32f;
    const float chassisW = 1.9f, chassisH = 0.62f, chassisL = 3.7f;
    const float roofW = 1.6f, roofH = 0.58f, roofL = 2.0f;
    const float chassisY = wheelRadius + chassisH * 0.5f;
    const float roofY = wheelRadius + chassisH + roofH * 0.5f;
    const float roofZOffset = -0.35f; // set back toward the rear, leaving a hood up front

    // chassis — two-tone SUV paint (dark cladding low, light paint above)
    glColor3f(1, 1, 1);
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, suvBodyTexture);
    glPushMatrix();
    glTranslatef(0, chassisY, 0);
    drawTexturedBox(chassisW, chassisH, chassisL);
    glPopMatrix();
    glDisable(GL_TEXTURE_2D);

    // roof / cabin, set back to leave a flat hood at the front
    glColor3f(0.92f, 0.90f, 0.85f);
    glPushMatrix();
    glTranslatef(0, roofY, roofZOffset);
    glScalef(roofW, roofH, roofL);
    glutSolidCube(1.0);
    glPopMatrix();

    // front & rear bumpers, plus a grille block up front
    glColor3f(0.08f, 0.08f, 0.09f);
    glPushMatrix();
    glTranslatef(0, wheelRadius + 0.15f, chassisL * 0.5f + 0.05f);
    glScalef(chassisW + 0.1f, 0.3f, 0.15f);
    glutSolidCube(1.0);
    glPopMatrix();
    glPushMatrix();
    glTranslatef(0, wheelRadius + 0.15f, -chassisL * 0.5f - 0.05f);
    glScalef(chassisW + 0.1f, 0.3f, 0.15f);
    glutSolidCube(1.0);
    glPopMatrix();
    glPushMatrix();
    glTranslatef(0, chassisY + 0.05f, chassisL * 0.5f + 0.02f);
    glScalef(chassisW * 0.55f, chassisH * 0.55f, 0.06f);
    glutSolidCube(1.0);
    glPopMatrix();

    // headlights (lit via emission so they read as "on") and taillights
    float hlx = chassisW * 0.38f;
    float hly = chassisY + 0.05f;
    GLfloat glowOn[]  = { 1.0f, 0.95f, 0.6f, 1.0f };
    GLfloat glowOff[] = { 0.0f, 0.0f, 0.0f, 1.0f };
    for (int side = -1; side <= 1; side += 2) {
        glMaterialfv(GL_FRONT, GL_EMISSION, glowOn);
        glColor3f(1.0f, 0.95f, 0.7f);
        glPushMatrix();
        glTranslatef(side * hlx, hly, chassisL * 0.5f + 0.05f);
        glScalef(0.22f, 0.14f, 0.06f);
        glutSolidCube(1.0);
        glPopMatrix();
        glMaterialfv(GL_FRONT, GL_EMISSION, glowOff);
    }
    for (int side = -1; side <= 1; side += 2) {
        glColor3f(0.75f, 0.1f, 0.1f);
        glPushMatrix();
        glTranslatef(side * hlx, hly, -chassisL * 0.5f - 0.05f);
        glScalef(0.2f, 0.16f, 0.06f);
        glutSolidCube(1.0);
        glPopMatrix();
    }

    // roof rack — two side rails plus three cross bars
    glColor3f(0.08f, 0.08f, 0.08f);
    float rackY = roofY + roofH * 0.5f + 0.04f;
    for (int side = -1; side <= 1; side += 2) {
        glPushMatrix();
        glTranslatef(side * roofW * 0.42f, rackY, roofZOffset);
        glScalef(0.06f, 0.06f, roofL * 0.95f);
        glutSolidCube(1.0);
        glPopMatrix();
    }
    for (int i = 0; i < 3; i++) {
        float zc = roofZOffset - roofL * 0.35f + i * (roofL * 0.35f);
        glPushMatrix();
        glTranslatef(0, rackY, zc);
        glScalef(roofW * 0.8f, 0.05f, 0.06f);
        glutSolidCube(1.0);
        glPopMatrix();
    }

    // spare tire mounted on the tailgate — the classic Land Cruiser silhouette
    glPushMatrix();
    glTranslatef(0, chassisY, -chassisL * 0.5f - 0.16f);
    glRotatef(90, 0, 1, 0);
    drawWheel(wheelRadius * 0.85f, 0.18f);
    glPopMatrix();

    // wheels
    float wx = chassisW * 0.5f - 0.05f;
    float wz = chassisL * 0.34f;
    Vec3 wheelPos[4] = {
        Vec3(-wx, wheelRadius, wz), Vec3(wx, wheelRadius, wz),
        Vec3(-wx, wheelRadius, -wz), Vec3(wx, wheelRadius, -wz)
    };
    for (int i = 0; i < 4; i++) {
        glPushMatrix();
        glTranslatef(wheelPos[i].x, wheelPos[i].y, wheelPos[i].z);
        glRotatef(90, 0, 1, 0);
        drawWheel(wheelRadius, wheelWidth);
        glPopMatrix();
    }

    glPopMatrix();
}

// ----------------------------------------------------------------
// SECTION 8 — SKY BACKGROUND + HUD (2D overlays)
// ----------------------------------------------------------------

// Full-screen vertical gradient, drawn before the 3D scene each frame
// so the horizon looks like open sky instead of a flat clear color.
void drawSky() {
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_FOG);
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, windowWidth, 0, windowHeight);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    glBegin(GL_QUADS);
        glColor3f(0.35f, 0.55f, 0.85f); // deep sky at the top
        glVertex2f(0, windowHeight);
        glVertex2f(windowWidth, windowHeight);
        glColor3f(0.78f, 0.88f, 0.98f); // pale near the horizon
        glVertex2f(windowWidth, windowHeight * 0.45f);
        glVertex2f(0, windowHeight * 0.45f);
    glEnd();

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_FOG);
}

void drawText(float x, float y, const std::string& text, void* font = GLUT_BITMAP_HELVETICA_18) {
    glRasterPos2f(x, y);
    for (size_t i = 0; i < text.size(); i++) glutBitmapCharacter(font, text[i]);
}

// Solid translucent rectangle — the building block for HUD panels.
void drawRect(float x, float y, float w, float h, float r, float g, float b, float a) {
    glColor4f(r, g, b, a);
    glBegin(GL_QUADS);
        glVertex2f(x, y); glVertex2f(x + w, y);
        glVertex2f(x + w, y + h); glVertex2f(x, y + h);
    glEnd();
}

void drawRectOutline(float x, float y, float w, float h, float r, float g, float b, float a) {
    glColor4f(r, g, b, a);
    glBegin(GL_LINE_LOOP);
        glVertex2f(x, y); glVertex2f(x + w, y);
        glVertex2f(x + w, y + h); glVertex2f(x, y + h);
    glEnd();
}

void drawHUD() {
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_FOG);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, windowWidth, 0, windowHeight);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    char buf[128];

    if (gameState == MENU) {
        float pw = 460, ph = 220;
        float px = windowWidth / 2 - pw / 2, py = windowHeight / 2 - ph / 2;
        drawRect(px, py, pw, ph, 0.05f, 0.05f, 0.08f, 0.65f);
        drawRect(px, py + ph - 8, pw, 8, 0.9f, 0.25f, 0.15f, 0.95f); // accent stripe
        drawRectOutline(px, py, pw, ph, 1, 1, 1, 0.25f);

        glColor4f(1, 1, 1, 1);
        drawText(px + 65, py + ph - 55, "VELOCITY RUSH", GLUT_BITMAP_TIMES_ROMAN_24);
        glColor4f(0.85f, 0.85f, 0.9f, 1);
        drawText(px + 90, py + ph - 100, "Press ENTER to start racing");
        drawText(px + 40, py + 70, "W / S   -   accelerate & brake");
        drawText(px + 40, py + 48, "A / D   -   steer left & right");
        drawText(px + 40, py + 26, "R  -  restart          ESC  -  quit");
    } else if (gameState == PLAYING) {
        Level& lvl = levels[currentLevelIndex];

        drawRect(15, windowHeight - 135, 260, 120, 0.05f, 0.05f, 0.08f, 0.55f);
        drawRectOutline(15, windowHeight - 135, 260, 120, 1, 1, 1, 0.2f);

        glColor4f(1, 1, 1, 1);
        drawText(28, windowHeight - 25, lvl.name, GLUT_BITMAP_HELVETICA_18);
        sprintf(buf, "Level %d/%d   Time %.1fs", currentLevelIndex + 1, (int)levels.size(), levelTimer);
        drawText(28, windowHeight - 48, buf);

        // speed bar: green at low speed, sliding to red near top speed
        float sr = fabs(car.speed) / MAX_SPEED;
        if (sr > 1) sr = 1;
        drawText(28, windowHeight - 92, "Speed");
        drawRect(85, windowHeight - 98, 175, 12, 0.2f, 0.2f, 0.22f, 0.9f);
        float barR = sr < 0.5f ? 0.2f + sr * 1.2f : 0.9f;
        float barG = sr < 0.5f ? 0.75f : 0.85f - (sr - 0.5f) * 1.4f;
        drawRect(85, windowHeight - 98, 175 * sr, 12, barR, barG, 0.2f, 0.95f);

        // checkpoint progress bar
        float cpRatio = (float)(nextCheckpoint - 1) / (float)(lvl.waypoints.size() - 1);
        drawText(28, windowHeight - 118, "Progress");
        drawRect(95, windowHeight - 124, 165, 8, 0.2f, 0.2f, 0.22f, 0.9f);
        drawRect(95, windowHeight - 124, 165 * cpRatio, 8, 0.3f, 0.8f, 0.4f, 0.95f);

    } else if (gameState == LEVEL_COMPLETE) {
        float pw = 420, ph = 130;
        float px = windowWidth / 2 - pw / 2, py = windowHeight / 2 - ph / 2;
        drawRect(px, py, pw, ph, 0.05f, 0.08f, 0.05f, 0.7f);
        drawRectOutline(px, py, pw, ph, 0.3f, 0.9f, 0.3f, 0.4f);
        glColor4f(0.6f, 1.0f, 0.6f, 1);
        sprintf(buf, "LEVEL COMPLETE - %.1fs", lastLevelTime);
        drawText(px + 45, py + ph - 55, buf, GLUT_BITMAP_TIMES_ROMAN_24);
        glColor4f(1, 1, 1, 1);
        drawText(px + 90, py + 35, "Press ENTER to continue");
    } else if (gameState == ALL_COMPLETE) {
        float pw = 460, ph = 130;
        float px = windowWidth / 2 - pw / 2, py = windowHeight / 2 - ph / 2;
        drawRect(px, py, pw, ph, 0.08f, 0.07f, 0.02f, 0.7f);
        drawRectOutline(px, py, pw, ph, 0.95f, 0.8f, 0.2f, 0.5f);
        glColor4f(1.0f, 0.85f, 0.3f, 1);
        drawText(px + 45, py + ph - 55, "ALL LEVELS COMPLETE!", GLUT_BITMAP_TIMES_ROMAN_24);
        glColor4f(1, 1, 1, 1);
        drawText(px + 55, py + 35, "Press R to race again from Level 1");
    }

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);

    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_FOG);
}

// ----------------------------------------------------------------
// SECTION 9 — GLUT CALLBACKS
// ----------------------------------------------------------------
void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    drawSky();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    if (gameState != MENU) {
        applyCamera();
        updateHeadlights();

        Level& lvl = levels[currentLevelIndex];
        drawGround();
        drawRoad(lvl);
        drawCheckpointGates(lvl);
        drawObstacles(lvl);
        drawCar();
    } else {
        gluLookAt(0, 30, 40, 0, 0, 0, 0, 1, 0);
        drawGround();
    }

    drawHUD();
    glutSwapBuffers();
}

void reshape(int w, int h) {
    windowWidth = w;
    windowHeight = (h == 0) ? 1 : h;
    glViewport(0, 0, windowWidth, windowHeight);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, (double)windowWidth / windowHeight, 0.1, 500.0);
    glMatrixMode(GL_MODELVIEW);
}

void keyboardDown(unsigned char key, int, int) {
    key = (unsigned char)tolower(key);
    keyDown[key] = true;
    if (key == 13) { // Enter
        if (gameState == MENU) startLevel(0);
        else if (gameState == LEVEL_COMPLETE) {
            if (currentLevelIndex + 1 < (int)levels.size()) startLevel(currentLevelIndex + 1);
            else gameState = ALL_COMPLETE;
        }
    }
    if (key == 'r') {
        if (gameState == PLAYING || gameState == LEVEL_COMPLETE) startLevel(currentLevelIndex);
        else if (gameState == ALL_COMPLETE) startLevel(0);
    }
    if (key == 27) exit(0); // Esc
}

void keyboardUp(unsigned char key, int, int) { keyDown[(unsigned char)tolower(key)] = false; }
void specialDown(int key, int, int) { specialKeyDown[key] = true; }
void specialUp(int key, int, int) { specialKeyDown[key] = false; }

void timerFunc(int) {
    int now = glutGet(GLUT_ELAPSED_TIME);
    float dt = (now - prevTimeMs) / 1000.0f;
    prevTimeMs = now;
    if (dt > 0.1f) dt = 0.1f; // clamp to avoid big jumps after a stall

    if (gameState == PLAYING) updateCar(dt);

    glutPostRedisplay();
    glutTimerFunc(16, timerFunc, 0);
}

// ----------------------------------------------------------------
// SECTION 10 — ENTRY POINT
// ----------------------------------------------------------------
int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(windowWidth, windowHeight);
    glutCreateWindow("Velocity Rush - 3D Racing Game");

    initGL();
    buildLevels();

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboardDown);
    glutKeyboardUpFunc(keyboardUp);
    glutSpecialFunc(specialDown);
    glutSpecialUpFunc(specialUp);

    prevTimeMs = glutGet(GLUT_ELAPSED_TIME);
    glutTimerFunc(16, timerFunc, 0);

    glutMainLoop();
    return 0;
}
