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
const float ACCELERATION = 22.0f;
const float BRAKE_DECEL  = 34.0f;
const float FRICTION     = 14.0f;
const float OFFROAD_DRAG = 46.0f;
const float MAX_SPEED    = 42.0f;
const float MAX_REVERSE  = -16.0f;
const float TURN_RATE    = 130.0f; // degrees/sec at full speed

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
    car.radius = 1.2f;
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

    bool accel = keyDown['w'] || keyDown['W'] || specialKeyDown[GLUT_KEY_UP];
    bool brake = keyDown['s'] || keyDown['S'] || specialKeyDown[GLUT_KEY_DOWN];

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

    bool left  = keyDown['a'] || keyDown['A'] || specialKeyDown[GLUT_KEY_LEFT];
    bool right = keyDown['d'] || keyDown['D'] || specialKeyDown[GLUT_KEY_RIGHT];
    float speedFactor = car.speed / MAX_SPEED; // flips steering sense in reverse, like a real car
    if (left)  car.heading -= TURN_RATE * dt * speedFactor;
    if (right) car.heading += TURN_RATE * dt * speedFactor;

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

    quadric = gluNewQuadric();
}

// ----------------------------------------------------------------
// SECTION 7 — DRAWING
// ----------------------------------------------------------------
void drawGround() {
    glColor3f(0.25f, 0.55f, 0.25f);
    glNormal3f(0, 1, 0);
    glBegin(GL_QUADS);
        glVertex3f(-300, -0.05f, -300);
        glVertex3f(-300, -0.05f,  300);
        glVertex3f( 300, -0.05f,  300);
        glVertex3f( 300, -0.05f, -300);
    glEnd();
}

void drawRoad(Level& lvl) {
    glNormal3f(0, 1, 0);
    glColor3f(lvl.roadColor.x, lvl.roadColor.y, lvl.roadColor.z);
    glBegin(GL_QUADS);
    for (size_t i = 0; i + 1 < lvl.waypoints.size(); i++) {
        Vec3 a = lvl.waypoints[i];
        Vec3 b = lvl.waypoints[i + 1];
        Vec3 dir = (b - a).normalized();
        Vec3 perp(-dir.z, 0, dir.x); // rotate direction 90 degrees in the XZ plane
        Vec3 half = perp * (lvl.trackWidth * 0.5f);
        Vec3 a1 = a + half, a2 = a - half;
        Vec3 b1 = b + half, b2 = b - half;
        glVertex3f(a1.x, 0.01f, a1.z);
        glVertex3f(a2.x, 0.01f, a2.z);
        glVertex3f(b2.x, 0.01f, b2.z);
        glVertex3f(b1.x, 0.01f, b1.z);
    }
    glEnd();
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

void drawCar() {
    glPushMatrix();
    glTranslatef(car.position.x, 0.55f, car.position.z);
    glRotatef(car.heading, 0, 1, 0);

    glColor3f(0.85f, 0.1f, 0.15f);
    glPushMatrix(); glScalef(1.6f, 0.7f, 3.2f); glutSolidCube(1.0); glPopMatrix();

    glColor3f(0.65f, 0.85f, 0.95f);
    glPushMatrix();
    glTranslatef(0, 0.55f, -0.2f);
    glScalef(1.2f, 0.55f, 1.6f);
    glutSolidCube(1.0);
    glPopMatrix();

    glColor3f(0.05f, 0.05f, 0.05f);
    float wx = 0.85f, wy = -0.35f, wz = 1.1f;
    Vec3 wheelPos[4] = { Vec3(-wx, wy, wz), Vec3(wx, wy, wz), Vec3(-wx, wy, -wz), Vec3(wx, wy, -wz) };
    for (int i = 0; i < 4; i++) {
        glPushMatrix();
        glTranslatef(wheelPos[i].x, wheelPos[i].y, wheelPos[i].z);
        glRotatef(90, 0, 1, 0);
        gluCylinder(quadric, 0.35f, 0.35f, 0.3f, 10, 2);
        glPopMatrix();
    }
    glPopMatrix();
}

// ----------------------------------------------------------------
// SECTION 8 — HUD (2D text overlay)
// ----------------------------------------------------------------
void drawText(float x, float y, const std::string& text, void* font = GLUT_BITMAP_HELVETICA_18) {
    glRasterPos2f(x, y);
    for (size_t i = 0; i < text.size(); i++) glutBitmapCharacter(font, text[i]);
}

void drawHUD() {
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, windowWidth, 0, windowHeight);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    char buf[128];
    glColor3f(1, 1, 1);

    if (gameState == MENU) {
        drawText(windowWidth / 2 - 150, windowHeight / 2 + 40, "VELOCITY RUSH", GLUT_BITMAP_TIMES_ROMAN_24);
        drawText(windowWidth / 2 - 160, windowHeight / 2, "Press ENTER to start racing");
        drawText(windowWidth / 2 - 200, windowHeight / 2 - 30, "W/S accelerate & brake, A/D steer, R restart");
    } else if (gameState == PLAYING) {
        Level& lvl = levels[currentLevelIndex];
        sprintf(buf, "Level %d/%d - %s", currentLevelIndex + 1, (int)levels.size(), lvl.name.c_str());
        drawText(20, windowHeight - 30, buf);
        sprintf(buf, "Time: %.1fs", levelTimer);
        drawText(20, windowHeight - 55, buf);
        sprintf(buf, "Checkpoint: %d/%d", nextCheckpoint, (int)lvl.waypoints.size() - 1);
        drawText(20, windowHeight - 80, buf);
        sprintf(buf, "Speed: %.0f", fabs(car.speed));
        drawText(20, windowHeight - 105, buf);
    } else if (gameState == LEVEL_COMPLETE) {
        sprintf(buf, "LEVEL COMPLETE! Time: %.1fs", lastLevelTime);
        drawText(windowWidth / 2 - 140, windowHeight / 2 + 20, buf);
        drawText(windowWidth / 2 - 150, windowHeight / 2 - 10, "Press ENTER to continue");
    } else if (gameState == ALL_COMPLETE) {
        drawText(windowWidth / 2 - 150, windowHeight / 2 + 20, "ALL LEVELS COMPLETE!");
        drawText(windowWidth / 2 - 170, windowHeight / 2 - 10, "Press R to race again from Level 1");
    }

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
}

// ----------------------------------------------------------------
// SECTION 9 — GLUT CALLBACKS
// ----------------------------------------------------------------
void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
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
    keyDown[key] = true;
    if (key == 13) { // Enter
        if (gameState == MENU) startLevel(0);
        else if (gameState == LEVEL_COMPLETE) {
            if (currentLevelIndex + 1 < (int)levels.size()) startLevel(currentLevelIndex + 1);
            else gameState = ALL_COMPLETE;
        }
    }
    if (key == 'r' || key == 'R') {
        if (gameState == PLAYING || gameState == LEVEL_COMPLETE) startLevel(currentLevelIndex);
        else if (gameState == ALL_COMPLETE) startLevel(0);
    }
    if (key == 27) exit(0); // Esc
}

void keyboardUp(unsigned char key, int, int) { keyDown[key] = false; }
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
