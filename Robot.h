#ifndef ROBOT_H
#define ROBOT_H

#define NUM_SONARS  16
#define LOG         1

#include <fstream>
#include <iostream>
#include<cstdlib>
#include <ctime>

using namespace std;
#include "Simulator.h"

#include <math.h>
extern "C" {
   #include "extApi.h"
   #include "v_repLib.h"
}

struct point{
    float x,y;
};
typedef struct point points[180];
struct node {
    float x,y;
    struct node *next;
    struct node *child;
    points coordinates;
};

typedef struct node node;

class Robot
{
public:
    Robot(Simulator *sim, std::string name);
    void update();
    void updateSensors();
    void updatePose();
    void printPose();
    void writeGT();
    void writeSonars();
    void move(float vLeft, float vRight);
    double vRToDrive(double vLinear, double vAngular);
    double vLToDrive(double vLinear, double vAngular);
    void drive(double vLinear, double vAngular);
    void stop();
    bool obstacleCheck();
    void srt();
    node * newNode(float x, float y,points coordinates);
    node * addSibling(node *, float x, float y,points coordinates);
    node * addChild(node *, float x, float y,points coordinates);
    void printTree(node * root);
    bool isVisited(node *root, float x, float y);
    node * getParent(node *tree);
    void writePointsPerSonars();
    void writePointsPerLaser();
    void writeLaser();
    double distance(double dX0, double dY0, double dX1, double dY1);
    void voidObstacle();
private:
    const float L = 0.381;                                   // distance between wheels
    const float R    = 0.0975;                                  // wheel radius
    const float MARGIN = 0.1;
    const float LIMIAR = 0.5;
    const float OBSTACLE_MARGIN = 0.3;
    const float MAX_SONAR_READING = 1.0;
    const float MAX_LASER_READING = 3.0;
    const float ALPHA = 0.8;
    const float DMIN = 2.0;

    const float MAX_ANGULAR_VELOCITY = 25;
    const float MAX_LINEAR_VELOCITY = 50;

    const int EXPLORATION = 1;
    const int EXPLORATION_DONE = 2;
    int robotState = EXPLORATION;

    float theta = 0;
    int indexFindingQ = 0;
    const int MAX_I = 1000;
    const float MARGIN_THETA = 0.1;

    const int Q_NOT_FOUND = 0;
    const int Q_FOUND = 1;
    int srtState = Q_NOT_FOUND;

    const int FINDING_THETA = 0;
    const int GO_TO_THETA = 1;
    const int CHECK_Q = 2;
    const int Q_ARRIVED = 3;
    const int GO_TO_Q = 4;
    int srtRobotState = FINDING_THETA;

    const int TURNING_RIGHT = 0;
    const int TURNING_LEFT = 1;
    const int TURNING_UNDEFINED = 2;
    int robotTurning = TURNING_UNDEFINED;
    std::string name;
    Simulator *sim;

    simxInt handle;                                        // robot handle
    simxFloat velocity[2] = {1,1};                         // wheels' speed
    simxInt sonarHandle[16];                               // handle for sonars
    simxInt motorHandle[2] = {0,0};                        // [0]-> leftMotor [1]->rightMotor
    simxInt encoderHandle[2] = {0,0};
    simxFloat encoder[2] = {0,0};
    simxFloat lastEncoder[2] = {0,0};

    /* Robot Position  */
    simxFloat robotPosition[3] = {0,0,0};                    // current robot position
    simxFloat robotOrientation[3] = {0,0,0};                 // current robot orientation
    float initialPose[3] = {0,0,0};
    simxFloat robotLastPosition[3] = {0,0,0};                // last robot position
    float sonarReadings[8];
    node *tree;
    node *currentNode;
    float sonarAngles[9] = {90, 50, 30, 10, -10, -30, -50, -90};
    simxUChar* laserSignal;
    float laserReadings[180];
    simxInt laserHandle;
};

#endif // ROBOT_H
