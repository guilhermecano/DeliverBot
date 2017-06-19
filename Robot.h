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


struct node {
    float x,y;
    struct node *next;
    struct node *child;
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
    void srt();
    node * newNode(float x, float y);
    node * addSibling(node *, float x, float y);
    node * addChild(node *, float x, float y);
    void printTree(node * root);
    bool isVisited(node *root, float x, float y);
    node * getParent(node *tree);
private:
    const float L = 0.381;                                   // distance between wheels
    const float R    = 0.0975;                                  // wheel radius
    const float VISITED_MARGIN = 0.5;
    const float MARGIN = 0.1;
    const float LIMIAR = 0.1;
    const int EXPLORATION = 1;
    const int EXPLORATION_DONE = 2;
    const int MAX_I = 8;
    const float MAX_SONAR_READING = 1.0;
    const float ALPHA = 0.9;
    const float DMIN = 0.7;
    int robotState = EXPLORATION;


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
};

#endif // ROBOT_H
