#include "Robot.h"

Robot::Robot(Simulator *sim, std::string name) {
    this->sim = sim;
    this->name = name;
    handle = sim->getHandle(name);

    if (LOG) {
        FILE *data =  fopen("gt.txt", "wt");
        if (data!=NULL)
            fclose(data);
        data =  fopen("sonar.txt", "wt");
        if (data!=NULL)
            fclose(data);
        data =  fopen("points.txt", "wt");
            if (data!=NULL)
              fclose(data);
    }

    /* Get handles of sensors and actuators */
    encoderHandle[0] = sim->getHandle("Pioneer_p3dx_leftWheel");
    encoderHandle[1] = sim->getHandle("Pioneer_p3dx_rightWheel");
    std::cout << "Left Encoder: "<< encoderHandle[0] << std::endl;
    std::cout << "Right Encoder: "<< encoderHandle[1] << std::endl;

    /* Get handles of sensors and actuators */
    motorHandle[0] = sim->getHandle("Pioneer_p3dx_leftMotor");
    motorHandle[1] = sim->getHandle("Pioneer_p3dx_rightMotor");
    std::cout << "Left motor: "<<  motorHandle[0] << std::endl;
    std::cout << "Right motor: "<<  motorHandle[1] << std::endl;

    simxChar sensorName[31];
    /* Connect to sonar sensors. Requires a handle per sensor. Sensor name: Pioneer_p3dx_ultrasonicSensorX, where
     * X is the sensor number, from 1 - 16 */
    for(int i = 0; i < 16; i++)
    {
        sprintf(sensorName,"%s%d","Pioneer_p3dx_ultrasonicSensor",i+1);
        sonarHandle[i] = sim->getHandle(sensorName);
        if (sonarHandle[i] == -1)
            std::cout <<  "Error on connecting to sensor " + i+1 << std::endl;
        else
        {
//            std::cout << "Connected to sensor" << std::endl;
        }
        sim->readProximitySensor(sonarHandle[i],NULL,NULL,0);
    }

    /* Get the robot current absolute position */
    sim->getObjectPosition(handle,robotPosition);
    sim->getObjectOrientation(handle,robotOrientation);

    initialPose[0]=robotPosition[0];
    initialPose[1]=robotPosition[1];
    initialPose[2]=robotOrientation[2];
    for(int i = 0; i < 3; i++)
    {
        robotLastPosition[i] = robotPosition[i];
    }

    /* Get the encoder data */
    sim->getJointPosition(motorHandle[0],&encoder[0]);
    sim->getJointPosition(motorHandle[1],&encoder[1]);
    tree = newNode(robotPosition[0],robotPosition[1]);
    currentNode = tree;
}

void Robot::update() {

    updateSensors();
    updatePose();
    if(robotState == EXPLORATION){
        srt();
    }else{
        std::cout << "Exploration done" << std::endl;
        drive(0,0);
    }
}

void Robot::updateSensors()
{
    node *treeB = tree;
    node *currentNodeB = currentNode;

    /* Update sonars */
    for(int i = 0; i < NUM_SONARS; i++)
    {
        simxUChar state;       // sensor state
        simxFloat coord[3];    // detected point coordinates [only z matters]
        /* simx_opmode_streaming -> Non-blocking mode */

        /* read the proximity sensor
         * detectionState: pointer to the state of detection (0=nothing detected)
         * detectedPoint: pointer to the coordinates of the detected point (relative to the frame of reference of the sensor) */
        if (sim->readProximitySensor(sonarHandle[i],&state,coord,1)==1)
        {
            if(state > 0)
                sonarReadings[i] = coord[2];
            else
                sonarReadings[i] = -1;
        }
    }
    //std::cout << "Connected to sensor" << std::endl;
    /* Update encoder data */
    lastEncoder[0] = encoder[0];
    lastEncoder[1] = encoder[1];

    /* Get the encoder data */
    if (sim->getJointPosition(motorHandle[0], &encoder[0]) == 1)
        //std::cout << "ok left enconder"<< encoder[0] << std::endl;  // left
    if (sim->getJointPosition(motorHandle[1], &encoder[1]) == 1)
        //std::cout << "ok right enconder"<< encoder[1] << std::endl;  // right

    tree = treeB;
    currentNode = currentNodeB;
}

void Robot::updatePose()
{
    for(int i = 0; i < 3; i++)
    {
        robotLastPosition[i] = robotPosition[i];
    }

    /* Get the robot current position and orientation */
    sim->getObjectPosition(handle,robotPosition);
    sim->getObjectOrientation(handle,robotOrientation);
}

void Robot::writeGT() {
    /* write data to file */
    /* file format: robotPosition[x] robotPosition[y] robotPosition[z] robotLastPosition[x] robotLastPosition[y] robotLastPosition[z]
     *              encoder[0] encoder[1] lastEncoder[0] lastEncoder[1] */

    if (LOG) {
        FILE *data =  fopen("gt.txt", "at");
        if (data!=NULL)
        {
            for (int i=0; i<3; ++i)
                fprintf(data, "%.2f\t",robotPosition[i]);
            for (int i=0; i<3; ++i)
                fprintf(data, "%.2f\t",robotLastPosition[i]);
            for (int i=0; i<3; ++i)
                fprintf(data, "%.2f\t",robotOrientation[i]);
            for (int i=0; i<2; ++i)
                fprintf(data, "%.2f\t",encoder[i]);
            for (int i=0; i<2; ++i)
                fprintf(data, "%.2f\t",lastEncoder[i]);
            fprintf(data, "\n");
            fflush(data);
            fclose(data);
          }
          else
            std::cout << "Unable to open file" << std::endl;
    }
}

void Robot::writeSonars() {
    /* write data to file */
    /* file format: robotPosition[x] robotPosition[y] robotPosition[z] robotLastPosition[x] robotLastPosition[y] robotLastPosition[z]
     *              encoder[0] encoder[1] lastEncoder[0] lastEncoder[1] */
    if (LOG) {
        FILE *data =  fopen("sonar.txt", "at");
        if (data!=NULL)
        {
            if (data!=NULL)
            {
                for (int i=0; i<NUM_SONARS; ++i)
                    fprintf(data, "%.2f\t",sonarReadings[i]);
                fprintf(data, "\n");
                fflush(data);
                fclose(data);
            }
        }
    }
}

void Robot::printPose() {
    std::cout << "[" << robotPosition[0] << ", " << robotPosition[1] << ", " << robotOrientation[2] << "]" << std::endl;
}

void Robot::move(float vLeft, float vRight) {
    sim->setJointTargetVelocity(motorHandle[0], vLeft);
    sim->setJointTargetVelocity(motorHandle[1], vRight);
}

void Robot::stop() {
    sim->setJointTargetVelocity(motorHandle[0], 0);
    sim->setJointTargetVelocity(motorHandle[1], 0);
}

void Robot::drive(double vLinear, double vAngular)
{
    sim->setJointTargetVelocity(motorHandle[0], vLToDrive(vLinear,vAngular));
    sim->setJointTargetVelocity(motorHandle[1], vRToDrive(vLinear,vAngular));
}

double Robot::vRToDrive(double vLinear, double vAngular)
{
    return (((2*vLinear)+(L*vAngular))/2*R);
}

double Robot::vLToDrive(double vLinear, double vAngular)
{
    return (((2*vLinear)-(L*vAngular))/2*R);

}

void Robot::srt(){
    float minDist = 999;
    int indexMin = -1;
    float minLeft = 999;
    float minRight = 999;

    for (int i=3; i<5; ++i){
        if(sonarReadings[i] > 0 && sonarReadings[i] < minLeft && i < 4){
            minLeft = sonarReadings[i];
        }
        if(sonarReadings[i] > 0 && sonarReadings[i] < minRight && i >= 4){
            minRight = sonarReadings[i];
        }
        if(sonarReadings[i] > 0 && sonarReadings[i] < minDist){
          indexMin = i;
          minDist = sonarReadings[i];
        }
    }
    //obstacle is close
    if(minDist < OBSTACLE_MARGIN && minDist > 0){
        if(!robotObstacleFound){
            robotObstacleFound = true;
            robotTurning =  rand() % 2;
            currentNode->x = robotPosition[0];
            currentNode->y = robotLastPosition[1];
            std::cout << "OBSTACLE CLOSE: " << std::endl;
        }
    }else{
        if(robotObstacleFound){
            robotObstacleFound = false;
            robotTurning = TURNING_UNDEFINED;
        }
    }
    float alfa, beta, p, dx, dy;
    float v, w, kRho = 50, kAlfa = 40, kBeta= -5;
    dx = currentNode->x - robotPosition[0];
    dy = currentNode->y - robotPosition[1];
    p = dx*dx + dy*dy;
    alfa = atan2(dy,dx) - robotOrientation[2] ;
    beta = - robotOrientation[2] - alfa;
    v = kRho*p;
    if(v>20)
    {
        v = 20;
    }
    w = kAlfa*alfa + kBeta*beta;

    float delta = atan2(dy,dx)*atan2(dy,dx) - robotOrientation[2]*robotOrientation[2];

    if(robotTurning == TURNING_UNDEFINED){
        if(delta < 0){
              robotTurning = TURNING_LEFT;
        }else{
              robotTurning = TURNING_RIGHT;
        }
    }

    if(robotTurning == TURNING_RIGHT){
        w = -20;
    }else{
        w = 20;
    }

    if(alfa*alfa < MARGIN
            //&& !robotObstacleFound
            ){
        //reto
        robotTurning = TURNING_UNDEFINED;
        w = 0;
    }else{
        //
        v = 0;
    }

    if(p < LIMIAR && w == 0){
        v = 1.0;
    }
//    std::cout << "delta*delta = " << delta*delta << " p: " << p << std::endl;

    if(p < LIMIAR){
        int i = 0;
        bool foundNewQ = false;
        float q,x,y,r;
        int theta;
        while(i < MAX_I && foundNewQ == false){
            theta = rand() % 8; //0 - 7
            r = sonarReadings[theta];
            if(r < 0){
                r = MAX_SONAR_READING;
            }

            q = r*ALPHA;
            x = robotPosition[0] + (q) * cos(robotOrientation[2] + (sonarAngles[theta]*M_PI)/180);
            y = robotPosition[1] + (q) * sin(robotOrientation[2] + (sonarAngles[theta]*M_PI)/180);

            if(q >= DMIN && !isVisited(tree,x,y)){
                foundNewQ = true;
                break;
            }
            i++;
        }

        if(foundNewQ == false){
            //goto parent node
            currentNode = getParent(tree);
            std::cout << "NO Q FOUND -> GOTO PARENT  : " << currentNode->x << ", " << currentNode->y << std::endl;

        }else{
            std::cout << "NEW NODE FOUND: (" << x << "," << y << ")" << std::endl;
            //new node
            if(currentNode->child == NULL){
                currentNode = addChild(currentNode, x,y);
            }else{
                currentNode = addSibling(currentNode, x,y);
            }
        }
    }else{
        drive(v,w);
    }
}

node * Robot::newNode(float x, float y)
{
    node *new_node = (node*)malloc(sizeof(node));

    if ( new_node ) {
        new_node->next = NULL;
        new_node->child = NULL;
        new_node->x = x;
        new_node->y = y;
    }

    return new_node;
}

node * Robot::addSibling(node * n, float x, float y)
{
    if ( n == NULL )
        return NULL;

    while (n->next)
        n = n->next;

    return (n->next = newNode(x,y));
}

node * Robot::addChild(node * n, float x, float y)
{
    if ( n == NULL )
        return NULL;

    if ( n->child )
        return addSibling(n->child, x,y);
    else
        return (n->child = newNode(x,y));
}

void Robot::printTree(node * n)
{
    if ( n == NULL ){
        return;
    }else{
        std::cout << " x= "<< n->x <<" y= "<< n->y << std::endl;
        while(n->next){
            std::cout << " x= "<< tree->x <<" y= "<< tree->y << std::endl;
            n = n->next;
        }

        printTree(n->child);

    }
}

bool Robot::isVisited(node *tree, float x, float y){
    if ( tree == NULL ){
        return NULL;
    }else{
        bool check = false;

        if (tree->child != NULL){
            if (tree->x >= x - VISITED_MARGIN && tree->x <= x + VISITED_MARGIN &&
                tree->y >= y - VISITED_MARGIN && tree->y <= y + VISITED_MARGIN){
                return true;
            } else {
                check = isVisited(tree->child,x,y);
            }
        }

        if (check == false)
        {
            while (tree->next != NULL){
                if (tree->x >= x - VISITED_MARGIN && tree->x <= x + VISITED_MARGIN &&
                    tree->y >= y - VISITED_MARGIN && tree->y <= y + VISITED_MARGIN){
                    return true;
                } else {
                  check = isVisited(tree->next,x,y);

                  if (check == false)
                  {
                    tree = tree->next;
                  } else {
                      return check;
                  }
                }
            }
        }
        return check;
    }
}

node * Robot::getParent(node *tree){
    if ( tree == NULL ){
        return NULL;
    } else {
        node * parent = NULL;

        if (tree->child != NULL){
            if (currentNode->x == tree->child->x &&
                currentNode->y == tree->child->y){
                parent = tree;
            } else {
                parent = getParent(tree->child);
            }
        }

        if (parent == NULL)
        {
            parent = tree;
            node * parent_child = NULL;

            while (tree->next != NULL){
                if (currentNode->x == tree->next->x &&
                    currentNode->y == tree->next->y){
                    return parent;
                } else {
                  parent_child = getParent(tree->next);

                  if (parent_child == NULL)
                  {
                    tree = tree->next;
                  } else {
                      return parent_child;
                  }
                }
            }
        }

        return parent;
    }
}

void Robot::writePointsPerSonars() {
  float x, y;
  if (LOG) {
    FILE *data =  fopen("points.txt", "at");

    if (data!=NULL){
      // Somente 1 sonar por enquanto, para testes
      for (int i=0; i<8; ++i){
        if(sonarReadings[i] > 0){
          x = robotPosition[0] + (sonarReadings[i] + R) * cos(robotOrientation[2] + (sonarAngles[i]*M_PI)/180);
          y = robotPosition[1] + (sonarReadings[i] + R) * sin(robotOrientation[2] + (sonarAngles[i]*M_PI)/180);
          fprintf(data, "%.4f \t %.4f \n", x, y);
        }
      }
      fflush(data);
      fclose(data);
    }
  }
}
