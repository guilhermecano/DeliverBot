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
        data =  fopen("points_laser.txt", "wt");
            if (data!=NULL)
              fclose(data);
        data =  fopen("laser.txt", "wt");
            if (data!=NULL)
              fclose(data);
        data =  fopen("nodes.txt", "wt");
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

    points coordinates;
    for(int i = 0; i < 180; i++){
        float xPoint = robotPosition[0] + (laserReadings[i]) * cos(robotOrientation[2] + ((i - 90)*M_PI)/180);
        float yPoint = robotPosition[1] + (laserReadings[i]) * sin(robotOrientation[2] + ((i - 90)*M_PI)/180);

        coordinates[i].x = xPoint;
        coordinates[i].y = yPoint;
    }

    tree = newNode(robotPosition[0],robotPosition[1], coordinates);
    currentNode = tree;
    FILE *data =  fopen("nodes.txt", "at");
    if (data!=NULL)
    {
        fprintf(data, "%.2f\t",robotPosition[0]);
        fprintf(data, "%.2f\t",robotPosition[1]);
        fprintf(data, "\n");
        fflush(data);
        fclose(data);
    }

    //get laser readings
    laserHandle = sim->getHandle("Hokuyo_URG_04LX_UG01_ROS");
    simxInt signalSize;
    simxGetStringSignal(sim->getId(),"laserSignal",&laserSignal,&signalSize,simx_opmode_streaming);

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

void Robot::voidObstacle(){
    float v = MAX_LINEAR_VELOCITY;
    float w = 0;
    //void obstacle
    float angle[8] = {-10.0,-60.0,-60.0,-60.0,50.0,40.0,40.0,10.0};
    float minDist[8] = {0.1,0.3,0.4,0.4,0.4,0.4,0.4,0.1};
    for(int i = 1; i < 7; i++){
        if(sonarReadings[i] > 0 && sonarReadings[i] < minDist[i]){
          v = 10.0;
          w += angle[i];
        }
    }
    drive(v,w);
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
    simxInt signalSize;

    if (simxGetStringSignal(sim->getId(),"laserSignal",&laserSignal,&signalSize,simx_opmode_buffer)==simx_error_noerror){

       int cnt=180;
        int i = 0;
        int angle = 0;
        while (i < 180){
            laserReadings[angle]=((float*)laserSignal)[i];
            if(laserReadings[angle] == 0){
                laserReadings[angle] = -1;
            }
            i++;
            angle++;
        }
    }
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

void Robot::writeLaser() {
    /* write data to file */
    /* file format: robotPosition[x] robotPosition[y] robotPosition[z] robotLastPosition[x] robotLastPosition[y] robotLastPosition[z]
     *              encoder[0] encoder[1] lastEncoder[0] lastEncoder[1] */
    if (LOG) {
        FILE *data =  fopen("laser.txt", "at");
        if (data!=NULL)
        {
            if (data!=NULL)
            {
                for (int i=0; i<180; ++i)
                    fprintf(data, "%.2f\t",laserReadings[i]);
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

bool Robot::obstacleCheck(){
    float minDist = 999;
    int indexMin = -1;
    float minLeft = 999;
    float minRight = 999;

    for (int i=0; i<8; ++i){
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
        return true;
    }

    return false;
}

void Robot::srt(){
    float v, w;
    float alfa, beta, p, dx, dy;
    float kRho = 50, kAlfa = 50, kBeta= -5;
    if(srtState == Q_NOT_FOUND){
        v = 0;
        if(indexFindingQ > MAX_I){
            float lastX = currentNode->x;
            float lastY = currentNode->y;
            currentNode = getParent(tree);
            std::cout << "GOTO PARENT(" << currentNode->x << ", " << currentNode->y << ")" << std::endl;

            srtState = Q_FOUND;
            srtRobotState = GO_TO_Q;
            indexFindingQ = 0;

            if(lastX == currentNode->x && lastY == currentNode->y){
                robotState = EXPLORATION_DONE;
            }
        }

        if(srtRobotState == FINDING_THETA){
             w = -MAX_ANGULAR_VELOCITY;
            if(rand()%10 >= 5 ){
                theta = robotOrientation[2];
                srtRobotState = GO_TO_THETA;
                indexFindingQ++;
            }
        }

        if(srtRobotState == GO_TO_THETA){
             //check robotOrientation
            float delta = robotOrientation[2] - theta;
             if(delta*delta < MARGIN_THETA ){
//                w = 0;
                srtRobotState = CHECK_Q;
             }else{
                 if(robotTurning == TURNING_UNDEFINED){
                    if(delta > 0){
                        robotTurning = TURNING_LEFT;
                    }else{
                        robotTurning = TURNING_RIGHT;
                    }
                 }
                 if(robotTurning == TURNING_RIGHT){
                    w = -MAX_ANGULAR_VELOCITY;
                 }
                 if(robotTurning == TURNING_LEFT){
                    w = MAX_ANGULAR_VELOCITY;
                 }
             }
        }

        if(srtRobotState == CHECK_Q){
            float minLaserReading = 999;
            float minAngle = 0;
            float dist = 999;
            for (int j = 88 ; j < 92; j++){
                dist = laserReadings[j];
                if(dist > MAX_LASER_READING){
                    dist = MAX_LASER_READING;
                }
                if(dist > 0 && dist < minLaserReading){
                    minLaserReading = dist;
                    minAngle = j;
                }
            }

            if(minLaserReading == 999){
                srtRobotState = FINDING_THETA;
            }else{
                //CHECK Q
                float q,x,y,r;
                r = minLaserReading*ALPHA;

                x = robotPosition[0] + (r) * cos(robotOrientation[2] + ((minAngle - 90)*M_PI)/180);
                y = robotPosition[1] + (r) * sin(robotOrientation[2] + ((minAngle - 90)*M_PI)/180);

                if(r < DMIN || isVisited(tree,x,y)){
                    srtRobotState = FINDING_THETA;
                }else{
                    srtState = Q_FOUND;
                    srtRobotState = GO_TO_Q;
                    indexFindingQ = 0;

                    points coordinates;
                    for(int i = 0; i < 180; i++){
                        coordinates[i].x = x;
                        coordinates[i].y = y;
                    }

                    std::cout << "NEW NODE FOUND: (" << x << "," << y << ")" << std::endl;
                    //new node
                    if(currentNode->child == NULL){
                        currentNode = addChild(currentNode, x,y, coordinates);
                    }else{
                        currentNode = addSibling(currentNode, x,y,coordinates);
                    }
                    FILE *data =  fopen("nodes.txt", "at");
                    if (data!=NULL)
                    {
                        fprintf(data, "%.2f\t",x);
                        fprintf(data, "%.2f\t",y);
                        fprintf(data, "\n");
                        fflush(data);
                        fclose(data);
                    }
                }
            }
        }

    }

    if(srtState == Q_FOUND){

        if(srtRobotState == GO_TO_Q){
            dx = currentNode->x - robotPosition[0];
            dy = currentNode->y - robotPosition[1];
            p = dx*dx + dy*dy;
            alfa = atan2(dy,dx) - robotOrientation[2] ;
            beta = - robotOrientation[2] - alfa;
            v = kRho*p;
            if(v>MAX_LINEAR_VELOCITY)
            {
                v = MAX_LINEAR_VELOCITY;
            }
            w = kAlfa*alfa + kBeta*beta;

            float delta = atan2(dy,dx)*atan2(dy,dx) - robotOrientation[2]*robotOrientation[2];

            if(alfa*alfa > 0.1){
                v = 0;
            }

            //Q_ARRIVED
            if(p < LIMIAR){
                std::cout << "Q ARRIVED" << std::endl;
                srtState = Q_NOT_FOUND;
                srtRobotState = FINDING_THETA;

                //update node
                for(int i = 0; i < 180; i++){
                    float dist = laserReadings[i];
                    if(dist <= 0){
                        dist = MAX_LASER_READING;
                    }
                    float xPoint = robotPosition[0] + (dist) * cos(robotOrientation[2] + ((i - 90)*M_PI)/180);
                    float yPoint = robotPosition[1] + (dist) * sin(robotOrientation[2] + ((i - 90)*M_PI)/180);

                    currentNode->coordinates[i].x = xPoint;
                    currentNode->coordinates[i].y = yPoint;
                }
                writePointsPerLaser();
            }

        }
    }
    if(obstacleCheck() && srtRobotState == GO_TO_Q){
        voidObstacle();
    }else{
        drive(v,w);
    }
}

node * Robot::newNode(float x, float y,points coordinates)
{
    node *new_node = (node*)malloc(sizeof(node));

    if ( new_node ) {
        new_node->next = NULL;
        new_node->child = NULL;
        new_node->x = x;
        new_node->y = y;

        for (int i = 0; i < 180;i++){
            new_node->coordinates[i].x = coordinates[i].x;
            new_node->coordinates[i].y = coordinates[i].y;
        }
    }

    return new_node;
}

node * Robot::addSibling(node * n, float x, float y, points coordinates)
{
    if ( n == NULL )
        return NULL;

    while (n->next)
        n = n->next;

    return (n->next = newNode(x,y,coordinates));
}

node * Robot::addChild(node * n, float x, float y,points coordinates)
{
    if ( n == NULL )
        return NULL;

    if ( n->child )
        return addSibling(n->child, x,y,coordinates);
    else
        return (n->child = newNode(x,y,coordinates));
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

    bool check = false;
    if ( tree == NULL ){
        return false;
    } else {
        float min = 999, delta = 999, minX = 0, minY = 0;
        for (int j = 0; j < 180 ; j++){
            delta = distance(tree->coordinates[j].x,tree->coordinates[j].y,x,y);
            if( delta < min ){
                min = delta;
                minX = tree->coordinates[j].x;
                minY = tree->coordinates[j].y;
            }
        }
        float distToNode = distance(tree->x,tree->y,minX,minY);
        float distToPoint = distance(tree->x,tree->y,x,y);

        if(distToNode > distToPoint && currentNode->x != tree->x && currentNode->y != tree->y){
            check = true;
        }

        if (tree->child != NULL && check == false){
            check = isVisited(tree->child,x,y);
        }

        if (check == false)
        {
            while (tree->next != NULL){

              check = isVisited(tree->next,x,y);

              if (check == false)
              {
                tree = tree->next;
              } else {
                  return check;
              }
            }
        }

        return check;
    }
    return check;
}

node * Robot::getParent(node *tree){
    if ( tree == NULL ){
        return NULL;
    } else {
        node * parent = NULL;

        if (tree->child != NULL){
            if (currentNode->x == tree->child->x &&
                currentNode->y == tree->child->y){
                return tree;
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

void Robot::writePointsPerLaser() {
  float x, y;
  if (LOG) {
    FILE *data =  fopen("points_laser.txt", "at");

    if (data!=NULL){
      // Somente 1 sonar por enquanto, para testes
      for (int i=0; i<180; ++i){
        if(laserReadings[i] > 0){
          x = robotPosition[0] + (laserReadings[i]+R) * cos(robotOrientation[2] + ((i - 90)*M_PI)/180);
          y = robotPosition[1] + (laserReadings[i]+R) * sin(robotOrientation[2] + ((i - 90)*M_PI)/180);
          fprintf(data, "%.4f \t %.4f \n", x, y);
        }
      }
      fflush(data);
      fclose(data);
    }
  }
}

double Robot::distance(double dX0, double dY0, double dX1, double dY1)
{
    return sqrt((dX1 - dX0)*(dX1 - dX0) + (dY1 - dY0)*(dY1 - dY0));
}
