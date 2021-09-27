#include "Robot.h"

#include <unistd.h>
#include <GL/glut.h>
#include <cmath>
#include <iostream>
#include <algorithm>

//////////////////////////////////////
///// CONSTRUCTORS & DESTRUCTORS /////
//////////////////////////////////////

Robot::Robot()
{
    ready_ = false;
    running_ = true;
    
    PID_PreviousCTE_ = 0.0;
    PID_CTE_ = 0.0;
    PID_CTESum_ = 0.0;

    grid = new Grid();

    plan = new Planning();
    plan->setGrid(grid);
    plan->setMaxUpdateRange(base.getMaxLaserRange());

    // variables used for navigation
    motionMode_=MANUAL_SIMPLE;

    // variables used for visualization
    viewMode=0;
    numViewModes=5;


}

Robot::~Robot()
{
    base.closeARIAConnection();
    if(grid!=NULL)
        delete grid;
}

////////////////////////////////////
///// INITIALIZE & RUN METHODS /////
////////////////////////////////////

void Robot::initialize(ConnectionMode cmode, LogMode lmode, std::string fname)
{
    logMode_ = lmode;
//    logFile_ = new LogFile(logMode_,fname);
    ready_ = true;

    // initialize ARIA
    if(logMode_!=PLAYBACK){
        bool success = base.initialize(cmode,lmode,fname);
        if(!success){
            printf("Could not connect to robot... exiting\n");
            exit(0);
        }
    }

    ready_ = true;
    controlTimer.startLap();
}

void Robot::run()
{
    controlTimer.waitTime(0.2);

    if(logMode_==PLAYBACK){
        bool hasEnded = base.readFromLog();
        if(hasEnded){
            std::cout << "PROCESS COMPLETE. CLOSING PROGRAM." << std::endl;
            exit(0);
        }
    }else{
        bool success = base.readOdometryAndSensors();
        if(!success){
            usleep(50000);
            return;
        }

        if(logMode_==RECORDING)
            base.writeOnLog();
    }

    currentPose_ = base.getOdometry();

    pthread_mutex_lock(grid->mutex);

    // Mapping
    mappingWithHIMMUsingLaser();
    mappingWithLogOddsUsingLaser();
    mappingUsingSonar();

    pthread_mutex_unlock(grid->mutex);

    plan->setNewRobotPose(currentPose_);

    // Save path traversed by the robot
    if(base.isMoving() || logMode_==PLAYBACK){
        path_.push_back(base.getOdometry());
    }

    // Navigation
    switch(motionMode_){
        case WANDER:
            wanderAvoidingCollisions();
            break;
        case FARFROMWALLS:
            keepAsFarthestAsPossibleFromWalls();
            break;
        case ENDING:
            running_=false;
            break;
        default:
            break;
    }

    base.resumeMovement();

    usleep(50000);
}

//////////////////////////////
///// NAVIGATION METHODS /////
//////////////////////////////

void Robot::move(MovingDirection dir)
{
    switch(dir){
        case FRONT:
            std::cout << "moving front" << std::endl;
            break;
        case BACK:
            std::cout << "moving back" << std::endl;
            break;
        case LEFT:
            std::cout << "turning left" << std::endl;
            break;
        case RIGHT:
            std::cout << "turning right" << std::endl;
            break;
        case STOP:
            std::cout << "stopping robot" << std::endl;
    }

    if(motionMode_==MANUAL_SIMPLE)
        base.setMovementSimple(dir);
    else if(motionMode_==MANUAL_VEL)
        base.setMovementVel(dir);
}

void Robot::wanderAvoidingCollisions()
{
    float laserLeft  = base.getMinLaserValueInRange(0,74);
    float laserFront = base.getMinLaserValueInRange(75,105);
    float laserRight = base.getMinLaserValueInRange(106,180);

    float linVel = 1.0;
    float angVel = 0.0;

    // Ditance to be considered as blocked
    float laserThresholdWallDistance = 1.2;

    base.setWheelsVelocity_fromLinAngVelocity(linVel, angVel);

    bool isUnblocked, isLeftFrontBlocked, isRightFrontBlocked, isLeftBlocked, isRightBlocked;

    isLeftBlocked = (laserLeft <= laserThresholdWallDistance);

    isRightBlocked = (laserRight <= laserThresholdWallDistance);

    isLeftFrontBlocked = isLeftBlocked && (laserFront <= laserThresholdWallDistance);

    isRightFrontBlocked = isRightBlocked && (laserFront <= laserThresholdWallDistance);

    isUnblocked = (laserFront > laserThresholdWallDistance);

    std::cout << "LOG:laserLeft: " << laserLeft << std::endl;
    std::cout << "LOG:laserFront: " << laserFront << std::endl;
    std::cout << "LOG:laserRight: " << laserRight << std::endl;

    if( isUnblocked )
    {
        std::cout << "No obstacles. Moving forward." << std::endl;
        base.setMovementSimple(FRONT);
    }
    else if( isRightFrontBlocked || isRightBlocked )
    {
        std::cout << "Right/Right-front blocked. Turning left." << std::endl;
        base.setMovementSimple(LEFT);
    }
    else if( isLeftFrontBlocked || isLeftBlocked )
    {
        std::cout << "Left/Left-front blocked. Turning right." << std::endl;
        base.setMovementSimple(RIGHT);
    }
}

void Robot::keepAsFarthestAsPossibleFromWalls()
{
    float linVel = 0.5;
    float angVel = 0.0;

    float laserThresholdAngleDeg = 60;
    float laserThresholdReach = 1.75;
    
    // Hyperparameters for the PID controller 
    double Tp = 3, Td = 20, Ti = 0.0005;

    double p, i, d = 0.0;

    // Limits the range of the left and right laser
    float laserLeft = base.getMinLaserValueInRange(0, laserThresholdAngleDeg - 1);
    float laserRight = base.getMinLaserValueInRange(181 - laserThresholdAngleDeg, 180);

    // Limits the range for both lasers so that the perturbation when meeting long "empty"
    // spaces does not affect so much the resulting angular velocity
    if ( laserLeft > laserThresholdReach )
    {
        laserLeft = laserThresholdReach;
    }

    if ( laserRight > laserThresholdReach )
    {
        laserRight = laserThresholdReach;
    }

    PID_CTE_ = laserLeft - laserRight;
    PID_CTESum_ += PID_CTE_;

    // Proportional term
    p = Tp * PID_CTE_;

    // Integral term
    i = Ti * PID_CTESum_;

    // Derivative term
    d = Td * (PID_CTE_ - PID_PreviousCTE_);

    angVel = p + i + d;

    std::cout << "LOG:angVel: " << angVel << std::endl;
    std::cout << "LOG:laserLeft: " << laserLeft << std::endl;
    std::cout << "LOG:laserRight: " << laserRight << std::endl;
    std::cout << "LOG:PID_CTE_: " << PID_CTE_ << std::endl;
    std::cout << "LOG:PID_CTESum_: " << PID_CTESum_ << std::endl;
    std::cout << "LOG:PID_PreviousCTE_: " << PID_PreviousCTE_ << std::endl;

    PID_PreviousCTE_ = PID_CTE_;

    base.setWheelsVelocity_fromLinAngVelocity(linVel, angVel);
}

///////////////////////////
///// MAPPING METHODS /////
///////////////////////////

float Robot::getOccupancyFromLogOdds(float logodds)
{
    return 1.0 - 1.0/(1.0+exp(logodds));
}

float Robot::getLogOddsFromOccupancy(float occupancy)
{
    return log(occupancy/(1.0-occupancy));
}

void Robot::mappingWithLogOddsUsingLaser()
{
    float lambdaR = 0.1;  // 10 cm
    float lambdaPhi = 1.0;  // 1 degree

    int scale = grid->getMapScale();
    float maxRange = base.getMaxLaserRange();
    int maxRangeInt = maxRange*scale;

    int robotX = currentPose_.x*scale;
    int robotY = currentPose_.y*scale;
    float robotAngle = currentPose_.theta;
    int maxX, maxY, minX, minY = 0;

    float locc, lfree;
    float pocc, pfree;

    // How to access a grid cell:
    //    Cell* c=grid->getCell(robotX,robotY);

    // Access log-odds value of variable in c->logodds
    
    // How to convert logodds to occupancy values:
    //    c->occupancy = getOccupancyFromLogOdds(c->logodds);

    // Defines fixed values of occupancy:
    //    0.0 < pfree < 0.5 < pocc < 1.0
    pfree = 0.3;
    pocc = 0.8;
        
    // To log odds
    locc = log( pocc / (1-pocc) );
    lfree = log( pfree / (1-pfree) );

    // Update cells in the sensors' field-of-view
    // You only need to check the cells at most maxRangeInt from the robot position
    // that is, in the following square region:
    //
    //  (robotX-maxRangeInt,robotY+maxRangeInt)  -------  (robotX+maxRangeInt,robotY+maxRangeInt)
    //                     |                       \                         |
    //                     |                        \                        |
    //                     |                         \                       |
    //  (robotX-maxRangeInt,robotY-maxRangeInt)  -------  (robotX+maxRangeInt,robotY-maxRangeInt)
    minX = robotX - maxRangeInt;
    minY = robotY - maxRangeInt;
    maxX = robotX + maxRangeInt;
    maxY = robotY + maxRangeInt;

    for(int i = minX; i <= maxX; i++)
    {
        for(int j = minY; j <= maxY; j++)
        {
            float r, phi, k;

            // Cell to be analyzed
            Cell* c = grid->getCell(i,j);

            // Computes the euclidean distance from the robot to the cell
            r = sqrt( pow(i - robotX, 2) + pow(j - robotY, 2) );
            r = r/scale;  // To meters

            // Computes the phi-orientation from celll relative to the robot's local coordinates
            phi = RAD2DEG(atan2(j - robotY, i - robotX)) - robotAngle;
            phi = normalizeAngleDEG(phi);  // Normalizes the angle (between −180 and 180 degrees)

            k = base.getNearestLaserBeam(phi);

            // Updates cell occupation (occupied or free) depending on the sensor region where its located
            if( (fabs(phi - base.getAngleOfLaserBeam(k)) > lambdaPhi/2 ) || (r > std::min(maxRange, (base.getKthLaserReading(k) + (lambdaR/2)))) )
            {
               c->logodds += 0;
            }
            else if( (base.getKthLaserReading(k) < maxRange) && (fabs(r - base.getKthLaserReading(k))< lambdaR/2) )
            {
               c->logodds += locc;
            }
            else if( r <= base.getKthLaserReading(k) )
            {
               c->logodds += lfree;
            }

            c->occupancy = getOccupancyFromLogOdds(c->logodds);
        }
    }
}

void Robot::mappingUsingSonar()
{
    float lambdaR = 0.5; //  50 cm
    float lambdaPhi = 30.0;  // 30 degrees

    int scale = grid->getMapScale();
    float maxRange = base.getMaxSonarRange();
    int maxRangeInt = maxRange*scale;

    int robotX = currentPose_.x*scale;
    int robotY = currentPose_.y*scale;
    float robotAngle = currentPose_.theta;
    int maxX, maxY, minX, minY = 0;

    float occUpdate, occ;
    float R = maxRange;

    minX = robotX - maxRangeInt;
    minY = robotY - maxRangeInt;
    maxX = robotX + maxRangeInt;
    maxY = robotY + maxRangeInt;

    for(int i = minX; i <= maxX; i++)
    {
        for(int j = minY; j <= maxY; j++)
        {
            float r, phi, k = 0, factor = 0;

            // Cell to be analyzed
            Cell* c = grid->getCell(i,j);

            occ = c->occupancySonar;

            // Computes the euclidean distance from the robot to the cell
            r = sqrt( pow(i - robotX, 2) + pow(j - robotY, 2) );
            r = r/scale;  // To meters

            // Computes the phi-orientation from celll relative to the robot's local coordinates
            phi = RAD2DEG(atan2(j - robotY, i - robotX)) - robotAngle;
            phi = normalizeAngleDEG(phi);  // Normalizes the angle (between −180 and 180 degrees)

            k = base.getNearestSonarBeam(phi);

            // Updates cell occupation (occupied or free) depending on the sensor region where its located
            // Region III
            if( (fabs(phi - base.getAngleOfSonarBeam(k)) > lambdaPhi/2) || (r > std::min(maxRange, (base.getKthSonarReading(k)+(lambdaR/2)))) )
            {
                occ += 0.0;
            }
            // Region I
            else if( (base.getKthSonarReading(k) < maxRange) && (fabs(r - base.getKthSonarReading(k)) < (lambdaR / 2)) )
            {  
                factor = (((R - r) / R) + ((lambdaPhi - lambdaR) / lambdaPhi)) / 2;
                occUpdate = 0.5 * factor + 0.5;
                occ = (occUpdate * occ) / ((occUpdate * occ) + ((1.0 - occUpdate) * (1.0 - occ)));
            }
            // Region II
            else if(r <= base.getKthSonarReading(k))
            {
                factor = (((R - r) / R) + ((lambdaPhi - lambdaR) / lambdaPhi)) / 2;
                occUpdate = 0.5 * (1.0 - factor);
                occ = (occUpdate * occ) / ( (occUpdate * occ) + ((1.0 - occUpdate) * (1.0 - occ)) );
            }

            // Prevents that Occ becomes 0 or 1:
            if(occ == 1)
            {
                occ = 0.99;
            }
            else if(occ == 0)
            {
                occ = 0.01;
            }

            c->occupancySonar = occ;
        }
    }
}

void Robot::mappingWithHIMMUsingLaser()
{
    float lambdaR = 0.2;  // 20 cm
    float lambdaPhi = 1.0;  // 1 degree

    int scale = grid->getMapScale();
    float maxRange = base.getMaxLaserRange();
    int maxRangeInt = maxRange*scale;

    int robotX = currentPose_.x*scale;
    int robotY = currentPose_.y*scale;
    float robotAngle = currentPose_.theta;
    int maxX, maxY, minX, minY = 0;

    // Update cells in the sensors' field-of-view
    // You only need to check the cells at most maxRangeInt from the robot position
    // that is, in the following square region:
    //
    //  (robotX-maxRangeInt,robotY+maxRangeInt)  -------  (robotX+maxRangeInt,robotY+maxRangeInt)
    //                     |                       \                         |
    //                     |                        \                        |
    //                     |                         \                       |
    //  (robotX-maxRangeInt,robotY-maxRangeInt)  -------  (robotX+maxRangeInt,robotY-maxRangeInt)
    minX = robotX - maxRangeInt;
    minY = robotY - maxRangeInt;
    maxX = robotX + maxRangeInt;
    maxY = robotY + maxRangeInt;

    for(int i = minX; i <= maxX; i++)
    {
        for(int j = minY; j <= maxY; j++)
        {
            float r, phi, k = 0, factor = 0;
            
            // Cell to be analyzed
            Cell* c = grid->getCell(i,j);
            
            // Computes the euclidean distance from the robot to the cell
            r = sqrt( pow(i - robotX, 2) + pow(j - robotY, 2) );
            r = r/scale;  // To meters

            // Computes the phi-orientation from celll relative to the robot's local coordinates
            phi = RAD2DEG(atan2(j - robotY, i - robotX)) - robotAngle;
            phi = normalizeAngleDEG(phi);  // Normalizes the angle (between −180 and 180 degrees)

            k = base.getNearestLaserBeam(phi);

            // Updates cell occupation (occupied or free) depending on the sensor region where its located
            if( (r > std::min(maxRange, base.getKthLaserReading(k))) || (fabs(phi - base.getAngleOfLaserBeam(k)) > lambdaPhi / 2) )
            {
                c->himm += 0;
            }
            else if( (base.getKthLaserReading(k) < maxRange) && (fabs(r - base.getKthLaserReading(k)) < (lambdaR / 2)) )
            {
                // When the cell is on an obstacle region from the sensor, increment the counter by 3
                // Limits the counter to be 15 at maximum
                if(c->himm == 15)
                {
                    c->himm += 0;
                }
                else
                {
                    c->himm += 3;
                }
            }
            else if( r <= base.getKthLaserReading(k) )
            {
                // When the cell is NOT on an obstacle region, decrement counter by 1
                // Limits the counter to be 0 at minimum
                if(c->himm == 0)
                {
                    c->himm += 0;
                }
                else
                {
                    c->himm -= 1;
                }
            }
        }
    }
}

/////////////////////////////////////////////////////
////// METHODS FOR READING & WRITING ON LOGFILE /////
/////////////////////////////////////////////////////

// Prints to file the data that we would normally be getting from sensors, such as the laser and the odometry.
// This allows us to later play back the exact run.
void Robot::writeOnLog()
{
    logFile_->writePose("Odometry",currentPose_);
    logFile_->writeSensors("Sonar",base.getSonarReadings());
    logFile_->writeSensors("Laser",base.getLaserReadings());
}

// Reads back into the sensor data structures the raw readings that were stored to file
// While there is still information in the file, it will return 0. When it reaches the end of the file, it will return 1.
bool Robot::readFromLog() {

    if(logFile_->hasEnded())
        return true;

    base.setOdometry(logFile_->readPose("Odometry"));
    base.setSonarReadings(logFile_->readSensors("Sonar"));
    base.setLaserReadings(logFile_->readSensors("Laser"));

    return false;
}

////////////////////////
///// DRAW METHODS /////
////////////////////////

void Robot::draw(float xRobot, float yRobot, float angRobot)
{
    float scale = grid->getMapScale();
    glTranslatef(xRobot,yRobot,0.0);
    glRotatef(angRobot,0.0,0.0,1.0);
    glScalef(1.0/scale,1.0/scale,1.0/scale);

    // sonars and lasers draw in cm
    if(viewMode==1)
        base.drawSonars(true);
    else if(viewMode==2)
        base.drawSonars(false);
    else if(viewMode==3)
        base.drawLasers(true);
    else if(viewMode==4)
        base.drawLasers(false);

    // robot draw in cm
    base.drawBase();

    glScalef(scale,scale,scale);
    glRotatef(-angRobot,0.0,0.0,1.0);
    glTranslatef(-xRobot,-yRobot,0.0);
}

/////////////////////////
///// OTHER METHODS /////
/////////////////////////

bool Robot::isReady()
{
    return ready_;
}

bool Robot::isRunning()
{
    return running_;
}

const Pose& Robot::getCurrentPose()
{
    return currentPose_;
}

void Robot::drawPath()
{
    float scale = grid->getMapScale();

    if(path_.size() > 1){
        glScalef(scale,scale,scale);
        glLineWidth(3);
        glBegin( GL_LINE_STRIP );
        {
            for(unsigned int i=0;i<path_.size()-1; i++){
                glColor3f(1.0,0.0,1.0);

                glVertex2f(path_[i].x, path_[i].y);
                glVertex2f(path_[i+1].x, path_[i+1].y);
            }
        }
        glEnd();
        glLineWidth(1);
        glScalef(1.0/scale,1.0/scale,1.0/scale);

    }
}

void Robot::waitTime(float t){
    float l;
    do{
        usleep(1000);
        l = controlTimer.getLapTime();
    }while(l < t);
    controlTimer.startLap();
}
