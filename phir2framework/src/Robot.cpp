#include "Robot.h"

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

    // variables used for navigation
    motionMode_ = MANUAL_SIMPLE;

    // variables used for visualization
    viewMode=1;
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
    controlTimer.waitTime(0.1);

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

