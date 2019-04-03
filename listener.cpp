#include "listener.h"
#include <cmath>
#include <ctime>

FILE *arduino;

//time_t startT;
uint64_t prevTimeFrame;

extern int changed;

//float timeDiff = 0;

int currX = 0;
int currY = 240;
int currZ = 210;
int currGrip = 0;

int activateArm = 0;
int startTime = 1;
int isHome = 0;

// distance in 3d plain 
double getDistance(int,int,int);

void SampleListener::onConnect(const Controller& controller) {
    std::cout << "Connected" << std::endl;

    arduino = fopen("/dev/tty.usbserial-AI06JFP3", "w");
    if(arduino == NULL){
        printf("Serial not opened\n");
        exit(1);
  }
}

void SampleListener::onFrame(const Controller& controller) {

    const Frame frame = controller.frame();

    int xPos, yPos, zPos;
    int dis = 0;

    hands = frame.hands(); // list of hand objects
    fingers = hands.rightmost().fingers(); // list of finger objects

    // only look at the rightmost hand
    const Vector handsTranslation = hands.rightmost().palmPosition();

    // if there is a hand being tracked, save the new X,Y,Z value
    if(hands.begin() != hands.end()){
        /* filtering */
        xPos = int(handsTranslation.x) * NEWR + currX * PREVR;
        yPos = (int(handsTranslation.z) - 240 ) * -1 * NEWR + currY * PREVR;
        zPos = int(handsTranslation.y) * NEWR + currZ * PREVR;
    }// if not, keep the current values
    else{
        xPos = currX;
        yPos = currY;
        zPos = currZ;
        activateArm = 0;
    }

    // validate X,Y,Z against ranges
    xPos = fmin(fmax(xPos,MIN_X), MAX_X);
    yPos = fmin(fmax(yPos,MIN_Y), MAX_Y);
    zPos = fmin(fmax(zPos,MIN_Z), MAX_Z);

    dis = int(getDistance(xPos,yPos,zPos));

    // save the timestamp at the very first frame
    if(startTime){
        prevTimeFrame = frame.timestamp();
        startTime = 0;
    }

    if(activateArm == 0)
        if(dis > 6 && dis < 15)
            activateArm = 1;

    // get index and thumb data
    Vector indexPosition, thumbPosition;

    FingerList::const_iterator fl = fingers.begin();
    const Finger thumb = *fl; 
    thumbPosition = thumb.tipPosition();

    ++fl;
    const Finger index = *fl;
    indexPosition = index.tipPosition();

    // get distance between index and thumb
    // validate it against the range
    int gripDistance = fmax(fmin(MAX_GRIP, abs(thumbPosition.x - indexPosition.x)),MIN_GRIP);
    //int diffGrip = abs(currGrip - gripDistance);

    double frameTimeDiff = (frame.timestamp() - prevTimeFrame) / MICRO_SEC;

    if(frameTimeDiff > 7 && (activateArm == 0) && (isHome == 0)){
        isHome = 1;
        changed = 0;

        currX = INITX;
        currY = INITY;
        currZ = INITZ;
        std::cout << "< Move Arm Back To Home Position >\n";

        // save current time stamp
        prevTimeFrame = frame.timestamp();
    }

    if(frameTimeDiff > 0.08 && activateArm){// && ((dis > 6 && dis < 15) || (diffGrip > 1 && diffGrip < 15 ))){
        isHome = 0;
        changed = 1;
        currX = xPos;
        currY = yPos;
        currZ = zPos;
        currGrip = gripDistance;

        fprintf(arduino, "%i,%i,%i,%i\n", xPos,yPos,zPos,gripDistance);
        printf("%i,%i,%i,%i\n", xPos,yPos,zPos,gripDistance);
        //std::cout << "time diff : " << frameTimeDiff << std::endl;

        // save current time stamp
        prevTimeFrame = frame.timestamp();
        
    }
}
double getDistance(int xval, int yval, int zval){
    double dis = 0.0;
    double val = 0.0;
    val = pow(xval-currX,2) + pow(yval-currY,2) + pow(zval-currZ,2);
    dis = pow(val,0.5);
    return dis;
}