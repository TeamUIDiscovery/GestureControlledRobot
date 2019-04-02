#include "sampleListener.h"
#include <cmath>
#include <ctime>

#define MAX_X 200
#define MIN_X -200
#define MAX_Y 350
#define MIN_Y 120
#define MAX_Z 350
#define MIN_Z 100

#define MICRO_SEC 1000000.0

#define MAX_GRIP 100
#define MIN_GRIP 0

#define INITX 0
#define INITY 240
#define INITZ 210

FILE *arduino;

time_t startT;
uint64_t prevTimeFrame;

int first = 1;
extern int change;

float timeDiff = 0;

int currX = 0;
int currY = 240;
int currZ = 210;
int currGrip = 0;

int difX = 0;
int difY = 0;
int difZ = 0;

int start = 1;

double getDistance(int xval, int yval, int zval){
    double dis = 0.0;
    double val = 0.0;
    val = pow(xval-currX,2) + pow(yval-currY,2) + pow(zval-currZ,2);
    dis = pow(val,0.5);
    return dis;
}

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
    if(first){
        time(&startT);
        prevTimeFrame = frame.timestamp();
        first = 0;
    }

    int xPos, yPos, zPos;
    int dis = 0;

    /*std::cout << frame.currentFramesPerSecond() << std::endl;
    std::cout << "Frame id: " << frame.id()
          << ", timestamp: " << frame.timestamp() << std::endl;*/
    hands = frame.hands();
    fingers = hands.rightmost().fingers();
    /*std::cout << "Frame id: " << frame.id()
          << ", timestamp: " << frame.timestamp()
          << ", hands: " << frame.hands().count()
          << ", fingers: " << frame.fingers().count()
          << ", gestures: " << frame.gestures().count() << std::endl;*/

	/*for (HandList::const_iterator hl = hands.begin(); hl != hands.end(); ++hl) {*/

    const Vector handsTranslation = hands.rightmost().palmPosition();
    /*std::cout << "\t" << "X position: " << handsTranslation.x << "\t"
    		<< "\t" << "Y position: " << handsTranslation.y << "\t"
    		<< "\t" << "Z position: " << handsTranslation.z << std::endl;*/

    std::string numStr = "";
    std::string xStr = "";
    std::string yStr = "";
    std::string zStr = "";
    std::string gripStr = "";

    if(hands.begin() != hands.end()){
        xPos = int(handsTranslation.x);
        yPos = (int(handsTranslation.z) - 240 ) * -1;
        zPos = int(handsTranslation.y);
    }
    else{
        xPos = currX;
        yPos = currY;
        zPos = currZ;
    }

    xPos = fmin(fmax(xPos,MIN_X), MAX_X);
    yPos = fmin(fmax(yPos,MIN_Y), MAX_Y);
    zPos = fmin(fmax(zPos,MIN_Z), MAX_Z);

    Vector indexPosition, thumbPosition;
    FingerList::const_iterator fl = fingers.begin();
    const Finger thumb = *fl; // thumb 
    thumbPosition = thumb.tipPosition();
    ++fl; // index finger
    const Finger index = *fl;
    indexPosition = index.tipPosition();

    int gripDistance = fmax(fmin(MAX_GRIP, abs(thumbPosition.x - indexPosition.x)),MIN_GRIP);
    int diffGrip = abs(currGrip - gripDistance);

    dis = int(getDistance(xPos,yPos,zPos));


    xStr = std::to_string(xPos);
    yStr = std::to_string(yPos);
    zStr = std::to_string(zPos);
    gripStr = std::to_string(gripDistance);

    numStr += xStr;
    numStr += ',';
    numStr += yStr;
    numStr += ',';
    numStr += zStr;
    numStr += ',';
    numStr += gripStr;
    numStr += '\n';
    //std::cout << numStr;

    time_t currT;
    time(&currT);

    double sec = difftime(currT,startT);
    double frameTimeDiff = (frame.timestamp() - prevTimeFrame) / MICRO_SEC;
    //std::cout << "time diff : " << frameTimeDiff << std::endl;

    if(sec >= 8 && sec <= 9 && change){
        change = 0;
        currX = INITX;
        currY = INITY;
        currZ = INITZ;
        std::cout << "*********************************back to home position\n";
        time(&startT);
        prevTimeFrame = frame.timestamp();
        //begin_time = clock();
    }

    if(((dis > 6 && dis < 15) || (diffGrip > 1 && diffGrip < 15) ) && frameTimeDiff > 0.06 ){//|| (diffGrip > 5 && diffGrip < 50) ){

        change = 1;
        currX = xPos;
        currY = yPos;
        currZ = zPos;
        currGrip = gripDistance;

        //begin_time = clock();
        fprintf(arduino, "%i,%i,%i,%i\n", xPos,yPos,zPos,gripDistance);
        std::cout << "time diff : " << frameTimeDiff << std::endl;
        time(&startT);
        prevTimeFrame = frame.timestamp();
        std::cout << numStr;
        
    }

    numStr = "";

    //std::cout << gripDistance << std::endl;

    //std::cout << indexPosition.x << "\t" << indexPosition.y << "\t" << indexPosition.z << "\n";


    /*for (FingerList::const_iterator fl = fingers.begin(); fl != fingers.end(); ++fl) {
    	const Finger finger = *fl;
    	fingerPosition = finger.tipPosition();
    	std::cout <<  fingerNames[finger.type()] << "\t:\t" << fingerPosition.x << std::string(4, '\t')
    			<< fingerPosition.y << std::string(4, '\t') << fingerPosition.z << std::endl << std::endl;



    }*/

}