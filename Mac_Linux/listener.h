#include "Leap.h"
#include <cstring>
#include <stdio.h> 
#include <unistd.h> 
#include <fcntl.h>  
#include <termios.h> 
#include <string>
#include <iostream>
#include <cmath>
#include <ctime>

/* Ranges of X, Y, Z and Grip*/
#define MAX_X 200
#define MIN_X -200
#define MAX_Y 350
#define MIN_Y 120
#define MAX_Z 350
#define MIN_Z 100
#define MAX_GRIP 100
#define MIN_GRIP 20

/* Initial X,Y and Z */
#define INITX 0
#define INITY 240
#define INITZ 210

/* Filtering ratio */
#define NEWR 0.5
#define CURR 0.5

#define MICRO_SEC 1000000.0

using namespace Leap;

const std::string fingerNames[] = {"Thumb", "Index", "Middle", "Ring", "Pinky"};

class SampleListener : public Listener {
public:
   	int *FD;
   	HandList hands;
   	FingerList fingers;
    SampleListener(int *f) { FD = f; }
    virtual void onConnect(const Controller&);
    virtual void onFrame(const Controller&);
};