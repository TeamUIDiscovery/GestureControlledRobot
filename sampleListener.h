#include "Leap.h"
#include <cstring>
#include <stdio.h> 
#include <unistd.h> 
#include <fcntl.h>  
#include <termios.h> 
#include <string>
#include <iostream>

using namespace Leap;

const std::string fingerNames[] = {"Thumb", "Index", "Middle", "Ring", "Pinky"};

class SampleListener : public Listener {
//private:
	//Frame frame;


public:
   	int *FD;
   	HandList hands;
   	FingerList fingers;
    SampleListener(int *f) { FD = f; }
    virtual void onConnect(const Controller&);
    virtual void onFrame(const Controller&);
};