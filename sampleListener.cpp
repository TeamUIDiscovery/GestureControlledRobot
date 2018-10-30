#include "sampleListener.h"

void SampleListener::onConnect(const Controller& controller) {
    std::cout << "Connected" << std::endl;
}

void SampleListener::onFrame(const Controller& controller) {
    //std::cout << "Frame available" << std::endl;
    const Frame frame = controller.frame();
    hands = frame.hands();
    fingers = hands.rightmost().fingers();
    /*std::cout << "Frame id: " << frame.id()
          << ", timestamp: " << frame.timestamp()
          << ", hands: " << frame.hands().count()
          << ", fingers: " << frame.fingers().count()
          << ", gestures: " << frame.gestures().count() << std::endl;*/

	/*for (HandList::const_iterator hl = hands.begin(); hl != hands.end(); ++hl) {*/

    const Vector handsTranslation = hands.rightmost().palmPosition();
    std::cout << "\t" << "X position: " << handsTranslation.x << "\t"
    		<< "\t" << "Y position: " << handsTranslation.y << "\t"
    		<< "\t" << "Z position: " << handsTranslation.z << std::endl;

    pos = int(handsTranslation.x);
    if(pos > 0)
        write(*FD,&pos,sizeof(pos));

    Vector fingerPosition;
    for (FingerList::const_iterator fl = fingers.begin(); fl != fingers.end(); ++fl) {
    	const Finger finger = *fl;
    	fingerPosition = finger.tipPosition();
    	std::cout <<  fingerNames[finger.type()] << "\t:\t" << fingerPosition.x << std::string(4, '\t')
    			<< fingerPosition.y << std::string(4, '\t') << fingerPosition.z << std::endl << std::endl;



    }

}