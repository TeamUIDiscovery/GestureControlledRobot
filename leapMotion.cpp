
#include "sampleListener.h"

//volatile int STOP=FALSE;
volatile int STOP=0;
extern clock_t begin_time;

int change = 1;

unsigned char buf[255]; 

int res;
int myCount=0;
int maxCount=10000;            // Number of cycles to time out serial port 

void init_port(int *fd, unsigned int baud)
{
   struct termios options;
   tcgetattr(*fd,&options); // returns 0 if successful, -1 if not
   switch(baud)
   {
           case 9600: cfsetispeed(&options,B9600);
                 cfsetospeed(&options,B9600);
                 break;
           case 19200: cfsetispeed(&options,B19200);
                 cfsetospeed(&options,B19200);
                 break;
           case 38400: cfsetispeed(&options,B38400);
                 cfsetospeed(&options,B38400);
                 break;
           default:cfsetispeed(&options,B9600);
                 cfsetospeed(&options,B9600);
                 break;
   }
   options.c_cflag |= (CLOCAL | CREAD);
   options.c_cflag &= ~PARENB;
   options.c_cflag &= ~CSTOPB;
   options.c_cflag &= ~CSIZE;
   options.c_cflag |= CS8;
   tcsetattr(*fd,TCSANOW,&options);
}

using namespace Leap;

int main(int argc, char** argv) {
	int fd;
    //int cmd = 1;
    /***************************************************************
    	-------------------
     	If you are using Mac and want to list usbSerial devices, 
     	use Terminal command "ls /dev/tty.*". Make sure you choose 
     	the same usbSerial name in ArduinoIDE.
     	-------------------
		If you are using Windows, do differently
		-------------------
	***************************************************************/
    //fd = open("/dev/tty.usbserial-AI06JFP3", O_RDWR | O_NOCTTY | O_NDELAY); 
     
   	/*if(fd == -1) { // Check for port errors
           std::cout << fd;
           perror("Unable to open serial port\n");
           return (0);
    }*/
    //init_port(&fd,9600);
    //init_port(&fd,38400);
    //write(fd,&cmd,sizeof(cmd));
	// create an instance of listener
	SampleListener listener(&fd);
	// create an instance of Controller class
	// add listener object to it.
	Controller controller(listener);


	/*if(controller.isConnected() == false){
		std::cout << "Leap Motion is NOT connected\n";
		exit(-1);
	}*/

    // Keep this process running until Enter is pressed
    std::cout << "Press Enter to quit..." << std::endl;
    std::cin.get();

    controller.removeListener(listener);

    return 0;
}
