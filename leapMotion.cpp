
#include "listener.h"

int changed = 1;

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
  /***************************************************************
    	-------------------
     	If you are using Mac and want to list usbSerial devices, 
     	use Terminal command "ls /dev/tty.*". Make sure you choose 
     	the same usbSerial name in ArduinoIDE.
     	-------------------
		If you are using Windows, do differently
		-------------------
	***************************************************************/

	SampleListener listener(&fd);
	// create an instance of Controller class
	// add listener object to it.
	Controller controller(listener);

  // Keep this process running until Enter is pressed
  std::cout << "Press Enter to quit..." << std::endl;
  std::cin.get();

  controller.removeListener(listener);

  return 0;
}
