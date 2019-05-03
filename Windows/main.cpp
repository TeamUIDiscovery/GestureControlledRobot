
#include "listener.h"

int changed = 1;
extern HANDLE hCom;
extern BOOL bErrorFlag;

using namespace Leap;

int main(int argc, char** argv) {

  int fd;
  char c = 'd';

  /* open serial port communication with Arduino board (WRITE ONLY) */
  hCom = CreateFile("COM3", // port name
    GENERIC_WRITE,          // open for writing
    0,                // do not share
    NULL,             // default security 
    OPEN_EXISTING,          // open exisitng file
    0,
    NULL              // no attr. template
  );
  if (hCom == INVALID_HANDLE_VALUE) {
    printf("Serial COM3 not opened\n");
    exit(1);
  }

  else {
    std::string cmd = "f";
    while (cmd != "1") {
      std::cout << "Enter 1 to start :   " << std::flush;
      std::cin >> cmd;
      std::cin.ignore();
    }
    bErrorFlag = WriteFile(
      hCom,           // open file handle
      cmd.c_str(),      // start of data to write
      strlen(cmd.c_str()),  // number of bytes to write
      NULL, // number of bytes that were written
      NULL);            // no overlapped structure

    if (FALSE == bErrorFlag)
      printf("Terminal failure: Unable to write to file.\n");

  }


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
