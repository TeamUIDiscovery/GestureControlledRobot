#define ARBOTIX_TO  7000
#define MAX_SERVO_DELTA_PERSEC 100

#include <ax12.h>
#include <BioloidController.h>
#include <Commander.h>

//=============================================================================
//=============================================================================
/* Servo IDs */
enum {
  SID_BASE=1, SID_RSHOULDER, SID_LSHOULDER, SID_RELBOW, SID_LELBOW, SID_WRIST, SID_GRIP };//SID_WRISTROT, SID_GRIP};

enum {
  IKM_IK3D_CARTESIAN, IKM_CYLINDRICAL, IKM_BACKHOE};

// status messages for IK return codes..
enum {
  IKS_SUCCESS=0, IKS_WARNING, IKS_ERROR};


#define CNT_SERVOS  7 //(sizeof(pgm_axdIDs)/sizeof(pgm_axdIDs[0]))

// Define some Min and Maxs for IK Movements...
//                y   Z
//                |  /
//                |/
//            ----+----X (X and Y are flat on ground, Z is up in air...
//                |
//                |
#define IK_MAX_X  250
#define IK_MIN_X  -250

#define IK_MAX_Y  350
#define IK_MIN_Y  120

#define IK_MAX_Z  350
#define IK_MIN_Z  80

#define IK_MAX_GA  90
#define IK_MIN_GA   -90

// Define Ranges for the different servos...
#define BASE_MIN    0
#define BASE_MAX    800 //1023

#define SHOULDER_MIN  350
#define SHOULDER_MAX  710 // 810

#define ELBOW_MIN    400 // 210
#define ELBOW_MAX    700 // 900

#define WRIST_MIN    300  // 200
#define WRIST_MAX    530  // 830

#define WROT_MIN     0
#define WROT_MAX     512 // 1023

#define GRIP_MIN     312
#define GRIP_MAX     450

// Define some lengths and offsets used by the arm
#define BaseHeight          110L   // (L0)about 120mm (90mm)
#define ShoulderLength      150L   // (L1)Not sure yet what to do as the servo is not directly in line,  Probably best to offset the angle?
//                                 // X is about 140, y is about 40 so sqrt is Hyp is about 155, so maybe about 21 degrees offset
#define ShoulderServoOffset 72L    // should offset us some...
#define ElbowLength         147L   //(L2)Length of the Arm from Elbow Joint to Wrist Joint
#define WristLength         137L   // (L3)Wrist length including Wrist rotate
#define G_OFFSET            0      // Offset for static side of gripper?

#define IK_FUDGE            5     // How much a fudge between warning and error

//=============================================================================
// Global Objects
//=============================================================================

BioloidController bioloid = BioloidController(1000000);

//=============================================================================
// Global Variables...
//=============================================================================
boolean         g_fArmActive = false;   // Is the arm logically on?
byte            g_bIKMode = IKM_IK3D_CARTESIAN;   // Which mode of operation are we in...
uint8_t         g_bIKStatus = IKS_SUCCESS;   // Status of last call to DoArmIK;
boolean         g_fServosFree = true;

// Current IK values
int            g_sIKGA;                  // IK Gripper angle..
int            g_sIKX;                  // Current X value in mm
int            g_sIKY;                  //
int            g_sIKZ;

// Values for current servo values for the different joints
int             g_sBase;                // Current Base servo value
int             g_sShoulder;            // Current shoulder target 
int             g_sElbow;               // Current
int             g_sWrist;               // Current Wrist value
int             g_sWristRot;            // Current Wrist rotation
int             g_sGrip;                // Current Grip position

// BUGBUG:: I hate too many globals...
int sBase, sShoulder, sElbow, sWrist, sWristRot, sGrip;

// Message informatino
unsigned long   ulLastMsgTime;          // Keep track of when the last message arrived to see if controller off

// flag : if the arm is in its home position
int isHome = 0;

//
#ifdef DEBUG
boolean        g_fDebugOutput = false;
#endif

//===================================================================================================
// Setup 
//====================================================================================================
void setup() {
  Serial.begin(38400); // 115200
  // Next initialize the Bioloid
  bioloid.poseSize = CNT_SERVOS;

  // Read in the current positions...
  bioloid.readPose();
  // Start off to put arm to sleep...
  //PutArmToSleep();
  Serial.println("Enter 1 to start");
  char cmd = '~';
  while(cmd != '1'){
    if(Serial.available() > 0){
     cmd = Serial.read(); 
    }
  }
  
  Serial.println("Moving the Arm to Home Position");
  //MoveArmToHome();
  delay(100);
  Serial.println("Starting Communication");
  //PutArmToSleep();

  //set Gripper Compliance so it doesn't tear itself apart
  ax12SetRegister(SID_GRIP, AX_CW_COMPLIANCE_SLOPE, 128);
  ax12SetRegister(SID_GRIP, AX_CCW_COMPLIANCE_SLOPE, 128);

}
//===================================================================================================
// loop: Our main Loop!
//===================================================================================================
// Define number of pieces
const int numberOfPieces = 4;
String pieces[numberOfPieces];

// This will be the buffered string from Serial.read() up until \n
// Should look something like "123,456,789"
String input = "";

// Keep track of current position in array
int counter = 0;

// Keep track of the last comma so we know where to start the substring
int lastIndex = 0;
void loop(){
  boolean fChanged = false;
  // Check for data coming in from serial
  
  if (Serial.available() > 0) {
    // Read the first byte and store it as a char
    char ch = Serial.read();
    
    // Do all the processing here since this is the end of a line
    if (ch == '\n' || ch == '\r') {
      for (int i = 0; i < input.length(); i++) {
        // Loop through each character and check if it's a comma
        if (input.substring(i, i+1) == ",") {
          // Grab piece from last index up to the current position and store it
          pieces[counter] = input.substring(lastIndex, i);
          // Update last position and add 1, so it starts from next character
          lastIndex = i + 1;
          // Increase the position in the array that we store into
          counter++;
        }

        // If we're at the end of the string (no more commas to stop us)
        if (i == input.length() - 1) {
          // Grab the last part of the string from the lastIndex to the end
          pieces[counter] = input.substring(lastIndex, i+1);
        }
      }

      // Update the position
      char buffer[10];
      pieces[0].toCharArray(buffer, 10);
      float x = atof(buffer);
      pieces[1].toCharArray(buffer, 10);
      float y = atof(buffer);
      pieces[2].toCharArray(buffer, 10);
      float z = atof(buffer);
      pieces[3].toCharArray(buffer, 10);
      float gripperVal = atof(buffer);
      g_sGrip = min(max(leapToGripper(gripperVal),GRIP_MIN),GRIP_MAX);
      
      // check if they are normal
      // 1-89 -9-87
      
      float ga = map(z, 0, 500, -30, 90);
      //doArmIKLimits(x, y, z, ga);
      Serial.print("( ");      
      Serial.print(pieces[0]);
      Serial.print(", ");
      Serial.print(pieces[1]);
      Serial.print(", ");
      Serial.print(pieces[2]);
      Serial.print(", ");
      Serial.print(g_sGrip);
      Serial.print(" )\n");

      // See if the Arm is active yet...
      if (g_fArmActive == true) {
        isHome = 0;
        
        sBase = g_sBase;
        sShoulder = g_sShoulder;
        sElbow = g_sElbow; 
        sWrist = g_sWrist;
        sGrip = g_sGrip;
        sWristRot = g_sWristRot;
  
        fChanged |= ProcessUserInput3D(x,y,z);
            
        // If something changed and we are not in an error condition
        if (fChanged && (g_bIKStatus != IKS_ERROR)) {
          MoveArmTo(sBase, sShoulder, sElbow, sWrist, sWristRot, sGrip, 50, true);
        }
                  
        else if (bioloid.interpolating > 0) {
          bioloid.interpolateStep();
        }
      }
      else {
        g_fArmActive = true;
        Serial.println("moveHome Called");
        MoveArmToHome(); 
         isHome = 1;     
      }
      ulLastMsgTime = millis();    // remember when we last got a message...

      // Clear out string and counters to get ready for the next incoming string
      input = "";
      counter = 0;
      lastIndex = 0;
    }
    else {
      // If we havent reached newline yet, add current character to string
      input += ch;
    }
  }
  else {
    if (bioloid.interpolating > 0) {
      bioloid.interpolateStep();
    }
    // error see if we exceeded a timeout
    if (g_fArmActive && ((millis() - ulLastMsgTime) > ARBOTIX_TO) && (isHome == 0)) {
      //PutArmToSleep();
      Serial.println("Going back to home position");
      MoveArmToHome();
      isHome = 1;
    }
  }
}
float leapToGripper(float leapVal){
 return -2 * leapVal + 512; 
}
//===================================================================================================
// PutArmToSleep
//===================================================================================================
void PutArmToSleep(void) {
  g_fArmActive = false;
  //MoveArmTo(512, 212, 212, 512, 512, 256, 500, true);

  // And Relax all of the servos...
  for(uint8_t i=1; i <= CNT_SERVOS; i++) {
    Relax(i);
  }
  g_fServosFree = true;
}
//===================================================================================================
// MoveArmToHome
//===================================================================================================
void MoveArmToHome(void) {

  if (g_bIKMode != IKM_BACKHOE) {
    g_bIKStatus = doArmIK(true, 0, (2*ElbowLength)/3+WristLength, BaseHeight+(2*ShoulderLength)/3, 0);
    //g_bIKStatus = doArmIK(true, 0, 30, 400, 0);
    Serial.print("To home position < ");
    Serial.print(sBase);
    Serial.print(", ");
    Serial.print(sShoulder);
    Serial.print(", ");
    Serial.print(sElbow);
    Serial.print(", ");
    Serial.print(sWrist);
    Serial.println(" >");
    MoveArmTo(sBase, sShoulder, sElbow, sWrist, 512, 420, 500, true);
  }
  else {
    Serial.println("else stmt");
    g_bIKStatus = IKS_SUCCESS;  // assume sucess so we will continue to move...
    MoveArmTo(512, 512, 330, 690, 600, 512, 500, true);
  }
}

boolean ProcessUserInput3D(int xPos, int yPos, int zPos) {
  // We Are in IK mode, so figure out what position we would like the Arm to move to.
  // We have the Coordinates system like:
  //
  //                y   Z
  //                |  /
  //                |/
  //            ----+----X (X and Y are flat on ground, Z is up in air...
  //                |
  //                |
  //
  boolean fChanged = false;
  int   sIKX;                  // Current X value in mm
  int   sIKY;                  //
  int   sIKZ;
  int   sIKGA;
  // Limit how far we can go by checking the status of the last move.  If we are in a warning or error
  // condition, don't allow the arm to move farther away...
  if (g_bIKStatus == IKS_SUCCESS) {
    Serial.println("IKS success");
    sIKX = min(max(xPos, IK_MIN_X), IK_MAX_X);
    sIKY = min(max(yPos, IK_MIN_Y), IK_MAX_Y);
    sIKZ = min(max(zPos, IK_MIN_Z), IK_MAX_Z);
    sIKGA = min(max(g_sIKGA, IK_MIN_GA), IK_MAX_GA);
    /*sIKX = min(max(g_sIKX + xPos/100, IK_MIN_X), IK_MAX_X);
    sIKY = min(max(g_sIKX + yPos/100, IK_MIN_Y), IK_MAX_Y);
    sIKZ = min(max(g_sIKX + zPos/100, IK_MIN_Z), IK_MAX_Z);
    sIKGA = min(max(g_sIKGA, IK_MIN_GA), IK_MAX_GA);*/
    }

  else {
    // In an Error/warning condition, only allow things to move in closer...
    Serial.println("IKS not success");
    sIKX = g_sIKX + 1;
    sIKY = g_sIKY;
    sIKZ = g_sIKZ;
    sIKGA = g_sIKGA;

    /*if (((g_sIKX > 0) && (xPos < 0)) || ((g_sIKX < 0) && (xPos > 0)))
      sIKX = min(max(xPos, IK_MIN_X), IK_MAX_X);
    if (((g_sIKY > 0) && (yPos < 0)) || ((g_sIKY < 0) && (yPos > 0)))
      sIKY = min(max(yPos, IK_MIN_Y), IK_MAX_Y);
    if (((g_sIKZ > 0) && (zPos < 0)) || ((g_sIKZ < 0) && (zPos > 0)))
      sIKZ = min(max(zPos, IK_MIN_Z), IK_MAX_Z);
    if (((g_sIKGA > 0) && (zPos < 0)) || ((g_sIKGA < 0) && (zPos > 0))) 
      sIKGA = min(max(g_sIKGA, IK_MIN_GA), IK_MAX_GA);  // Currently in Servo coords... 
    sIKGA = min(max(g_sIKGA, IK_MIN_GA), IK_MAX_GA);  */    
  }
  fChanged = (sIKX != g_sIKX) || (sIKY != g_sIKY) || (sIKZ != g_sIKZ) || (sIKGA != g_sIKGA) ;

  if (fChanged) {
    Serial.println("CHANGED");
    g_bIKStatus = doArmIK(true, sIKX, sIKY, sIKZ, sIKGA);
  }
  return fChanged;
}
//===================================================================================================
// Convert radians to servo position offset. 
//===================================================================================================
int radToServo(float rads){
  float val = (rads*100)/51 * 100;
  return (int) val;
}
//===================================================================================================
// Compute Arm IK for 3DOF+Mirrors+Gripper - was based on code by Michael E. Ferguson
// Hacked up by me, to allow different options...
//===================================================================================================
uint8_t doArmIK(boolean fCartesian, int sIKX, int sIKY, int sIKZ, int sIKGA)
{
  int t;
  int sol0;
  uint8_t bRet = IKS_SUCCESS;  // assume success
#ifdef DEBUG
  if (g_fDebugOutput) {
    Serial.print("(");
    Serial.print(sIKX, DEC);
    Serial.print(",");
    Serial.print(sIKY, DEC);
    Serial.print(",");
    Serial.print(sIKZ, DEC);
    Serial.print(",");
    Serial.print(sIKGA, DEC);
    Serial.print(")=");
  }
#endif
  if (fCartesian) {
    // first, make this a 2DOF problem... by solving base
    sol0 = radToServo(atan2(sIKX,sIKY));
    // remove gripper offset from base
    t = sqrt(sq((long)sIKX)+sq((long)sIKY));
    // BUGBUG... Not sure about G here
#define G 30   
    sol0 -= radToServo(atan2((G/2)-G_OFFSET,t));
  }
  else {
    // We are in cylindrical mode, probably simply set t to the y we passed in...
    t = sIKY;
#ifdef DEBUG
    sol0 = 0;
#endif
  }
  // convert to sIKX/sIKZ plane, remove wrist, prepare to solve other DOF           
  float flGripRad = (float)(sIKGA)*3.14159/180.0;
  long trueX = t - (long)((float)WristLength*cos(flGripRad));   
  long trueZ = sIKZ - BaseHeight - (long)((float)WristLength*sin(flGripRad));

  long im = sqrt(sq(trueX)+sq(trueZ));        // length of imaginary arm
  float q1 = atan2(trueZ,trueX);              // angle between im and X axis
  long d1 = sq(ShoulderLength) - sq(ElbowLength) + sq((long)im);
  long d2 = 2*ShoulderLength*im;
  float q2 = acos((float)d1/float(d2));
  q1 = q1 + q2;
  int sol1 = radToServo(q1-1.57);

  d1 = sq(ShoulderLength)-sq((long)im)+sq(ElbowLength);
  d2 = 2*ElbowLength*ShoulderLength;
  q2 = acos((float)d1/(float)d2);
  int sol2 = radToServo(3.14-q2);

  // solve for wrist rotate
  int sol3 = radToServo(3.2 + flGripRad - q1 - q2 );

#ifdef DEBUG
  if (g_fDebugOutput) {
    Serial.print("<");
    Serial.print(sol0, DEC);
    Serial.print(",");
    Serial.print(trueX, DEC);
    Serial.print(",");
    Serial.print(trueZ, DEC);
    Serial.print(",");
    Serial.print(sol1, DEC);
    Serial.print(",");
    Serial.print(sol2, DEC);
    Serial.print(",");
    Serial.print(sol3, DEC);
    Serial.print(">");
  }
#endif   

  // Lets calculate the actual servo values.

  if (fCartesian) {
    sBase = min(max(512 - sol0, BASE_MIN), BASE_MAX);
  }
  sShoulder = min(max(512 - sol1, SHOULDER_MIN), SHOULDER_MAX);

  // Magic Number 819???
  sElbow = min(max(819 - sol2, SHOULDER_MIN), SHOULDER_MAX);

#define Wrist_Offset 512
  sWrist = min(max(Wrist_Offset - sol3, WRIST_MIN), WRIST_MAX);

  // Remember our current IK positions
  g_sIKX = sIKX; 
  g_sIKY = sIKY;
  g_sIKZ = sIKZ;
  g_sIKGA = sIKGA;
  // Simple test im can not exceed the length of the Shoulder+Elbow joints...

  if (im > (ShoulderLength + ElbowLength)) {
    if (g_bIKStatus != IKS_ERROR) {
#ifdef DEBUG
      if (g_fDebugOutput) {
        Serial.println("IK Error");
      }
#endif
     
    }
    bRet = IKS_ERROR;  
  }
  else if(im > (ShoulderLength + ElbowLength-IK_FUDGE)) {
    if (g_bIKStatus != IKS_WARNING) {
#ifdef DEBUG
      if (g_fDebugOutput) {
        Serial.println("IK Warning");
      }
#endif
     
    }
    bRet = IKS_WARNING;  
  }

  return bRet;
}
//===================================================================================================
// MoveArmTo
//===================================================================================================
void MoveArmTo(int sBase, int sShoulder, int sElbow, int sWrist, int sWristRot, int sGrip, int wTime, boolean fWait) {

  int sMaxDelta;
  int sDelta;

  // First make sure servos are not free...
  if (g_fServosFree) {
    g_fServosFree = false;

    for(uint8_t i=1; i <= CNT_SERVOS; i++) {
      TorqueOn(i);
    }
    bioloid.readPose();
  }


#ifdef DEBUG
  if (g_fDebugOutput) {
    Serial.print("[");
    Serial.print(sBase, DEC);
    Serial.print(" ");
    Serial.print(sShoulder, DEC);
    Serial.print(" ");
    Serial.print(sElbow, DEC);
    Serial.print(" ");
    Serial.print(sWrist, DEC);
    Serial.print(" ");
    Serial.print(sWristRot, DEC);
    Serial.print(" ");
    Serial.print(sGrip, DEC);
    Serial.println("]");
  }
#endif
  // Make sure the previous movement completed.
  // Need to do it before setNextPos calls as this
  // is used in the interpolating code...
  while (bioloid.interpolating > 0) {
    bioloid.interpolateStep();
    delay(50);
  }

  // Also lets limit how fast the servos will move as to not get whiplash.
  bioloid.setNextPose(SID_BASE, sBase);

  sMaxDelta = abs(bioloid.getCurPose(SID_RSHOULDER) - sShoulder);
  bioloid.setNextPose(SID_RSHOULDER, sShoulder);
  bioloid.setNextPose(SID_LSHOULDER, 1024-sShoulder);

  sDelta = abs(bioloid.getCurPose(SID_RELBOW) - sElbow);
  if (sDelta > sMaxDelta)
    sMaxDelta = sDelta;
  bioloid.setNextPose(SID_RELBOW, sElbow);
  bioloid.setNextPose(SID_LELBOW, 1024-sElbow);

  sDelta = abs(bioloid.getCurPose(SID_WRIST) - sWrist);
  if (sDelta > sMaxDelta)
    sMaxDelta = sDelta;
  bioloid.setNextPose(SID_WRIST, sWrist);

#ifdef OPT_WRISTROT
  bioloid.setNextPose(SID_WRISTROT, sWristRot); 
#endif  

  bioloid.setNextPose(SID_GRIP, sGrip);


  // Save away the current positions...
  g_sBase = sBase;
  g_sShoulder = sShoulder;
  g_sElbow = sElbow;
  g_sWrist = sWrist;
  g_sWristRot = sWristRot;
  g_sGrip = sGrip;

  // Now start the move - But first make sure we don't move too fast.  
  if (((long)sMaxDelta*wTime/1000L) > MAX_SERVO_DELTA_PERSEC) {
    wTime = ((long)sMaxDelta*1000L)/ MAX_SERVO_DELTA_PERSEC;
  }
  //Serial.println(wTime);
  bioloid.interpolateSetup(wTime);

  // Do at least the first movement
  bioloid.interpolateStep();

  // And if asked to, wait for the previous move to complete...
  if (fWait) {
    while (bioloid.interpolating > 0) {
      bioloid.interpolateStep();
      delay(3);
    }
  }
}





