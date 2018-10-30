#include <ax12.h>
#include <BioloidController.h>
#include "poses.h"

#define BAUD 9600
#define SERVOCOUNT 8

BioloidController bioloid = BioloidController(1000000);
int val;

void setup(){
  Serial.begin(BAUD);
  //MoveCenter();
  //RelaxServos();
  MoveCenter();
  delay(1000);
}

void loop(){
  if (Serial.available() > 0) {
    //val = Serial.read();
      while(Serial.available()){
        val = Serial.read();
        if(val > 0){
          SetPosition(1,val);
           Serial.println(val);
        }
       delay(5);
      }
    //SetPosition(1,val);
  }
  //delay(1000);
    
}
void MoveCenter(){
    delay(100);                    // recommended pause
    bioloid.loadPose(Center);   // load the pose from FLASH, into the nextPose buffer
    bioloid.readPose();            // read in current servo positions to the curPose buffer
    Serial.println("###########################");
    Serial.println("Moving servos to centered position");
    Serial.println("###########################");    
    delay(1000);
    bioloid.interpolateSetup(1000); // setup for interpolation from current->next over 1/2 a second
    while(bioloid.interpolating > 0){  // do this while we have not reached our new pose
        bioloid.interpolateStep();     // move servos, if necessary. 
        delay(50);
    }
   // if (RunCheck == 1){
   //   MenuOptions();
  //}
}

void RelaxServos(){
  int id;
  id = 1;
  Serial.println("###########################");
  Serial.println("Relaxing Servos.");
  Serial.println("###########################");    
  while(id <= SERVOCOUNT){
    Relax(id);
    id = (id++)%SERVOCOUNT;
    delay(50);
  }
  /* if (RunCheck == 1){
      MenuOptions();
  }*/
}
