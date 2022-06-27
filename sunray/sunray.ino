// Ardumower Sunray V1.0.276  with added GPS-Reboot by undocking; map setSpeed to app joystickspeed; speed parameters; new linear ramp; reduced bumper sensitivity
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


//  ---------------------------------------------------------------------------------------------------------
//
//  NOTE: Before uploading the code, please: 
//  1. Rename file 'config_example.h' into 'config.h'
//  2. Open the file config.h and verify (configure) your hardware modules!
//  ---------------------------------------------------------------------------------------------------------

#include "config.h"  // see note above if you get an error here!
#include "robot.h"


void setup(){
  start();
} 

void loop(){  
  run();
}
