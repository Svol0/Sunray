// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "op.h"
#include <Arduino.h>
#include "../../robot.h"
#include "../../LineTracker.h"
#include "../../map.h"
#include "../../config.h"

unsigned long maxWaitTimeForRecoverGPS = 0;	// value in ms

String KidnapWaitOp::name(){
  return "KidnapWait";
}

void KidnapWaitOp::begin(){    
  stateSensor = SENS_KIDNAPPED;
  if (GPS_COLD_REBOOT == true){
    maxWaitTimeForRecoverGPS = (60000 * 5);	// 5 minuten
  } else {
    maxWaitTimeForRecoverGPS = 30000;	// value in ms
  }
  CONSOLE.print("KidnapWait.cpp: maxWaitTimeForRecoverGPS");
  CONSOLE.print(maxWaitTimeForRecoverGPS);
  CONSOLE.println(" ms");  
  recoverGpsTime = millis() + maxWaitTimeForRecoverGPS;
  recoverGpsCounter = 0;
}


void KidnapWaitOp::end(){
}

void KidnapWaitOp::onKidnapped(bool state){
  if (!state) {
    changeOp(*nextOp);
  }
}

void KidnapWaitOp::onGpsNoSignal(){
    if (!maps.isUndocking()){
        stateSensor = SENS_GPS_INVALID;
        changeOp(gpsWaitFloatOp, true);
    }
}


void KidnapWaitOp::run(){  
  trackLine(false);       
  battery.resetIdle();

  if (millis() > recoverGpsTime){
    CONSOLE.println("KIDNAP_DETECT");
	if (GPS_COLD_REBOOT == true){
	  maxWaitTimeForRecoverGPS = (60000 * 5);	// 5 minuten
	} else {
	  maxWaitTimeForRecoverGPS = 30000;	// value in ms
	}
	CONSOLE.print("KidnapWait.cpp: maxWaitTimeForRecoverGPS");
	CONSOLE.print(maxWaitTimeForRecoverGPS);
	CONSOLE.println(" ms");	
    recoverGpsTime = millis() + maxWaitTimeForRecoverGPS;
    recoverGpsCounter++;
    if (recoverGpsCounter == 3){          
      CONSOLE.println("error: kidnapped!");
      stateSensor = SENS_KIDNAPPED;
      changeOp(errorOp);
      return;
    }   
    if (GPS_REBOOT_RECOVERY){           
      gps.reboot();   // try to recover from false GPS fix
    }
  }
}


