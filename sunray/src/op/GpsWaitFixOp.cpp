// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "op.h"
#include <Arduino.h>
#include "../../robot.h"
#include "../../map.h"

unsigned long waitRecoveryRebootFloatTime	= 0;
bool RebootFloatTimeTrg	= false;

String GpsWaitFixOp::name(){
    return "GpsWaitFix";
}

void GpsWaitFixOp::begin(){
    CONSOLE.println("WARN: no gps solution!");
    stateSensor = SENS_GPS_INVALID;
	waitRecoveryRebootFloatTime	= millis();	// set time stamp
	RebootFloatTimeTrg	= false;
	
    //setOperation(OP_ERROR);
    //buzzer.sound(SND_STUCK, true);          
    
    //linear = 0;
    //angular = 0;      
    //mow = false;
    motor.setLinearAngularSpeed(0,0, true); 
    motor.setMowState(false);     
}


void GpsWaitFixOp::end(){
}

void GpsWaitFixOp::run(){
    battery.resetIdle();
	// try to reboot gps if mower stays to long in float without getting fix
	if (GPS_REBOOT_RECOVERY == true && REQUIRE_VALID_GPS == true && GPS_REBOOT_RECOVERY_FLOAT_TIME > 0 && RebootFloatTimeTrg == false){
		if (millis() > (waitRecoveryRebootFloatTime + (GPS_REBOOT_RECOVERY_FLOAT_TIME * 60000))){
			RebootFloatTimeTrg	= true;
			CONSOLE.println("GpsWaitFixOp: Waiting for GPS-fix exceeds GPS_REBOOT_RECOVERY_FLOAT_TIME. GPS-reboot is initiated!");
			gps.reboot();  // try to recover from GPS float
		}	
	}		
    if (gps.solution == SOL_FIXED){
		RebootFloatTimeTrg	= false;
        changeOp(*nextOp);
    }     
}


