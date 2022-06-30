// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "op.h"
#include <Arduino.h>
#include "../../robot.h"
#include "../../map.h"


String EscapeForwardOp::name(){
    return "EscapeForward";
}

void EscapeForwardOp::begin(){
    // rotate stuck avoidance
	#if USE_LINEAR_SPEED_RAMP
		// new calculation for reverse way
		float mowerObstacleAvoidanceSpeed = OBSTACLEAVOIDANCESPEED;
		if ((mowerObstacleAvoidanceSpeed == 0) || (mowerObstacleAvoidanceSpeed > MOTOR_MAX_SPEED)) mowerObstacleAvoidanceSpeed = 0.1;
		float mowerAccRamp = (ACC_RAMP/MOTOR_MAX_SPEED)*mowerObstacleAvoidanceSpeed;
		float mowerDecRamp = (DEC_RAMP/MOTOR_MAX_SPEED)*mowerObstacleAvoidanceSpeed;
		if (mowerAccRamp == 0) mowerAccRamp = 1;
		if (mowerDecRamp == 0) mowerDecRamp = 1;	
		float wayAccRamp	= (mowerObstacleAvoidanceSpeed * (mowerAccRamp/1000)) / 2;
		float wayDecRamp	= (mowerObstacleAvoidanceSpeed * (mowerDecRamp/1000)) / 2;		
		float aBeschl	= (mowerObstacleAvoidanceSpeed * 1000) / mowerAccRamp;							// calculate acceleration by obstacle avoidance speed and ramp
		float sBeschl	= (0.3 / (mowerAccRamp + mowerDecRamp))*mowerAccRamp;		// calculate the part of the reversway by acceleration (30cm fix)
		float sVerz		= (0.3 / (mowerAccRamp + mowerDecRamp))*mowerDecRamp;		// calculate the part of the reversway by acceleration (30cm fix)
		float tBeschl	= 0;
		float sKonst	= 0;
		float tOffset	= 0;
		if (wayAccRamp < sBeschl){
			sKonst = sBeschl - wayAccRamp;
			tBeschl = mowerAccRamp;
			tOffset	= (0.1 / mowerObstacleAvoidanceSpeed)*1000;	// add 10cm more (result by testing)
		} else tBeschl	= (sqrt((2*sBeschl)/aBeschl) *1000);												// calculate time for reverse action
			
		if (wayDecRamp < sVerz) sKonst = sKonst + (sVerz - wayDecRamp);
		float tKonst	= (sKonst / mowerObstacleAvoidanceSpeed)*1000;
		
		CONSOLE.print("EscapeForwardOP::begin aBeschl:");
		CONSOLE.print(aBeschl);
		CONSOLE.print(" | sBeschl:");
		CONSOLE.print(sBeschl);
		CONSOLE.print(" | tBeschl:");
		CONSOLE.print(tBeschl);
		CONSOLE.print(" | wayAccRamp:");
		CONSOLE.print(wayAccRamp);
		CONSOLE.print(" | wayDecRamp:");
		CONSOLE.print(wayDecRamp);
		CONSOLE.print(" | sKonst:");
		CONSOLE.print(sKonst);
		CONSOLE.print(" | tKonst:");
		CONSOLE.println(tKonst);
		
      driveForwardStopTime = millis() + tBeschl + tKonst + tOffset;  // calculated time for reverse action
	#else
		driveForwardStopTime = millis() + 2000;
	#endif
}


void EscapeForwardOp::end(){
}


void EscapeForwardOp::run(){
    battery.resetIdle();
    motor.setLinearAngularSpeed(OBSTACLEAVOIDANCESPEED,0);
    motor.setMowState(false);                

    if (millis() > driveForwardStopTime){
        CONSOLE.println("driveForwardStopTime");
        motor.stopImmediately(false);  
        driveForwardStopTime = 0;
        /*maps.addObstacle(stateX, stateY);
        Point pt;
        if (!maps.findObstacleSafeMowPoint(pt)){
        setOperation(OP_DOCK, true); // dock if no more (valid) mowing points
        } else*/ 
        changeOp(*nextOp);    // continue current operation              
    }
}

void EscapeForwardOp::onImuTilt(){
    stateSensor = SENS_IMU_TILT;
    changeOp(errorOp);
}

void EscapeForwardOp::onImuError(){
    stateSensor = SENS_IMU_TIMEOUT;
    changeOp(errorOp);
}
