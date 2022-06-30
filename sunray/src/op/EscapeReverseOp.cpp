// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "op.h"
#include <Arduino.h>
#include "../../robot.h"
#include "../../StateEstimator.h"
#include "../../map.h"



String EscapeReverseOp::name(){
    return "EscapeReverse";
}

void EscapeReverseOp::begin(){
    // obstacle avoidance
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
		float sBeschl	= (OBSTACLEAVOIDANCEWAY / (mowerAccRamp + mowerDecRamp))*mowerAccRamp;		// calculate the part of the reversway by acceleration
		float sVerz		= (OBSTACLEAVOIDANCEWAY / (mowerAccRamp + mowerDecRamp))*mowerDecRamp;		// calculate the part of the reversway by acceleration
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
		
		CONSOLE.print("EscapeReversOP::begin aBeschl:");
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
		
      driveReverseStopTime = millis() + tBeschl + tKonst + tOffset;  // calculated time for reverse action
    #else
      driveReverseStopTime = millis() + 3000;
    #endif
}


void EscapeReverseOp::end(){
}


void EscapeReverseOp::run(){
    battery.resetIdle();
    motor.setLinearAngularSpeed(-OBSTACLEAVOIDANCESPEED,0);
    motor.setMowState(false);                                        

    if (millis() > driveReverseStopTime){
        CONSOLE.println("driveReverseStopTime");
        motor.stopImmediately(false); 
        driveReverseStopTime = 0;
        if (detectLift()) {
            CONSOLE.println("error: lift sensor!");
            stateSensor = SENS_LIFT;
            changeOp(errorOp);
            return;
        }
        if (maps.isDocking()){
            CONSOLE.println("continue docking");
            // continue without obstacles
            changeOp(*nextOp);    // continue current operation
        } else {
            CONSOLE.println("continue operation with virtual obstacle");
            maps.addObstacle(stateX, stateY);              
            //Point pt;
            //if (!maps.findObstacleSafeMowPoint(pt)){
            //    changeOp(dockOp); // dock if no more (valid) mowing points
            //} else changeOp(*nextOp);    // continue current operation
            changeOp(*nextOp);    // continue current operation
        }
    }
}



void EscapeReverseOp::onImuTilt(){
    stateSensor = SENS_IMU_TILT;
    changeOp(errorOp);
}

void EscapeReverseOp::onImuError(){
    stateSensor = SENS_IMU_TIMEOUT;
    changeOp(errorOp);
}


