// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "motor.h"
#include "config.h"
#include "helper.h"
#include "robot.h"
#include "Arduino.h"
#include <RunningMedian.h>

RunningMedian samples = RunningMedian(MOWMOTOR_CURRENT_MEDIAN_LEN);

unsigned long adaptivSpeedTimer = 0;
bool mowMsgTrg  = false;

bool mowTestActiv = false;
int pwmMowTest  = 0;

void Motor::begin() {
	pwmMax = 255;
 
  #ifdef MAX_MOW_RPM
    if (MAX_MOW_RPM <= 255) {
      pwmMaxMow = MAX_MOW_RPM;
    }
    else pwmMaxMow = 255;
  #else 
    pwmMaxMow = 255;
  #endif
  
  SpeedOffset = 1.0; //X
  adaptivSpeedTimer = millis();
  
  // calc ramp steps
  accStep = ((float)MOTOR_MAX_SPEED * 20) / (float)ACC_RAMP;
  decStep = ((float)MOTOR_MAX_SPEED *20) / (float)DEC_RAMP;
  
  lastLinearSetTime = millis();
  calcStopWay       = 0;
  
  //ticksPerRevolution = 1060/2;
  ticksPerRevolution = TICKS_PER_REVOLUTION;
	wheelBaseCm = WHEEL_BASE_CM;    // wheel-to-wheel distance (cm) 36
  wheelDiameter = WHEEL_DIAMETER; // wheel diameter (mm)
  ticksPerCm         = ((float)ticksPerRevolution) / (((float)wheelDiameter)/10.0) / 3.1415;    // computes encoder ticks per cm (do not change)  

  // check for MOW_TICKS_PER_REVOLUTION value
  #ifdef MOW_TICKS_PER_REVOLUTION
    ticksPerMowMotorRevolution = MOW_TICKS_PER_REVOLUTION;
    if (ticksPerMowMotorRevolution == 0) ticksPerMowMotorRevolution = 1;
  #else
    ticksPerMowMotorRevolution = 1; // avoid div by zero
  #endif

  motorLeftPID.Kp       = MOTOR_PID_KP;  // 2.0;  
  motorLeftPID.Ki       = MOTOR_PID_KI;  // 0.03; 
  motorLeftPID.Kd       = MOTOR_PID_KD;  // 0.03; 
  motorRightPID.Kp       = motorLeftPID.Kp;
  motorRightPID.Ki       = motorLeftPID.Ki;
  motorRightPID.Kd       = motorLeftPID.Kd;		 

  robotPitch = 0;
  #ifdef MOTOR_DRIVER_BRUSHLESS
    motorLeftSwapDir = true;
  #else
    motorLeftSwapDir = false;  
  #endif
  motorRightSwapDir = false;
  motorError = false;
  resetMotorFault = false;
  resetMotorFaultCounter = 0;
  nextResetMotorFaultTime = 0;
  enableMowMotor = true;
  
  motorLeftOverload = false;
  motorRightOverload = false;
  motorMowOverload = false; 
  
  odometryError = false;  
  
  motorLeftSense = 0;
  motorRightSense = 0;
  motorMowSense = 0;  
  motorLeftSenseLP = 0;
  motorRightSenseLP = 0;
  motorMowSenseLP = 0;  
  motorsSenseLP = 0;
  motorMowSenseMed = 0; //X

  activateLinearSpeedRamp = USE_LINEAR_SPEED_RAMP;
  linearSpeedSet = 0;
  angularSpeedSet = 0;
  motorLeftRpmSet = 0;
  motorRightRpmSet = 0;
  motorMowPWMSet = 0;
  motorMowForwardSet = true;
  toggleMowDir = MOW_TOGGLE_DIR;

  lastControlTime = 0;
  nextSenseTime = 0;
  motorLeftTicks =0;  
  motorRightTicks =0;
  motorLeftTicksZero=0;
  motorRightTicksZero=0;
  motorLeftPWMCurr =0;    
  motorRightPWMCurr=0; 
  motorMowPWMCurr = 0;
  motorLeftPWMCurrLP = 0;
  motorRightPWMCurrLP=0;   
  motorMowPWMCurrLP = 0;
  motorLeftRpmCurr=0;
  motorRightRpmCurr=0;
  motorLeftRpmLast = 0;
  motorRightRpmLast = 0;
  setLinearAngularSpeedTimeoutActive = false;  
  setLinearAngularSpeedTimeout = 0;
  motorMowSpinUpTime = 0;
}


void Motor::speedPWM ( int pwmLeft, int pwmRight, int pwmMow )
{
  if ((pwmMow != 0) && (ADAPTIVE_SPEED)) {
    switch (ADAPTIVE_SPEED_ALGORITHM) {
      //Simple 2 point controller that applies a linear ramp to mowerspeed through a offset.
      //The delta of SPEEDDOWNCURRENT-SPEEDUPCURRENT is the hysteresis
      case 1:
        if (motorMowSenseMed > SPEEDDOWNCURRENT){
         if (millis() > motor.motorMowSpinUpTime + MOW_SPINUPTIME){ //avoid trigger by speed up
            adaptivSpeedTimer = millis();
            SpeedOffset = SPEED_FACTOR_MIN;
            //if (pwmMaxMow < MAX_MOW_RPM) 
            pwmMaxMow = MAX_MOW_RPM;
          }
        }
        if (motorMowSenseMed < SPEEDUPCURRENT){
          if ((millis() - adaptivSpeedTimer) > 10000){
          SpeedOffset = SPEED_FACTOR_MAX;        
          // if (pwmMaxMow > MIN_MOW_RPM) 
          pwmMaxMow = MIN_MOW_RPM;
          }
        }
        break;
      case 2:
      //empty  
        break;
      case 3:
      //empty  
        break;
      default:
      //empty  
        break;
    }
    SpeedOffset = min(SPEED_FACTOR_MAX, max(SPEED_FACTOR_MIN, SpeedOffset));
  } else if ((pwmMow = 0) && (ADAPTIVE_SPEED)) SpeedOffset = SPEED_FACTOR_MAX;

   
   //########################  Check pwm higher than Max ############################

  if (mowTestActiv == true) pwmMaxMow = 255;  // if mow motor test is runing the is no limitation command: AT+D
  
  pwmLeft = min(pwmMax, max(-pwmMax, pwmLeft));
  pwmRight = min(pwmMax, max(-pwmMax, pwmRight));  
  pwmMow = min(pwmMaxMow, max(-pwmMaxMow, pwmMow));  
    
  if (motorLeftSwapDir) pwmLeft *= -1;
  if (motorRightSwapDir) pwmRight *= -1;
  motorDriver.setMotorPwm(pwmLeft, pwmRight, pwmMow);
  /*
  CONSOLE.print("pwmMow: ");
   CONSOLE.print(pwmMow);
   CONSOLE.print(" | pwmMaxMow: ");
   CONSOLE.print(pwmMaxMow); 
   CONSOLE.print(" | motorMowPWMSet: ");
   CONSOLE.print(motorMowPWMSet);
   CONSOLE.print(" | motorMowForwardSet: ");
   CONSOLE.println(motorMowForwardSet);
   */
}

// linear: m/s
// angular: rad/s
// -------unicycle model equations----------
//      L: wheel-to-wheel distance
//     VR: right speed (m/s)
//     VL: left speed  (m/s)
//  omega: rotation speed (rad/s)
//      V     = (VR + VL) / 2       =>  VR = V + omega * L/2
//      omega = (VR - VL) / L       =>  VL = V - omega * L/2
void Motor::setLinearAngularSpeed(float linear, float angular, bool useLinearRamp){
   setLinearAngularSpeedTimeout = millis() + 1000;
   setLinearAngularSpeedTimeoutActive = true;
   if ((activateLinearSpeedRamp) && (useLinearRamp)) {
   //  linearSpeedSet = 0.95 * linearSpeedSet + 0.05 * linear;

      if (millis() >= lastLinearSetTime) {
        if ((millis()+40) >= lastLinearSetTime) lastLinearSetTime = millis();
        calcStopWay = (linearSpeedSet * DEC_RAMP) / 2000;
        /*
        CONSOLE.print("setLinearAngularSpeed: last call: ");
        CONSOLE.print((millis() - lastLinearSetTime));
        CONSOLE.print(" calcStopWay: ");
        CONSOLE.print(calcStopWay);
        CONSOLE.print(" targetDist: ");
        CONSOLE.print(targetDist);
        CONSOLE.print("m | acc-step: ");
        CONSOLE.print(accStep);
        CONSOLE.print(" dec-step: ");
        CONSOLE.print(decStep);
        CONSOLE.print(" linearSpeedSet: ");
        CONSOLE.print(linearSpeedSet);
        CONSOLE.print(" linear: ");
        CONSOLE.println(linear);
*/
        if (ADAPTIVE_SPEED) linear = linear * SpeedOffset; // add speedoffset by use of ADAPTIVE_SPEED
        if (linear > 0) { // pos value
          if (linearSpeedSet < 0) linearSpeedSet = linearSpeedSet + decStep; // noch pos
          else if (linearSpeedSet < linear) linearSpeedSet = linearSpeedSet + accStep; // speed up
          else linearSpeedSet = linearSpeedSet - decStep; // slow down
        } else if (linear < 0) { // neg value
            if (linearSpeedSet > 0) linearSpeedSet = linearSpeedSet - decStep; // noch pos
            else if (linearSpeedSet < linear) linearSpeedSet = linearSpeedSet + decStep; 
            else linearSpeedSet = linearSpeedSet - accStep;
        } else { // linear = 0
          if (linearSpeedSet > 0) linearSpeedSet = linearSpeedSet - decStep;
          else linearSpeedSet = linearSpeedSet + decStep;
        }
        if ((linear == 0) && (linearSpeedSet != 0) && (fabs(linearSpeedSet) < decStep)) linearSpeedSet = 0;
        
        lastLinearSetTime = lastLinearSetTime + 20; // Rampe alle 20ms anpassen

      }
   } else {
     if (ADAPTIVE_SPEED) linear = linear * SpeedOffset; // add speedoffset by use of ADAPTIVE_SPEED
     linearSpeedSet = linear;
   }
   angularSpeedSet = angular;   
   float rspeed = linearSpeedSet + angularSpeedSet * (wheelBaseCm /100.0 /2);          
   float lspeed = linearSpeedSet - angularSpeedSet * (wheelBaseCm /100.0 /2);          
   // RPM = V / (2*PI*r) * 60
   motorRightRpmSet =  rspeed / (PI*(((float)wheelDiameter)/1000.0)) * 60.0;   
   motorLeftRpmSet = lspeed / (PI*(((float)wheelDiameter)/1000.0)) * 60.0;   
   /*CONSOLE.print("setLinearAngularSpeed ");
   CONSOLE.print(linear);
   CONSOLE.print(",");
   CONSOLE.print(rspeed);
   CONSOLE.print(",");
   CONSOLE.println(motorRightRpmSet);   */
}

void Motor::setMowState(bool switchOn){
  if ((enableMowMotor) && (switchOn)){
    if (!mowMsgTrg){
      CONSOLE.println("Mowmotor switched on");
      mowMsgTrg = true;
    }
    if ((abs(motorMowPWMSet) > 0) || (abs(motorMowPWMSet) != abs(pwmMaxMow))){
      if (abs(motorMowPWMSet) == abs(pwmMaxMow)){
        return; // mowing motor already switch ON or set value for pwmMaxMow changes, motorMowPWMSet need to be updated
      }
    }
    if (abs(motorMowPWMSet) == 0) motorMowSpinUpTime = millis();
    if (toggleMowDir){
      // toggle mowing motor direction each mow motor start
      if (abs(motorMowPWMSet) == 0) motorMowForwardSet = !motorMowForwardSet; // only toggel, if it is started from zero
      if (motorMowForwardSet) motorMowPWMSet = pwmMaxMow; 
        else motorMowPWMSet = -pwmMaxMow;  
    }  else  {      
      motorMowPWMSet = pwmMaxMow;  
    }
  } else {
    motorMowPWMSet = 0;  
    motorMowPWMCurr = 0;
    if (mowMsgTrg){
      CONSOLE.println("Mowmotor switched off");
      mowMsgTrg = false;
    }
  }
  SpeedOffset = 1.0; // reset Mow SpeedOffset
}

void Motor::stopImmediately(bool includeMowerMotor){
  linearSpeedSet = 0;
  angularSpeedSet = 0;
  motorRightRpmSet = 0;
  motorLeftRpmSet = 0;      
  motorLeftPWMCurr = 0;
  motorRightPWMCurr = 0;   
  if (includeMowerMotor) {
    motorMowPWMSet = 0;
    motorMowPWMCurr = 0;    
  }
  speedPWM(motorLeftPWMCurr, motorRightPWMCurr, motorMowPWMCurr);
}


void Motor::run() {
  if (millis() < lastControlTime + 50) return;
  
  if (setLinearAngularSpeedTimeoutActive){
    if (millis() > setLinearAngularSpeedTimeout){
      setLinearAngularSpeedTimeoutActive = false;
      motorLeftRpmSet = 0;
      motorRightRpmSet = 0;
    }
  }
    
  sense();        
  
  if ((!resetMotorFault) && (checkFault())) {
    stopImmediately(true);
    resetMotorFault = true;
    nextResetMotorFaultTime = millis() + 1000;
  }

  if (nextResetMotorFaultTime != 0){
    if (millis() > nextResetMotorFaultTime){
      if (resetMotorFault){
        nextResetMotorFaultTime = millis() + 5000;
        CONSOLE.print("resetMotorFaultCounter ");
        CONSOLE.println(resetMotorFaultCounter);
        resetMotorFaultCounter++;        
        motorDriver.resetMotorFaults();
        resetMotorFault = false;  
        if (resetMotorFaultCounter > 10){ // too many successive motor faults
          //stopImmediately();
          CONSOLE.println("ERROR: motor recovery failed");
          motorError = true;
        }
      } else {
        resetMotorFaultCounter = 0;
        nextResetMotorFaultTime = 0;
      }        
    }
  }

  if (nextResetMotorFaultTime == 0) {    
    if  (   ( (abs(motorLeftPWMCurr) > 100) && (abs(motorLeftPWMCurrLP) > 100) && (abs(motorLeftRpmCurr) < 0.001))    
        ||  ( (abs(motorRightPWMCurr) > 100) && (abs(motorRightPWMCurrLP) > 100) && (abs(motorRightRpmCurr) < 0.001))  )
    {               
      if (!odometryError){
        // odometry error
        CONSOLE.print("ERROR: odometry error rpm=");
        CONSOLE.print(motorLeftRpmCurr);
        CONSOLE.print(",");
        CONSOLE.println(motorRightRpmCurr);     
        odometryError = true;
      }      
    } else odometryError = false;
      
    if  (    ( (abs(motorMowPWMCurr) > 100) && (abs(motorMowPWMCurrLP) > 100) && (motorMowSenseLP < 0.005)) 
         ||  ( (abs(motorLeftPWMCurr) > 100) && (abs(motorLeftPWMCurrLP) > 100) && (motorLeftSenseLP < 0.005))    
         ||  ( (abs(motorRightPWMCurr) > 100) && (abs(motorRightPWMCurrLP) > 100) && (motorRightSenseLP < 0.005))  ){        
      // at least one motor is not consuming current      
      if (!motorError){
        CONSOLE.print("ERROR: motor malfunction pwm=");
        CONSOLE.print(motorLeftPWMCurr);
        CONSOLE.print(",");
        CONSOLE.print(motorRightPWMCurr);
        CONSOLE.print(",");
        CONSOLE.print(motorMowPWMCurr);
        CONSOLE.print("  sense=");
        CONSOLE.print(motorLeftSenseLP);
        CONSOLE.print(",");
        CONSOLE.print(motorRightSenseLP);
        CONSOLE.print(",");
        CONSOLE.println(motorMowSenseLP);
        motorError = true;
      }
    }
  }   
  
  int ticksLeft;
  int ticksRight;
  int ticksMow;
  motorDriver.getMotorEncoderTicks(ticksLeft, ticksRight, ticksMow);  
  
  if (motorLeftPWMCurr < 0) ticksLeft *= -1;
  if (motorRightPWMCurr < 0) ticksRight *= -1;
  motorLeftTicks += ticksLeft;
  motorRightTicks += ticksRight;

  unsigned long currTime = millis();
  float deltaControlTimeSec =  ((float)(currTime - lastControlTime)) / 1000.0;
  lastControlTime = currTime;

  // calculate speed via tick count
  // 2000 ticksPerRevolution: @ 30 rpm  => 0.5 rps => 1000 ticksPerSec
  // 20 ticksPerRevolution: @ 30 rpm => 0.5 rps => 10 ticksPerSec
  motorLeftRpmCurr = 60.0 * ( ((float)ticksLeft) / ((float)ticksPerRevolution) ) / deltaControlTimeSec;
  motorRightRpmCurr = 60.0 * ( ((float)ticksRight) / ((float)ticksPerRevolution) ) / deltaControlTimeSec;
  motorMowRpmCurr = 60.0 * ( ((float)ticksMow) / ((float)ticksPerMowMotorRevolution) ) / deltaControlTimeSec;

  if (ticksLeft == 0) {
    motorLeftTicksZero++;
    if (motorLeftTicksZero > 2) motorLeftRpmCurr = 0;
  } else motorLeftTicksZero = 0;

  if (ticksRight == 0) {
    motorRightTicksZero++;
    if (motorRightTicksZero > 2) motorRightRpmCurr = 0;
  } else motorRightTicksZero = 0;

  // speed controller
  control();    
  motorLeftRpmLast = motorLeftRpmCurr;
  motorRightRpmLast = motorRightRpmCurr;
}  



// check motor faults
// NOTE: motor drivers will indicate 'fault' signal if motor current (e.g. due to a stall on a molehole) or temperature is too high for a 
// certain time (normally a few seconds)  
bool Motor::checkFault() {
  bool fault = false;
  bool leftFault = false;
  bool rightFault = false;
  bool mowFault = false;
  motorDriver.getMotorFaults(leftFault, rightFault, mowFault);
  if (leftFault) {
    CONSOLE.println("Error: motor left fault");
    fault = true;
  }
  if  (rightFault) {
    CONSOLE.println("Error: motor right fault"); 
    fault = true;
  }
  if (mowFault) {
    CONSOLE.println("Error: motor mow fault");
    fault = true;
  }
  return fault;
}
  

// measure motor currents
void Motor::sense(){
  if (millis() < nextSenseTime) return;
  nextSenseTime = millis() + 20;
  motorDriver.getMotorCurrent(motorLeftSense, motorRightSense, motorMowSense);

  // correct the 
  motorLeftSense -= GEAR_DRIVER_IDLE_CURRENT; // correct value
  if (motorLeftSense < 0) motorLeftSense = 0;
  motorRightSense -= GEAR_DRIVER_IDLE_CURRENT; // correct value
  if (motorRightSense < 0) motorRightSense = 0;

  motorMowSense -= MOW_DRIVER_IDLE_CURRENT; // correct value
  if (motorMowSense < 0) motorMowSense = 0;
  
  float lp = 0.995; // 0.9
  motorRightSenseLP = lp * motorRightSenseLP + (1.0-lp) * motorRightSense;
  motorLeftSenseLP = lp * motorLeftSenseLP + (1.0-lp) * motorLeftSense;
  motorMowSenseLP = lp * motorMowSenseLP + (1.0-lp) * motorMowSense;
  samples.add(motorMowSense); //Puts Values of motorMowSense into median function
  motorMowSenseMed = samples.getMedian(); //Get the Running Median as motorMowSenseMed
  motorsSenseLP = motorRightSenseLP + motorLeftSenseLP + motorMowSenseLP;
  motorRightPWMCurrLP = lp * motorRightPWMCurrLP + (1.0-lp) * ((float)motorRightPWMCurr);
  motorLeftPWMCurrLP = lp * motorLeftPWMCurrLP + (1.0-lp) * ((float)motorLeftPWMCurr);
  lp = 0.99;
  motorMowPWMCurrLP = lp * motorMowPWMCurrLP + (1.0-lp) * ((float)motorMowPWMCurr); 
  
  // compute normalized current (normalized to 1g gravity)
  //float leftAcc = (motorLeftRpmCurr - motorLeftRpmLast) / deltaControlTimeSec;
  //float rightAcc = (motorRightRpmCurr - motorRightRpmLast) / deltaControlTimeSec;
  float cosPitch = cos(robotPitch); 
	float pitchfactor;
  float robotMass = 1.0;
	// left wheel friction
	if (  ((motorLeftPWMCurr >= 0) && (robotPitch <= 0)) || ((motorLeftPWMCurr < 0) && (robotPitch >= 0)) )
		pitchfactor = cosPitch; // decrease by angle
	else 
		pitchfactor = 2.0-cosPitch;  // increase by angle
	motorLeftSenseLPNorm = abs(motorLeftSenseLP) * robotMass * pitchfactor;  
	// right wheel friction
	if (  ((motorRightPWMCurr >= 0) && (robotPitch <= 0)) || ((motorRightPWMCurr < 0) && (robotPitch >= 0)) )
		pitchfactor = cosPitch;  // decrease by angle
	else 
		pitchfactor = 2.0-cosPitch; // increase by angle
  motorRightSenseLPNorm = abs(motorRightSenseLP) * robotMass * pitchfactor; 

  // NOTE: motor drivers will indicate 'fault' signal if motor current (e.g. due to a stall on a molehole) or temperature is too high for 
  // a certain time (normally a few seconds)
  // However, the following overload detection will detect situations the fault signal cannot detect: slightly higher current for a longer time 
  motorLeftOverload = (motorLeftSenseLP > MOTOR_OVERLOAD_CURRENT);
  motorRightOverload = (motorRightSenseLP > MOTOR_OVERLOAD_CURRENT);
  motorMowOverload = (motorMowSenseMed > MOW_OVERLOAD_CURRENT);
  if (motorLeftOverload || motorRightOverload || motorMowOverload){
    if (motorOverloadDuration == 0){
      CONSOLE.print("ERROR motor overload duration=");
      CONSOLE.print(motorOverloadDuration);
      CONSOLE.print("  current=");
      CONSOLE.print(motorLeftSenseLP);
      CONSOLE.print(",");
      CONSOLE.print(motorRightSenseLP);
      CONSOLE.print(",");
      CONSOLE.println(motorMowSenseLP);
    }
    motorOverloadDuration += 20;     
  } else {
    motorOverloadDuration = 0;
  }  
}


void Motor::control(){  
  /*CONSOLE.print("rpm set=");
  CONSOLE.print(motorLeftRpmSet);
  CONSOLE.print(",");
  CONSOLE.print(motorRightRpmSet);
  CONSOLE.print("   curr=");
  CONSOLE.print(motorLeftRpmCurr);
  CONSOLE.print(",");
  CONSOLE.println(motorRightRpmCurr);*/
  motorLeftPID.x = motorLeftRpmCurr;
  motorLeftPID.w  = motorLeftRpmSet;
  motorLeftPID.y_min = -pwmMax;
  motorLeftPID.y_max = pwmMax;
  motorLeftPID.max_output = pwmMax;
  motorLeftPID.compute();
  motorLeftPWMCurr = motorLeftPWMCurr + motorLeftPID.y;
  if (motorLeftRpmSet >= 0) motorLeftPWMCurr = min( max(0, (int)motorLeftPWMCurr), pwmMax); // 0.. pwmMax
  if (motorLeftRpmSet < 0) motorLeftPWMCurr = max(-pwmMax, min(0, (int)motorLeftPWMCurr));  // -pwmMax..0
  
  motorRightPID.x = motorRightRpmCurr;
  motorRightPID.w = motorRightRpmSet;
  motorRightPID.y_min = -pwmMax;
  motorRightPID.y_max = pwmMax;
  motorRightPID.max_output = pwmMax;
  motorRightPID.compute();
  motorRightPWMCurr = motorRightPWMCurr + motorRightPID.y;
  if (motorRightRpmSet >= 0) motorRightPWMCurr = min( max(0, (int)motorRightPWMCurr), pwmMax);  // 0.. pwmMax
  if (motorRightRpmSet < 0) motorRightPWMCurr = max(-pwmMax, min(0, (int)motorRightPWMCurr));   // -pwmMax..0  

  if ((abs(motorLeftRpmSet) < 0.01) && (motorLeftPWMCurr < 30)) motorLeftPWMCurr = 0;
  if ((abs(motorRightRpmSet) < 0.01) && (motorRightPWMCurr < 30)) motorRightPWMCurr = 0;
  
  //########################  Calculate PWM for mowing motor ############################

  if (ADAPTIVE_SPEED == true){
    if (motorMowPWMCurr >= MIN_MOW_RPM) motorMowPWMCurr = 0.92 * motorMowPWMCurr + 0.08 * motorMowPWMSet; // faster speed change between MIN_MOW_RPM and MAX_MOW_RPM
    else motorMowPWMCurr = 0.99 * motorMowPWMCurr + 0.01 * motorMowPWMSet;  // slow speedup till MIN_MOW_RPM
  } else motorMowPWMCurr = 0.99 * motorMowPWMCurr + 0.01 * motorMowPWMSet;
  
  speedPWM(motorLeftPWMCurr, motorRightPWMCurr, motorMowPWMCurr);
}


void Motor::dumpOdoTicks(int seconds){
  int ticksLeft=0;
  int ticksRight=0;
  int ticksMow=0;
  motorDriver.getMotorEncoderTicks(ticksLeft, ticksRight, ticksMow);  
  motorLeftTicks += ticksLeft;
  motorRightTicks += ticksRight;
  CONSOLE.print("t=");
  CONSOLE.print(seconds);
  CONSOLE.print("  ticks Left=");
  CONSOLE.print(motorLeftTicks);  
  CONSOLE.print("  Right=");
  CONSOLE.print(motorRightTicks);             
  CONSOLE.print("  current Left=");
  CONSOLE.print(motorLeftSense);
  CONSOLE.print("  Right=");
  CONSOLE.print(motorRightSense);
  CONSOLE.println();               
}

void Motor::dumpOdoMowTicks(){
  int ticksLeft=0;
  int ticksRight=0;
  int ticksMow=0;
  motorDriver.getMotorEncoderTicks(ticksLeft, ticksRight, ticksMow);  
  motorMowTicks += ticksMow;

  unsigned long currTime = millis();
  float deltaControlTimeSec =  ((float)(currTime - lastControlTime)) / 1000.0;
  lastControlTime = currTime;

  motorMowRpmCurr = 60.0 * ( ((float)ticksMow) / ((float)ticksPerMowMotorRevolution) ) / deltaControlTimeSec;

  float lp = 0.9; // 0.995
  motorMowRpmCurrLP = lp * motorMowRpmCurrLP + (1.0-lp) * motorMowRpmCurr;

  CONSOLE.print(" pwmMow: ");
  CONSOLE.print(pwmMowTest);
  CONSOLE.print(" | RpmCurr: ");
  CONSOLE.print(motorMowRpmCurr);
  CONSOLE.print("U/min | RpmCurrLP: ");
  CONSOLE.print(motorMowRpmCurrLP);
  CONSOLE.print("U/min | Sense: ");
  CONSOLE.print(motorMowSense); // motorstrom
  CONSOLE.print("A | SenseLP: ");        
  CONSOLE.print(motorMowSenseLP); // motorstrom
  CONSOLE.println("A");
}

void Motor::test(){
  CONSOLE.println("motor test - 10 revolutions");
  motorLeftTicks = 0;  
  motorRightTicks = 0;  
  unsigned long nextInfoTime = 0;
  int seconds = 0;
  int pwmLeft = 200;
  int pwmRight = 200; 
  bool slowdown = true;
  unsigned long stopTicks = ticksPerRevolution * 10;
  unsigned long nextControlTime = 0;
//  while (motorLeftTicks < stopTicks || motorRightTicks < stopTicks){
  while (motorLeftTicks < stopTicks || motorRightTicks < stopTicks || seconds < 45){  //abbort if test takes longer than 45 Seconds
    if (millis() > nextControlTime){
      nextControlTime = millis() + 20;
      if ((slowdown) && ((motorLeftTicks + ticksPerRevolution  > stopTicks)||(motorRightTicks + ticksPerRevolution > stopTicks))){  //Letzte halbe drehung verlangsamen
        pwmLeft = pwmRight = 20;
        slowdown = false;
      }    
      if (slowdown) {
        if (motorLeftTicks + ticksPerRevolution  > stopTicks){
          pwmLeft = 20;   //Letzte halbe drehung verlangsamen
        }
        if (motorRightTicks + ticksPerRevolution > stopTicks){
          pwmRight = 20;  //Letzte halbe drehung verlangsamen
        }
        if (pwmLeft == 20 && pwmRight == 20) slowdown = false;
      }
      if (millis() > nextInfoTime){      
        nextInfoTime = millis() + 1000;            
        dumpOdoTicks(seconds);
        seconds++;      
      }    
      if(motorLeftTicks >= stopTicks)
      {
        pwmLeft = 0;
      }  
      if(motorRightTicks >= stopTicks)
      {
        pwmRight = 0;      
      }
      
      speedPWM(pwmLeft, pwmRight, 0);
      sense();
      //delay(50);         
      watchdogReset();     
      robotDriver.run();
    }
  }  
  speedPWM(0, 0, 0);
  CONSOLE.println("motor test done - please ignore any IMU/GPS errors");
  if (seconds >= 45){
    CONSOLE.println("motor.cpp Motor::test - test aborted due to timeout (>45sec) - please ignore any IMU/GPS errors");
  } else CONSOLE.println("motor.cpp Motor::test - test done - please ignore any IMU/GPS errors");
}

//Svol0 TestMowMotor
void Motor::testMow(){
  unsigned long nextMowSpeedChange = 0;
  int MowTestStep = 0;
  int RepeatCounter = 0;
  motorMowTicks = 0;
  pwmMowTest    = 0;
  int pwmMowMem = 0;
  unsigned long nextValueOut = 0; 
  
  mowTestActiv  = true; // to disable speedlimitation
  // test, if 'start/stop button' is bridged
  if (digitalRead(pinButton) == HIGH) {
    while (mowTestActiv == true){  
      // abbruch, wenn die Start-Taste während des Tests gedrückt wird
      if (MowTestStep >= 4 && MowTestStep < 10){
        if (digitalRead(pinButton) == LOW){
          nextMowSpeedChange = millis();
          MowTestStep = 10;
        }
      }
      if (MowTestStep == 12 || MowTestStep == 13){
        if (digitalRead(pinButton) == LOW){
          nextMowSpeedChange = millis();
          MowTestStep = 14;
        }
      }

      sense();          
      buzzer.run();
      watchdogReset();

      // Infos im Sekundentakt rausschreiben
      if (MowTestStep >= 6 && MowTestStep < 10){
        if (millis() > nextValueOut){
          nextValueOut += 1000;
          dumpOdoMowTicks();
        }
      } else nextValueOut = millis();
      
      switch (MowTestStep) {
        case 0: // Infos out
          CONSOLE.println("*****************************************************************************************************************************");
          CONSOLE.println("motor.cpp Motor::testMow:");
          CONSOLE.println( "TEST STARTS WITH pwmMow = 100; VALUE INCREASES EVERY 10 SECONDS BY 5 TILL 'pwmMow = 255':");
          CONSOLE.println(" ATTENTION! TO START THE MOWMOTOR KEEP THE 'START/STOP BUTTON' PRESSED FOR AT LEAST 5 SECONDS!");
          CONSOLE.println(" THE MOWMOTOR CAN BE STOPPED BY PRESSING AGAIN THE 'START/STOP BUTTON'");
          MowTestStep++;
          nextMowSpeedChange = millis();
          break;

        case 1: // Warten, dass die Starttaste für min. 5 Sek gedrückt wird.
          if (digitalRead(pinButton) == HIGH) nextMowSpeedChange = millis();
          RepeatCounter = 0;
          delay(20);
          if (millis() - nextMowSpeedChange > 5000) MowTestStep++;
          break;

        case 2: // Bestätigungston Abspielen
          nextMowSpeedChange = millis() + 1000;
          CONSOLE.println(" PLEASE RELEASE THE 'START/STOP' TO GO ON WITH THE TEST");
          buzzer.sound(SND_READY, true);
          MowTestStep++;
          break;

        case 3: // warten, dass die Taste wieder losgelassen wird
          if (millis() - nextMowSpeedChange > 1000){
            if (digitalRead(pinButton) == HIGH){
              CONSOLE.println(" ATTENTION! MOWMOTOR WILL START SPINN UP IN LESS THAN 10 SECONDS!");
              MowTestStep++;
            }
          }
          break;
          
        case 4: // Warnton laden
          if (RepeatCounter <= 3){
            nextMowSpeedChange = millis() + 2000;
            buzzer.sound(SND_ERROR, true);
          } else {
            nextMowSpeedChange = millis() + 250;
            buzzer.sound(SND_READY, true);
          }
          RepeatCounter++;
          MowTestStep++;
          break;

        case 5: // Warnton abspielen
          if (millis() > nextMowSpeedChange){
            if (RepeatCounter <= 12) MowTestStep--;
            else {
              CONSOLE.println(" ATTENTION! MOWMOTOR IS SPINNING UP!");
              MowTestStep++;
            }
          }          
          break;

        case 6: // spin up mow motor
          if (millis() > nextMowSpeedChange) MowTestStep++;
          break;

        case 7: // spin up mow motor
          if (pwmMowTest < 100) {
            nextMowSpeedChange = millis() + 50; // erhöhung alle 50ms
            pwmMowTest += 1;
            speedPWM(0, 0, pwmMowTest);
            MowTestStep--;
          } else {
              CONSOLE.println(" MOWMOTOR SPINN UP COMPLETED. PWM-VALUE WILL INCREASE EVERY 10 SECOUNDS BY 5 TILL pwmMow = 255");    
              nextMowSpeedChange = millis() + 10000;
              MowTestStep++; // fertig hochgelaufen
          }
          break;

        case 8: // Alle 10 Sekunden wird der Mähmotor pwm-Wert um 5 erhöht
          buzzer.sound(SND_READY, true);
          pwmMowTest = pwmMowTest + 5;
          if (pwmMowTest > 255){
            pwmMowTest = 255;
            MowTestStep = 10;
          } else {
            speedPWM(0, 0, pwmMowTest);
            MowTestStep++;
          }
          break;

        case 9:
          if (millis() > nextMowSpeedChange){
            nextMowSpeedChange = millis() + 10000;
            MowTestStep--;
          }
          break;

        case 10:  // reduziere die Geschwindigkeit
          pwmMowMem = pwmMowTest;  // store last pwm-value
          CONSOLE.println(" MOWMOTOR IS SLOWING DOWN.");
          MowTestStep++;
          break;

        case 11:
          if (pwmMowTest > 0){
            if (millis() > nextMowSpeedChange){
              pwmMowTest = pwmMowTest - 1;
              speedPWM(0, 0, pwmMowTest);
              nextMowSpeedChange = millis() + 10;
            }
          } else {
            nextMowSpeedChange = millis();
            RepeatCounter = 0;
            MowTestStep++;
          }
          break;

        case 12: // Info wiederholt ausgeben
          CONSOLE.println(" YOU CAN ABORT THE DELAY OF 120 SEC BY PRESSING THE START/STOP BUTTON");
          nextMowSpeedChange = millis() + 5000;
          buzzer.sound(SND_READY, true);
          RepeatCounter++;
          MowTestStep++;
          break;

        case 13: // Warten
          if (millis() > nextMowSpeedChange){
            if (RepeatCounter <= 24) MowTestStep--;
            else {
              MowTestStep++;
            }
          }          
          break;

        case 14:
          CONSOLE.println(" MOWMOTOR-TEST DONE - please ignore any IMU/GPS errors.");
          CONSOLE.print(" LAST PWM VALUE BEFORE STOP WAS: ");
          CONSOLE.println(pwmMowMem);
          CONSOLE.println("*****************************************************************************************************************************");
          pwmMowTest = MIN_MOW_RPM;
          MowTestStep++;
          nextMowSpeedChange = millis() + 5000;
          break;

        case 15:
          if (millis() > nextMowSpeedChange){
            mowTestActiv  = false; // enable speedlimitation
          }
          break;
  
      }    
    } // while (mowTestActiv == true)
  }
  else {
    speedPWM(0, 0, 0);
    mowTestActiv  = false; // enable speedlimitation
    pwmMowTest = MIN_MOW_RPM;
    CONSOLE.println("motor.cpp Motor::testMow: START/STOP BUTTON SEEMS TO BE BRIDGED. END OF TEST");
    CONSOLE.println("motor.cpp Motor::testMow: please ignore any IMU/GPS errors");
    CONSOLE.println("*****************************************************************************************************************************");
    delay(4000);
  }
  
} 

void Motor::plot(){
  CONSOLE.println("motor plot - NOTE: Start Arduino IDE Tools->Serial Plotter (CTRL+SHIFT+L)");
  delay(5000);
  motorLeftTicks = 0;  
  motorRightTicks = 0;  
  int pwmLeft = 0;
  int pwmRight = 0; 
  bool forward = true;
  unsigned long nextPlotTime = 0;
  unsigned long stopTime = millis() + 30 * 1000;

  while (millis() < stopTime){   // 30 seconds...
    int ticksLeft=0;
    int ticksRight=0;
    int ticksMow=0;
    motorDriver.getMotorEncoderTicks(ticksLeft, ticksRight, ticksMow);  
    motorLeftTicks += ticksLeft;
    motorRightTicks += ticksRight;

    if (millis() > nextPlotTime){ 
      nextPlotTime = millis() + 100;
      CONSOLE.print(pwmLeft);
      CONSOLE.print(",");  
      CONSOLE.print(pwmRight);
      CONSOLE.print(",");
      CONSOLE.print(motorLeftTicks);    
      CONSOLE.print(",");
      CONSOLE.print(motorRightTicks);
      CONSOLE.println();
      motorLeftTicks = 0;
      motorRightTicks = 0;      
    }

    speedPWM(pwmLeft, pwmRight, 0);
    if (pwmLeft >= 255){
      forward = false; 
    }      
    if (pwmLeft <= -255){
      forward = true; 
    }          
    if (forward){
      pwmLeft++;
      pwmRight++;            
    } else {
      pwmLeft--;
      pwmRight--;
    }
    //sense();
    //delay(10);
    watchdogReset();     
    robotDriver.run();   
  }
  speedPWM(0, 0, 0);
  CONSOLE.println("motor plot done - please ignore any IMU/GPS errors");
}
