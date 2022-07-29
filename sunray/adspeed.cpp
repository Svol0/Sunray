// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#include "adspeed.h"
#include "motor.h"
#include "config.h"
#include "helper.h"
#include "robot.h"
#include "Arduino.h"
#include <RunningMedian.h>
#ifdef MOWMOTOR_CURRENT_MEDIAN_LEN
  RunningMedian RM_motorMowSense = RunningMedian(MOWMOTOR_CURRENT_MEDIAN_LEN);
#else
  RunningMedian RM_motorMowSense = RunningMedian(MowMotorCurrentMedLen);
#endif

RunningMedian RM_mowCurrNoLoad_MinSpeed = RunningMedian(9);   // median mowcurrent by no load at min speed
RunningMedian RM_mowCurrNoLoad_MaxSpeed = RunningMedian(9);   // median mowcurrent by no load at max speed
RunningMedian RM_mowCurrLoadDetection = RunningMedian(5);   // median mowcurrent by no load at min speed

unsigned long updateTimer = 0;

float AS_SpeedOffset = 0;
int   AS_pwmMowOut = 0;
unsigned long AS_lastControlTime = 0;
unsigned long AS_lastCall = 0;
unsigned long AS_millisDiv = 0;
bool AS_DEBUGMODE = false;

int AS_iAdaptiveSpeedAlgorithm = 0;
float AS_fSpeedFactorMin = 0;
float AS_fSpeedFactorMax = 0;
float AS_fSpeedDownCurrent = 0;
float AS_fSpeedUpCurrent = 0;
float AS_fCurrentFactorHighLoad = 0;
float AS_fCurrentFactorMiddleLoad = 0;
int AS_iMinMowRpm  = 0;
int AS_iMaxMowRpm  = 0;

void AdaptiveSpeed::begin() {
  
  CONSOLE.println("ADAPTIVESPEED is activated in config.h - type AT+AS? to get more information");

AS_mowCurrNoLoad_MinSpeedMed = 0;
AS_mowCurrNoLoad_MaxSpeedMed = 0;
AS_mowCurrForUpdate = 0;
AS_mowReversTimer = 0;
AS_mowReversTrg  = 0;
AS_mowCurrUpdateTrg  = false;
AS_mowSpeedInfoTrg = false;
AS_mowDriveRevers = false;
AS_mowMotorIsOn  = false;
AS_motorMowSense = 0;
AS_motorMowSenseMed = 0;
AS_pwmMow = 0;
AS_speedStep = 0;
AS_mowCurrUpdate = 0;
AS_linear = 0;
AS_angular = 0;

// intern variables for control
AS_iAdaptiveSpeedAlgorithm = ADAPTIVE_SPEED_ALGORITHM;
AS_fSpeedFactorMin    = SPEED_FACTOR_MIN;
AS_fSpeedFactorMax    = SPEED_FACTOR_MAX;
AS_fSpeedDownCurrent  = SPEEDDOWNCURRENT;
AS_fSpeedUpCurrent    = SPEEDUPCURRENT;
AS_fCurrentFactorHighLoad   = CURRENT_FACTOR_HIGH_LOAD;
AS_fCurrentFactorMiddleLoad = CURRENT_FACTOR_MIDDLE_LOAD;
AS_iMinMowRpm         = MIN_MOW_RPM;
AS_iMaxMowRpm         = MAX_MOW_RPM;


//statistic
AS_maxMowSense  = 0;                // maximum measured mowmotor current at stable RPM
AS_maxMowSenseSpeedChange  = 0;     // maximum measured mowmotor current during speed change
AS_MowSenseAtIdleMIN_MOW_RPM = 0;   // mowmotor current at MIN_MOW_RPM in idle
AS_MowSenseAtIdleMAX_MOW_RPM = 0;   // mowmotor current at MAX_MOW_RPM in idle
AS_MowDurationAtMIN_MOW_RPM = 0;
AS_MowDurationAtMID_MOW_RPM = 0;
AS_MowDurationAtMAX_MOW_RPM = 0;
AS_MowReversActionCounter = 0;
AS_lastCall = millis();
AS_millisDiv  = 0;

AS_Timer = 0;
RM_mowCurrNoLoad_MinSpeed.add(AS_fSpeedUpCurrent); //Puts Values of motorMowSense into median function
RM_mowCurrNoLoad_MaxSpeed.add(AS_fSpeedUpCurrent); //Puts Values of motorMowSense into median function
AS_mowCurrNoLoad_MaxSpeedMed = RM_mowCurrNoLoad_MaxSpeed.getMedian(); //Get the Running Median as motorMowSenseMed
AS_mowCurrNoLoad_MinSpeedMed = RM_mowCurrNoLoad_MinSpeed.getMedian(); //Get the Running Median as motorMowSenseMed

}

void AdaptiveSpeed::getActDriveSpeedValue(float linear, float angular){
  AS_linear = linear;     // lese aktuelle Fahrgeschwindigkeit
  AS_angular = angular;   // lese aktuelle Drehgeschwindigkeit
}

void AdaptiveSpeed::getActMowSpeedValue(int pwmMow){
  AS_pwmMow = abs(pwmMow);
}

void AdaptiveSpeed::getMowIsOn(bool mowMotorIsOn){
  AS_mowMotorIsOn = mowMotorIsOn;
}

/*
void AdaptivSpeed::getActMowSenseValue(float motorMowSense, float motorMowSenseMed){
  AS_motorMowSense = motorMowSense;
  AS_motorMowSenseMed = motorMowSenseMed;
}
*/

float AdaptiveSpeed::setSpeedOffset(){
  return AS_SpeedOffset;
}

int AdaptiveSpeed::setPwmMow(){
  return AS_pwmMowOut;
}

void AdaptiveSpeed::debugOut(){
  AS_DEBUGMODE = AS_DEBUGMODE xor true;
  if (AS_DEBUGMODE) CONSOLE.println("ADAPTIVE_SPEED - Debug mode switched on!");
  else CONSOLE.println("ADAPTIVE_SPEED - Debug mode switched off!");
}

void AdaptiveSpeed::run() {
//  if ((AS_pwmMow != 0) && (ADAPTIVE_SPEED)) {
if (millis() < AS_lastControlTime + 50) return;
AS_lastControlTime = millis();

if (millis() > updateTimer){
  updateTimer = millis() + 5000;
/*  
CONSOLE.print("ADAPTIV_SPEED AS_linear: ");
CONSOLE.print(AS_linear);
CONSOLE.print(" AS_pwmMow: ");
CONSOLE.print(AS_pwmMow);
CONSOLE.print(" AS_mowMotorIsOn: ");
CONSOLE.print(AS_mowMotorIsOn);
CONSOLE.print(" AS_motorMowSense: ");
CONSOLE.print(AS_motorMowSense);
CONSOLE.print("AS_motorMowSenseMed: ");
CONSOLE.print(AS_motorMowSenseMed);
CONSOLE.print(" AS_SpeedOffset: ");
CONSOLE.print(AS_SpeedOffset);
CONSOLE.print(" AS_pwmMowOut: ");
CONSOLE.println(AS_pwmMowOut);
*/
}

  RM_motorMowSense.add(motor.motorMowSense); //Puts Values of motorMowSense into median function
  AS_motorMowSenseMed = RM_motorMowSense.getMedian(); //Get the Running Median as motorMowSenseMed
  AS_motorMowSense = motor.motorMowSense;
  RM_mowCurrLoadDetection.add(motor.motorMowSense); //Puts Values of motorMowSense into median function
  AS_mowCurrForUpdate = RM_mowCurrLoadDetection.getMedian();
  
    AS_millisDiv = millis() - AS_lastCall;
    AS_lastCall = millis();
    
    switch (AS_iAdaptiveSpeedAlgorithm) {
      //Simple 2 point controller that applies a linear ramp to mowerspeed through a offset.
      //The delta of SPEEDDOWNCURRENT-SPEEDUPCURRENT is the hysteresis
      case 1:
        if (AS_motorMowSenseMed > AS_fSpeedDownCurrent){
         if (millis() > motor.motorMowSpinUpTime + MOW_SPINUPTIME){ //avoid trigger by speed up
            AS_Timer = millis();
            AS_SpeedOffset = AS_fSpeedFactorMin;
            //if (pwmMaxMow < MAX_MOW_RPM) 
            if (AS_pwmMowOut == AS_iMinMowRpm){
              if (AS_DEBUGMODE) CONSOLE.println("ADAPTIVE_SPEED - use high load setting");
              if (AS_DEBUGMODE) CONSOLE.print("AS_motorMowSenseMed: ");
              if (AS_DEBUGMODE) CONSOLE.print(AS_motorMowSenseMed);
              if (AS_DEBUGMODE) CONSOLE.println("A");
            }
            AS_pwmMowOut = AS_iMaxMowRpm;
          }
        }
        if (AS_motorMowSenseMed < AS_fSpeedUpCurrent){
          if ((millis() - AS_Timer) > 10000){
            AS_SpeedOffset = AS_fSpeedFactorMax;        
            // if (pwmMaxMow > MIN_MOW_RPM) 
            if (AS_pwmMowOut == AS_iMaxMowRpm){
              if (AS_DEBUGMODE) CONSOLE.println("ADAPTIVE_SPEED - use low load setting");
              if (AS_DEBUGMODE) CONSOLE.print("AS_motorMowSenseMed: ");
              if (AS_DEBUGMODE) CONSOLE.print(AS_motorMowSenseMed);
              if (AS_DEBUGMODE) CONSOLE.println("A");
            }
            AS_pwmMowOut = AS_iMinMowRpm;
          }
        }
        if (AS_mowMotorIsOn && (AS_pwmMowOut == AS_iMinMowRpm)) AS_MowDurationAtMIN_MOW_RPM += AS_millisDiv;
        if (AS_mowMotorIsOn && (AS_pwmMowOut == AS_iMaxMowRpm)) AS_MowDurationAtMAX_MOW_RPM += AS_millisDiv;
        break;


      // 3 point controller with automated idle current measurement to set the trigger points for mowmotor speedup and slowdown drive speed
      case 2:

        // reset speed steps if mowmotor is of
        if (!AS_mowMotorIsOn && (AS_speedStep != 0)){
          if (AS_DEBUGMODE) CONSOLE.println("ADAPTIVE_SPEED - mow motor was switched off!");
          AS_speedStep = 0;
        }

        switch (AS_speedStep){
          case 0: // init
            AS_SpeedOffset = AS_fSpeedFactorMax;
            AS_pwmMowOut   = AS_iMinMowRpm;
            if (AS_mowMotorIsOn && (millis() > motor.motorMowSpinUpTime + MOW_SPINUPTIME)){ //avoid trigger at speed up
              if (abs(AS_pwmMow - AS_pwmMowOut) <= 3){  // wait till setspeed is reached
                if (AS_DEBUGMODE) CONSOLE.println("ADAPTIVE_SPEED - mow motor speed up finished!");
                AS_speedStep = 10; // start
              }
            }
            break;
            
          case 10: // drive at high speed with low mow speed
            AS_SpeedOffset = AS_fSpeedFactorMax;
            AS_pwmMowOut   = AS_iMinMowRpm;
            AS_MowDurationAtMIN_MOW_RPM += AS_millisDiv;
            // wenn der gemittelte motorstrom steigt und der aktuelle motorstrom deutlich über dem idle liegt, soll es gleich mit Einstellung für hohe Last 
//            if ((AS_motorMowSenseMed > (AS_mowCurrNoLoad_MinSpeedMed * 1.2)) && (AS_motorMowSense > (AS_mowCurrNoLoad_MinSpeedMed * AS_fCurrentFactorHighLoad))){
            if ((AS_mowCurrForUpdate > (AS_mowCurrNoLoad_MinSpeedMed * AS_fCurrentFactorHighLoad))){
              if ((millis() - AS_Timer) > 250){ // if mow current is continously low, go speed step up
                AS_Timer = millis();  // reset timer
                if ((!AS_mowReversTrg) && (lastTargetDist > 0.05)){  // wenn der mäher mind. 5cm vom letzten Ziel entfernt ist, darf er bei hoher Last kurz zurücksetzen
    
                  #if USE_LINEAR_SPEED_RAMP
                    // calculation for stop way
                    float mowerActualSpeed = fabs(AS_linear);
                    if (mowerActualSpeed == 0) mowerActualSpeed = 0.1;
                
                    float mowerDecRamp = (DEC_RAMP/MOTOR_MAX_SPEED)*mowerActualSpeed;         // Anteil Bremsrampe von aktueller Geschwindigkeit ermitteln
                    float mowerWayTillStop = (mowerActualSpeed * (mowerDecRamp / 1000)) / 2;  // zurückgelegter Weg bis zum stop.
                
                /*    
                    CONSOLE.print("---------------------");
                    CONSOLE.print("EscapeReversOP::begin mowerWayTillStop: ");
                    CONSOLE.print(mowerWayTillStop);
                    CONSOLE.print(" | mowerDecRamp:");
                    CONSOLE.print(mowerDecRamp);
                    CONSOLE.print(" | mowerActualSpeed:");
                    CONSOLE.print(mowerActualSpeed);
                    CONSOLE.print(" | AS_linear:");
                    CONSOLE.println(AS_linear);
                */
                    
                    float mowerObstacleAvoidanceSpeed = OBSTACLEAVOIDANCESPEED;
                    if ((mowerObstacleAvoidanceSpeed == 0) || (mowerObstacleAvoidanceSpeed > MOTOR_MAX_SPEED)) mowerObstacleAvoidanceSpeed = 0.1;
                    float mowerAccRamp = (ACC_RAMP/MOTOR_MAX_SPEED)*mowerObstacleAvoidanceSpeed;  // Beschleunigungszeit bis Sollgeschwindigkeit berechnen
                          mowerDecRamp = (DEC_RAMP/MOTOR_MAX_SPEED)*mowerObstacleAvoidanceSpeed;  // Verzögerungszeit von Sollgeschwindigkeit bis stop berechnen
                    if (mowerAccRamp == 0) mowerAccRamp = 1;
                    if (mowerDecRamp == 0) mowerDecRamp = 1;
                    // Wegstrecken die bis zur Endgeschwindigkeit zurückgelegt werden
                    float wayAccRamp  = (mowerObstacleAvoidanceSpeed * (mowerAccRamp/1000)) / 2;
                    float wayDecRamp  = (mowerObstacleAvoidanceSpeed * (mowerDecRamp/1000)) / 2;    
                
                    float mowerObstacleAvoidanceWay = 0.4;  // es sollen 40cm zurückgefahren werden
                    mowerObstacleAvoidanceWay = mowerObstacleAvoidanceWay + mowerWayTillStop; // Den Anhalteweg hinzufügen 
                    if (mowerObstacleAvoidanceWay > lastTargetDist) mowerObstacleAvoidanceWay = lastTargetDist; // Wenn die Entfernung zum letzten Wegpunkt kleiner als die Strecke zur Reversieren ist, wird nur bis zum Wegpunkt reversiert
                    float sBeschl = (mowerObstacleAvoidanceWay / (mowerAccRamp + mowerDecRamp))*mowerAccRamp;    // calculate the part of the reversway by acceleration
                    float sVerz   = (mowerObstacleAvoidanceWay / (mowerAccRamp + mowerDecRamp))*mowerDecRamp;    // calculate the part of the reversway by acceleration
                    float aBeschl = (mowerObstacleAvoidanceSpeed * 1000) / mowerAccRamp;              // calculate acceleration by obstacle avoidance speed and ramp
                    float aVerz   = (mowerObstacleAvoidanceSpeed * 1000) / mowerDecRamp;              // calculate deceleration by actual speed and ramp
                    float tBeschl = 0;
                    float tVerz = 0;
                    float sKonstBeschl  = 0;
                    float sKonstVerz  = 0;
                    float tOffset = 0;
                
                    if (wayAccRamp < sBeschl){
                      sKonstBeschl = sBeschl - wayAccRamp;
                      tBeschl = mowerAccRamp;
                //      tOffset = (0.1 / mowerObstacleAvoidanceSpeed)*1000; // add 10cm more (result by testing)
                    } else tBeschl  = (sqrt((2*sBeschl)/aBeschl) *1000);                        // calculate time for reverse action    
                
                    if (wayDecRamp < sVerz){
                      sKonstVerz = sVerz - wayDecRamp;
                      tVerz = mowerDecRamp;
                      //tOffset = (0.1 / mowerActualSpeed)*1000; // add 10cm more (result by testing)
                    } else tVerz  = (sqrt((2*sVerz)/aVerz) *1000);                        // calculate time for reverse action
                      
                    float tKonst  = ((sKonstBeschl + sKonstVerz) / mowerObstacleAvoidanceSpeed)*1000;
/*                    
                    CONSOLE.print("EscapeReversOP::begin aBeschl:");
                    CONSOLE.print(aBeschl);
                    CONSOLE.print(" | sBeschl:");
                    CONSOLE.print(sBeschl);
                    CONSOLE.print(" | tBeschl:");
                    CONSOLE.print(tBeschl);
                    CONSOLE.print(" | mowerAccRamp:");
                    CONSOLE.print(mowerAccRamp);
                    CONSOLE.print(" | mowerObstacleAvoidanceSpeed:");
                    CONSOLE.print(mowerObstacleAvoidanceSpeed);
                    CONSOLE.print(" | wayAccRamp:");
                    CONSOLE.println(wayAccRamp);
                    CONSOLE.print("EscapeReversOP::begin aVerz:");
                    CONSOLE.print(aVerz);
                    CONSOLE.print(" | sVerz:");
                    CONSOLE.print(sVerz);
                    CONSOLE.print(" | tVerz:");
                    CONSOLE.print(tVerz);
                    CONSOLE.print(" | mowerDecRamp:");
                    CONSOLE.print(mowerDecRamp);
                    CONSOLE.print(" | mowerActualSpeed:");
                    CONSOLE.print(mowerActualSpeed);
                    CONSOLE.print(" | wayDecRamp:");
                    CONSOLE.println(wayDecRamp);
                    CONSOLE.print(" | sKonstBeschl:");
                    CONSOLE.print(sKonstBeschl);
                    CONSOLE.print(" | sKonstVerz:");
                    CONSOLE.print(sKonstVerz);
                    CONSOLE.print(" | tKonst:");
                    CONSOLE.println(tKonst);
*/                    
                    AS_mowReversTimer = tVerz + tBeschl + tKonst + tOffset;  // calculated time for reverse action
                  #else
                    AS_mowReversTimer = mowerObstacleAvoidanceWay * 100000 / (mowerObstacleAvoidanceSpeed * 100);
                  #endif
     
                  AS_mowDriveRevers = true;   // reverse line tracking needs negative speed
                  AS_MowReversActionCounter++;
                  if (AS_DEBUGMODE) CONSOLE.println("ADAPTIVE_SPEED - mow motor load high enough for reverse");
                  // resetLinearMotionMeasurement();
                }
                AS_mowReversTrg = true;
                if (AS_DEBUGMODE) CONSOLE.println("ADAPTIVE_SPEED - use high load setting");
                if (AS_DEBUGMODE) CONSOLE.print("AS_mowCurrForUpdate: ");
                if (AS_DEBUGMODE) CONSOLE.print(AS_mowCurrForUpdate);
                if (AS_DEBUGMODE) CONSOLE.print(" AS_motorMowSense: ");
                if (AS_DEBUGMODE) CONSOLE.println(AS_motorMowSense);
                AS_speedStep = 30; // go to high load
              }
//            } else if (AS_motorMowSenseMed > (AS_mowCurrNoLoad_MinSpeedMed * AS_fCurrentFactorMiddleLoad)){
            } else if (AS_mowCurrForUpdate > (AS_mowCurrNoLoad_MinSpeedMed * AS_fCurrentFactorMiddleLoad)){
              if ((millis() - AS_Timer) > 1000){ // if mow current is continously low, go speed step up
                AS_Timer = millis();  // reset timer
                if (AS_DEBUGMODE) CONSOLE.println("ADAPTIVE_SPEED - use middle load setting");
                if (AS_DEBUGMODE) CONSOLE.print("AS_mowCurrForUpdate: ");
                if (AS_DEBUGMODE) CONSOLE.print(AS_mowCurrForUpdate);
                if (AS_DEBUGMODE) CONSOLE.print(" AS_motorMowSense: ");
                if (AS_DEBUGMODE) CONSOLE.println(AS_motorMowSense);
                AS_speedStep = 20; // go to middel load
              }
            } else AS_Timer = millis();  // reset timer
            if (AS_linear < (MOTOR_MIN_SPEED/2)) AS_Timer = millis();  // reset timer if mower stops or is turning on waypoints
            break;

          case 20: // drive at middle speed with middle mow speed
            AS_SpeedOffset = ((AS_fSpeedFactorMin + AS_fSpeedFactorMax) / 2);
            AS_pwmMowOut   = ((AS_iMinMowRpm + AS_iMaxMowRpm) / 2);
            AS_MowDurationAtMID_MOW_RPM += AS_millisDiv;
//            if (AS_motorMowSenseMed < (((AS_mowCurrNoLoad_MinSpeedMed + AS_mowCurrNoLoad_MaxSpeedMed) / 2) * 1.3)){
            if (AS_mowCurrForUpdate < (((AS_mowCurrNoLoad_MinSpeedMed + AS_mowCurrNoLoad_MaxSpeedMed) / 2) * (AS_fCurrentFactorMiddleLoad * 0.9))){
              if ((millis() - AS_Timer) > 5000){ // if mow current is continously low, go speed step up
                AS_Timer = millis();  // reset timer
                AS_mowReversTrg = false;
                if (AS_DEBUGMODE) CONSOLE.println("ADAPTIV_SPEED - use low load setting");
                if (AS_DEBUGMODE) CONSOLE.print("AS_mowCurrForUpdate: ");
                if (AS_DEBUGMODE) CONSOLE.print(AS_mowCurrForUpdate);
                if (AS_DEBUGMODE) CONSOLE.print(" AS_motorMowSense: ");
                if (AS_DEBUGMODE) CONSOLE.println(AS_motorMowSense);                
                AS_speedStep = 10; // go to low load
              }
//            } else if ((AS_motorMowSenseMed > (AS_mowCurrNoLoad_MinSpeedMed * 1.4)) && (AS_motorMowSense > (AS_mowCurrNoLoad_MinSpeedMed * AS_fCurrentFactorHighLoad))){
            } else if ((AS_mowCurrForUpdate > (AS_mowCurrNoLoad_MinSpeedMed * AS_fCurrentFactorHighLoad))){
              if ((millis() - AS_Timer) > 1000){ // if mow current is continously high, go speed step down
                AS_Timer = millis();  // reset timer
                if (AS_DEBUGMODE) CONSOLE.println("ADAPTIVE_SPEED - use high load setting");
                if (AS_DEBUGMODE) CONSOLE.print("AS_mowCurrForUpdate: ");
                if (AS_DEBUGMODE) CONSOLE.print(AS_mowCurrForUpdate);
                if (AS_DEBUGMODE) CONSOLE.print(" AS_motorMowSense: ");
                if (AS_DEBUGMODE) CONSOLE.println(AS_motorMowSense);
                AS_speedStep = 30; // go to high load
              }              
            } else AS_Timer = millis();  // reset timer
            if (AS_linear < (MOTOR_MIN_SPEED/2)) AS_Timer = millis();  // reset timer if mower stops or is turning on waypoints
            break;
            
          case 30: // drive at low speed with high mow speed
            AS_SpeedOffset = AS_fSpeedFactorMin;
            AS_pwmMowOut   = AS_iMaxMowRpm;
            AS_MowDurationAtMAX_MOW_RPM += AS_millisDiv;
            //if (AS_mowCurrForUpdate < (AS_mowCurrNoLoad_MaxSpeedMed * 1.3)){
            if (AS_mowCurrForUpdate < (AS_mowCurrNoLoad_MaxSpeedMed * (AS_fCurrentFactorMiddleLoad * 0.9))){
              if ((millis() - AS_Timer) > 10000){ // if mow current is continously low, go speed step up
                AS_Timer = millis();  // reset timer
                if (AS_DEBUGMODE) CONSOLE.println("ADAPTIVE_SPEED - use middle load setting");
                if (AS_DEBUGMODE) CONSOLE.print("AS_mowCurrForUpdate: ");
                if (AS_DEBUGMODE) CONSOLE.print(AS_mowCurrForUpdate);
                if (AS_DEBUGMODE) CONSOLE.print(" AS_motorMowSense: ");
                if (AS_DEBUGMODE) CONSOLE.println(AS_motorMowSense);
                AS_speedStep = 20; // go to middle load  
              }
            } else AS_Timer = millis();  // reset timer
            if (AS_linear < (MOTOR_MIN_SPEED/2)) AS_Timer = millis();  // reset timer if mower stops or is turning on waypoints
            break;

          default:
              if (AS_DEBUGMODE) CONSOLE.print("ADAPTIVE_SPEED - wrong AS_speedStep:");
              if (AS_DEBUGMODE) CONSOLE.println(AS_speedStep);
            break;
            
        }

        switch (AS_mowCurrUpdate){
          case 0: // wait for linearSpeedSet = 0 (no linear movement of the robot, but mow motor is turning and has reached the setspeed)
            if ((AS_linear < (MOTOR_MIN_SPEED/2)) && AS_mowMotorIsOn && (AS_speedStep > 0)){
              if (AS_mowDriveRevers) AS_mowCurrUpdate = 99;
              else AS_mowCurrUpdate = 1;
            }
            /*
                  CONSOLE.print(AS_linear);
                  CONSOLE.print("!!!AS_linear: ");
                  CONSOLE.print(AS_linear);                
                  CONSOLE.print(" AS_mowMotorIsOn:");
                  CONSOLE.print(AS_mowMotorIsOn);
                  CONSOLE.print(" AS_mowCurrUpdate: ");
                  CONSOLE.print(AS_mowCurrUpdate);
                  CONSOLE.println("");
*/
            break;

          case 1: // wait for linearSpeedSet
            if (AS_linear > (MOTOR_MIN_SPEED/2)){
              if (AS_mowMotorIsOn){
                if ((AS_pwmMowOut == AS_iMaxMowRpm) && (abs(AS_pwmMow - AS_iMaxMowRpm) <= 3)){ 
                  RM_mowCurrNoLoad_MaxSpeed.add(AS_mowCurrForUpdate); //Puts Values of motorMowSense into median function
                  if (AS_DEBUGMODE) CONSOLE.print("ADAPTIVE_SPEED - update idle mow current at MAX_MOW_RPM: ");
                } else if ((AS_pwmMowOut == AS_iMinMowRpm) && (abs(AS_pwmMow - AS_iMinMowRpm) <= 3)){
                  RM_mowCurrNoLoad_MinSpeed.add(AS_mowCurrForUpdate); //Puts Values of motorMowSense into median function
                  if (AS_DEBUGMODE) CONSOLE.print("ADAPTIVE_SPEED - update idle mow current at MIN_MOW_RPM: ");
                } else {
                  if (AS_DEBUGMODE) CONSOLE.print("ADAPTIVE_SPEED - Mow Speed out of range: ");
                  if (AS_DEBUGMODE) CONSOLE.print(AS_pwmMow);
                  if (AS_DEBUGMODE) CONSOLE.print(" AS_motorMowSense: ");
                }
                AS_mowCurrNoLoad_MaxSpeedMed = RM_mowCurrNoLoad_MaxSpeed.getMedian(); //Get the Running Median as motorMowSenseMed
                AS_mowCurrNoLoad_MinSpeedMed = RM_mowCurrNoLoad_MinSpeed.getMedian(); //Get the Running Median as motorMowSenseMed
                AS_MowSenseAtIdleMIN_MOW_RPM = AS_mowCurrNoLoad_MinSpeedMed;
                AS_MowSenseAtIdleMAX_MOW_RPM = AS_mowCurrNoLoad_MaxSpeedMed;
                if (AS_DEBUGMODE){
                  CONSOLE.print(AS_motorMowSense);
                  CONSOLE.print("A mowCurrForUpdate: ");
                  CONSOLE.print(AS_mowCurrForUpdate);                
                  CONSOLE.print("A");
                  CONSOLE.print(" AS_mowCurrMinMed: ");
                  CONSOLE.print(AS_mowCurrNoLoad_MinSpeedMed);
                  CONSOLE.print(" AS_mowCurrMaxMed: ");
                  CONSOLE.print(AS_mowCurrNoLoad_MaxSpeedMed);
                  CONSOLE.println("A");
                }
              }
              AS_mowCurrUpdate = 0;
            }
            break;  
          case 99:  // wait for normal speed
            if ((AS_linear > (MOTOR_MIN_SPEED/2)) && !AS_mowMotorIsOn){
              if (AS_DEBUGMODE) CONSOLE.println("ADAPTIV_SPEED - no idle current update cause of reverse drive!");
              AS_mowCurrUpdate = 0;
            }
            break;  
        } // switch (AS_mowCurrUpdate)

        // drive short reverse on high mowmotor load
        if (AS_mowReversTimer > 0){
          resetLinearMotionMeasurement();
          if (AS_mowReversTimer > AS_millisDiv ){
            AS_mowReversTimer -= AS_millisDiv;             
          } else {
            AS_mowReversTimer = 0;
            AS_mowDriveRevers = false;   // reset reverse driving    
          }
/*          
          CONSOLE.print(" should Drive reverse  AS_mowDriveRevers:");
          CONSOLE.print(AS_mowDriveRevers);
          CONSOLE.print(" AS_mowReversTimer:");
          CONSOLE.print(AS_mowReversTimer);
          CONSOLE.print(" AS_linear:");
          CONSOLE.println(AS_linear);
*/
        }
        
        break;
      case 3:
      //empty  
        break;
      default:
      //empty  
        break;
    }

    // log maximum mowmotor current if set speed is reached
    if (((AS_pwmMowOut == AS_iMinMowRpm) && (abs(AS_pwmMow - AS_iMinMowRpm) <= 2)) || 
      ((AS_pwmMowOut == ((AS_iMinMowRpm + AS_iMaxMowRpm)/2)) && (abs(AS_pwmMow - ((AS_iMinMowRpm + AS_iMaxMowRpm)/2)) <= 2)) || 
      ((AS_pwmMowOut == AS_iMaxMowRpm) && (abs(AS_pwmMow - AS_iMaxMowRpm) <= 2))){
      if (AS_motorMowSenseMed > AS_maxMowSense) AS_maxMowSense = AS_motorMowSenseMed;
    } else if (AS_motorMowSenseMed > AS_maxMowSenseSpeedChange) AS_maxMowSenseSpeedChange = AS_motorMowSenseMed;  // update AS_maxMowSenseSpeedChange during speed changes
    

    AS_SpeedOffset = min(AS_fSpeedFactorMax, max(AS_fSpeedFactorMin, AS_SpeedOffset));
//  } else if ((AS_pwmMow = 0) && (ADAPTIVE_SPEED)) AS_SpeedOffset = SPEED_FACTOR_MAX;  
}

void AdaptiveSpeed::statisticsOutput(){
  CONSOLE.println("------------------------------------------------------");
  CONSOLE.println("ADAPTIVE_SPEED STATISTIC OUTPUT:");
  CONSOLE.print(" Mowtime with MIN_MOW_RPM : ");
  CONSOLE.print((AS_MowDurationAtMIN_MOW_RPM / 60000));
  CONSOLE.println("min");
  if (AS_iAdaptiveSpeedAlgorithm == 2){
    CONSOLE.print(" Mowtime with middel RPM  : ");
    CONSOLE.print((AS_MowDurationAtMID_MOW_RPM / 60000));
    CONSOLE.println("min");
  }
  CONSOLE.print(" Mowtime with MAX_MOW_RPM : ");
  CONSOLE.print((AS_MowDurationAtMAX_MOW_RPM / 60000));
  CONSOLE.println("min");
  CONSOLE.print(" Maximum mow motor current at stable speed: ");
  CONSOLE.print(AS_maxMowSense);
  CONSOLE.println("A");
  CONSOLE.print(" Maximum mow motor current at speed change: ");
  CONSOLE.print(AS_maxMowSenseSpeedChange);
  CONSOLE.println("A");
  if (AS_iAdaptiveSpeedAlgorithm == 2){  
    CONSOLE.print(" Mow motor idle current at MIN_MOW_RPM: ");
    CONSOLE.print(AS_MowSenseAtIdleMIN_MOW_RPM);
    CONSOLE.println("A");
    CONSOLE.print(" Mow motor idle current at MAX_MOW_RPM: ");
    CONSOLE.print(AS_MowSenseAtIdleMAX_MOW_RPM);
    CONSOLE.println("A");
    CONSOLE.print(" Reverse drive caused by high mow motor load: ");
    CONSOLE.println(AS_MowReversActionCounter);
  }
  CONSOLE.println("------------------------------------------------------");
}

void AdaptiveSpeed::statisticsReset(){
  //statistic
AS_maxMowSense  = 0;                // maximum measured mowmotor current
AS_maxMowSenseSpeedChange = 0;
AS_MowSenseAtIdleMIN_MOW_RPM = 0;   // mowmotor current at MIN_MOW_RPM in idle
AS_MowSenseAtIdleMAX_MOW_RPM = 0;   // mowmotor current at MAX_MOW_RPM in idle
AS_MowDurationAtMIN_MOW_RPM = 0;
AS_MowDurationAtMID_MOW_RPM = 0;
AS_MowDurationAtMAX_MOW_RPM = 0;
AS_MowReversActionCounter = 0;
CONSOLE.println("ADAPTIVE_SPEED - History was cleared!");
}

void AdaptiveSpeed::helpOutput(){
  CONSOLE.println("------------------------------------------------------");
  CONSOLE.println("ADAPTIVE_SPEED HELP OUTPUT:");
  CONSOLE.println(" AT COMMAND: ");
  CONSOLE.println("  AT+ASC : for setting up the adaptive_speed feature it is possible to send AT command to change parameter during run");
  CONSOLE.println("           AT+ASC,mode,value1,value2,value3,value4,value5,value6,");
  CONSOLE.println("           example: AT+ASC,1,0.5,1.0,0.8,0.6,170,210,");
  CONSOLE.println("           mode:   1 = 2 step controller | 2 = 3 step controller with automatic current setpoint");
  CONSOLE.println("           value1: SPEED_FACTOR_MIN      | SPEED_FACTOR_MIN");
  CONSOLE.println("           value2: SPEED_FACTOR_MAX      | SPEED_FACTOR_MAX");
  CONSOLE.println("           value3: SPEEDDOWNCURRENT      | CURRENT_FACTOR_HIGH_LOAD");
  CONSOLE.println("           value4: SPEEDUPCURRENT        | CURRENT_FACTOR_MIDDLE_LOAD");
  CONSOLE.println("           value5: MIN_MOW_RPM           | MIN_MOW_RPM");
  CONSOLE.println("           value5: MAX_MOW_RPM           | MAX_MOW_RPM");
  CONSOLE.println("  AT+ASD : switch degub mode on /off - debug mode on will show status information in console output");
  CONSOLE.println("  AT+ASS : show statistics from adaptivespeed feature since last reset");
  CONSOLE.println("  AT+ASR : reset the statistics of the adaptivespeed feature");
  CONSOLE.println("------------------------------------------------------");  
}

void AdaptiveSpeed::getCommand(int mode, float value1, float value2, float value3, float value4, int value5 , int value6){
  if (mode != AS_iAdaptiveSpeedAlgorithm) statisticsReset();
  
  if (mode == 1) {

    AS_iAdaptiveSpeedAlgorithm = mode;
    AS_fSpeedFactorMin    = value1;
    AS_fSpeedFactorMax    = value2;
    AS_fSpeedDownCurrent  = value3;
    AS_fSpeedUpCurrent    = value4;
    if (value5 > 0 && value5 <= 255) AS_iMinMowRpm = value5;
    if (value6 > 0 && value6 <= 255) AS_iMaxMowRpm = value6;
  }
  if (mode == 2) {
    // Reset switches
    AS_mowCurrUpdate = 0;
    AS_iAdaptiveSpeedAlgorithm = mode;
    AS_fSpeedFactorMin    = value1;
    AS_fSpeedFactorMax    = value2;
//    AS_fSpeedDownCurrent  = value3;
//    AS_fSpeedUpCurrent    = value4;
    if (value5 > 0 && value5 <= 255) AS_iMinMowRpm = value5;
    if (value6 > 0 && value6 <= 255) AS_iMaxMowRpm = value6;

  }
  if (mode == 1 || mode == 2){
    CONSOLE.println("ADAPTIVESPEED use new parameter settings. Do not forget to save parameters in config.h if you find the pefect values");
    CONSOLE.print(" ADAPTIVE_SPEED_ALGORITHM: ");
    CONSOLE.println(AS_iAdaptiveSpeedAlgorithm);
    CONSOLE.print(" SPEED_FACTOR_MIN: ");
    CONSOLE.println(AS_fSpeedFactorMin);
    CONSOLE.print(" SPEED_FACTOR_MAX: ");
    CONSOLE.println(AS_fSpeedFactorMax);
    if (mode == 1){
      CONSOLE.print(" SPEEDDOWNCURRENT: ");
      CONSOLE.println(AS_fSpeedDownCurrent);
      CONSOLE.print(" SPEEDUPCURRENT: ");
      CONSOLE.println(AS_fSpeedUpCurrent);
    } else if (mode == 2){
      CONSOLE.print(" CURRENT_FACTOR_HIGH_LOAD: ");
      CONSOLE.println(AS_fCurrentFactorHighLoad);
      CONSOLE.print(" CURRENT_FACTOR_MIDDLE_LOAD: ");
      CONSOLE.println(AS_fCurrentFactorMiddleLoad);
    }
    
    CONSOLE.print(" MIN_MOW_RPM: ");
    CONSOLE.println(AS_iMinMowRpm);
    CONSOLE.print(" MAX_MOW_RPM: ");
    CONSOLE.println(AS_iMaxMowRpm);
  }
}
