// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

// buzzer - play beep sounds (async)

#ifndef ADAPTIVESPEED_H
#define ADAPTIVESPEED_H

//#include "pid.h"


// selected motor
//enum MotorSelect {MOTOR_LEFT, MOTOR_RIGHT, MOTOR_MOW} ;
//typedef enum MotorSelect MotorSelect;


class AdaptiveSpeed {
  public:
    float AS_mowCurrNoLoad_MinSpeedMed;
    float AS_mowCurrNoLoad_MaxSpeedMed;
    float AS_mowCurrForUpdate;
    unsigned long AS_mowReversTimer;
    bool AS_mowReversTrg;
    bool AS_mowCurrUpdateTrg;
    bool AS_mowSpeedInfoTrg;
    
    // for statistics
    float AS_maxMowSense  = 0;                // maximum measured mowmotor current at stable RPM
    float AS_maxMowSenseSpeedChange  = 0;     // maximum measured mowmotor current during speed change
    float AS_MowSenseAtIdleMIN_MOW_RPM = 0;   // mowmotor current at MIN_MOW_RPM in idle
    float AS_MowSenseAtIdleMAX_MOW_RPM = 0;   // mowmotor current at MAX_MOW_RPM in idle
    unsigned long AS_MowDurationAtMIN_MOW_RPM = 0;
    unsigned long AS_MowDurationAtMID_MOW_RPM = 0;
    unsigned long AS_MowDurationAtMAX_MOW_RPM = 0;
    int AS_MowReversActionCounter = 0;
    
    
    bool AS_mowDriveRevers;
    bool AS_mowMotorIsOn;
    float AS_motorMowSense;
    float AS_motorMowSenseMed;

    int AS_pwmMow;            // act pwm for mow motor
    int AS_speedStep;
    float AS_linear;
    float AS_angular;
    
    int AS_mowCurrUpdate;
    
    unsigned long AS_Timer;

    void begin();
    void run();      
    void getActDriveSpeedValue(float linear, float angular);
    void getActMowSpeedValue(int pwmMow);
    void getMowIsOn(bool mowMotorIsOn = false);
    void getActMowSenseValue(float motorMowSense, float motorMowSenseMed);
    void getCommand(int mode, float value1, float value2, float value3, float value4, int value5 , int value6);
    void statisticsOutput();
    void statisticsReset();
    void helpOutput();
    float setSpeedOffset();
    int setPwmMow();
    void debugOut();
/*
    void test();
    
    void setMowState(bool switchOn);   
    void stopImmediately(bool includeMowerMotor);
*/        
  protected: 
/*
    float motorLeftRpmSet; // set speed
    void speedPWM ( int pwmLeft, int pwmRight, int pwmMow );
    void control();    
*/
};


#endif
