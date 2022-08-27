// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#ifndef MOTOR_H
#define MOTOR_H

#include "pid.h"


// selected motor
enum MotorSelect {MOTOR_LEFT, MOTOR_RIGHT, MOTOR_MOW} ;
typedef enum MotorSelect MotorSelect;
extern float reactivateLinear;


class Motor {
  public:
    float robotPitch;  // robot pitch (rad)
    float wheelBaseCm;  // wheel-to-wheel diameter
    int wheelDiameter;   // wheel diameter (mm)
    int ticksPerRevolution; // ticks per revolution
    int ticksPerMowMotorRevolution; // ticks per revolution of the Mowmotor
    float ticksPerCm;  // ticks per cm
    bool activateLinearSpeedRamp;  // activate ramp to accelerate/slow down linear speed?
    bool toggleMowDir; // toggle mowing motor direction each mow motor start?    
    bool motorLeftSwapDir;
    bool motorRightSwapDir;
    bool motorError;
    bool motorLeftOverload; 
    bool motorRightOverload; 
    bool motorMowOverload; 
    bool tractionMotorsEnabled;       
    bool enableMowMotor;
    bool odometryError;    
    unsigned long motorOverloadDuration; // accumulated duration (ms)
    int  pwmMax;
    int  pwmMaxMow;
    float  pwmSpeedOffset;
    float SpeedOffset;
    float mowMotorCurrentAverage;
    float currentFactor;
    bool pwmSpeedCurveDetection;
    unsigned long motorLeftTicks;
    unsigned long motorRightTicks;
    unsigned long motorMowTicks;    
    float linearSpeedSet; // m/s
    float angularSpeedSet; // rad/s
    float motorLeftSense; // left motor current (amps)
    float motorRightSense; // right  motor current (amps)
    float motorMowSense;  // mower motor current (amps)         
    float motorLeftSenseLP; // left motor current (amps, low-pass)
    float motorRightSenseLP; // right  motor current (amps, low-pass)
    float motorMowSenseLP;  // mower motor current (amps, low-pass)       
    float motorsSenseLP; // all motors current (amps, low-pass)
    float motorLeftSenseLPNorm; 
    float motorRightSenseLPNorm;
    float motorLeftRpmCurrLP;
    float motorRightRpmCurrLP;
    float motorMowRpmCurrLP;
    float motorMowPWMCurr; 
    float motorMowSenseMed; //mower motor current (amps, Median) //X
    unsigned long motorMowSpinUpTime;
    void begin();
    void run();      
    void test();
    void testMow(); //Svol0 TestMowMotor
    void plot();
    void enableTractionMotors(bool enable);
    void setLinearAngularSpeed(float linear, float angular, bool useLinearRamp = true);
    void setMowState(bool switchOn);   
    void stopImmediately(bool includeMowerMotor);
    float calcStopWay;  // calculated distance for stop from actual speed
    float motorMowPWMSet;  
    bool reactivateBlockBlDriver;
    
  protected: 
    float motorLeftRpmSet; // set speed
    float motorRightRpmSet;   
    float motorLeftRpmCurr;
    float motorRightRpmCurr;
    float motorMowRpmCurr;    
    float motorLeftRpmLast;
    float motorRightRpmLast;
    bool motorMowForwardSet; 
//    float motorMowPWMSet;  
//    float motorMowPWMCurr; 
    int motorLeftPWMCurr;
    int motorRightPWMCurr;    
    float motorMowPWMCurrLP;
    float motorLeftPWMCurrLP;
    float motorRightPWMCurrLP;    
    unsigned long lastControlTime;    
    unsigned long nextSenseTime;            
    bool recoverMotorFault;
    int recoverMotorFaultCounter;
    unsigned long nextRecoverMotorFaultTime;
    int motorLeftTicksZero;    
    int motorRightTicksZero;    
    PID motorLeftPID;
    PID motorRightPID;        
    bool setLinearAngularSpeedTimeoutActive;
    unsigned long setLinearAngularSpeedTimeout;    
    void speedPWM ( int pwmLeft, int pwmRight, int pwmMow );
    void control();    
    bool checkFault();
    void checkOverload();
    bool checkOdometryError();
    bool checkMowRpmFault();
    bool checkCurrentTooHighError();    
    bool checkCurrentTooLowError();
    void sense();
    void dumpOdoTicks(int seconds);
    void dumpOdoMowTicks();    
    unsigned long lastLinearSetTime;  // Svol0: test
    float accStep;
    float decStep;
    float mowRampStep;
};


#endif
