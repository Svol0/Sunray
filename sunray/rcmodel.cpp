// Ardumower Sunray 
/***************************************************************************
 * Modified version for use with an wireless PS2 controller to 
 * start/stop mowmotor and give obsticle feedback by DualShockController.
 * programm PS2X_ArduMower needs to be installed on an Arduino Nano for
 * the wireless PS2 controller.
 * 
 ***************************************************************************/
#include "config.h"
#include "rcmodel.h"
#include "robot.h"

#define RC_DEBUG

volatile unsigned long PPM_start_lin = 0;
volatile unsigned long PPM_end_lin = 0;                
volatile unsigned long PPM_start_ang = 0;
volatile unsigned long PPM_end_ang = 0 ;
bool xFeedbackToggle  = false;              // for pulse signal at opsticle detection 
bool xMowMotor        = false;              // state storage for mowmotor on/off
unsigned long nextOutputTime =0;
bool mowOn = false;
unsigned long timeToggleForcefedback  = 0;
unsigned long timeFunctionButtonHigh  = 0;
int functionMode  = 0;
int reciveStep  = 0;

#ifdef RCMODEL_ENABLE      
  void get_lin_PPM(){                                                        // Interrupt Service Routine
    if (digitalRead(pinRemoteMow)==HIGH) PPM_start_lin = micros();  
    else                                   PPM_end_lin = micros();    
  }

  void get_ang_PPM(){                                                        // Interrupt Service Routine
    if (digitalRead(pinRemoteSteer)==HIGH) PPM_start_ang = micros();  
    else                                   PPM_end_ang = micros();  
  }
#endif

void RCModel::begin(){  
#ifdef RCMODEL_ENABLE
  CONSOLE.println("RCModel enabled in config");  
  lin_PPM = 0;                                            
  linearPPM = 0;                                         
  ang_PPM = 0;                                            
  angularPPM = 0;                                         
  RC_Mode = false; 
  nextControlTime = 0;
  timeFunctionButtonHigh  = millis();
  // R/C
  pinMode(pinRemoteSpeed, OUTPUT);  // remote control force feedback
  pinMode(pinRemoteSwitch, INPUT);  // remote control mow motor on/off
  #ifdef RC_DEBUG
    nextOutputTime = millis() + 1000;
  #endif
#else
  CONSOLE.println("RCModel disabled in config");  
#endif
} 

void RCModel::run(){
#ifdef RCMODEL_ENABLE
  unsigned long t = millis();
  if (!RCMODEL_ENABLE) return;


  if (RC_Mode) {
    // motor.setLinearAngularSpeed(linearPPM, angularPPM, false);                     // R/C Signale an Motor leiten
    motor.setLinearAngularSpeed(linearPPM, angularPPM, true);                     // R/C Signale an Motor leiten mit linearRamp
  }


  switch (reciveStep) {
    case 0: // wait for high
      if (digitalRead(pinRemoteSwitch) == HIGH){
        reciveStep++;
        timeFunctionButtonHigh = millis();
      }
      break;

    case 1: // wait for max 100ms till low
      if (digitalRead(pinRemoteSwitch) == LOW){
        CONSOLE.print("RC-MODE: high for ");
        CONSOLE.print(millis()-timeFunctionButtonHigh);
        CONSOLE.print("ms; step: ");
        CONSOLE.println(reciveStep);
        
        if (((millis() - timeFunctionButtonHigh) < 50) || ((millis() - timeFunctionButtonHigh) > 150)) {
          reciveStep = 0;
        } else {
          timeFunctionButtonHigh = millis();
          reciveStep++;
        }
      }
      break;

    case 2: // wait for high
      if (digitalRead(pinRemoteSwitch) == HIGH){
        CONSOLE.print("RC-MODE: low for ");
        CONSOLE.print(millis()-timeFunctionButtonHigh);
        CONSOLE.print("ms; step: ");
        CONSOLE.println(reciveStep);

        if (((millis() - timeFunctionButtonHigh) < 50) || ((millis() - timeFunctionButtonHigh) > 150)) {
          reciveStep = 0;
        } else {
          timeFunctionButtonHigh = millis();
          reciveStep++;
        }
      } else {
        if ((millis() - timeFunctionButtonHigh) > 150) reciveStep = 0;
      }
      break;

    case 3: // wait for max 100ms till low
      if (digitalRead(pinRemoteSwitch) == LOW){
        CONSOLE.print("RC-MODE: signal lenght ");
        CONSOLE.print(millis()-timeFunctionButtonHigh);
        CONSOLE.print("ms; step: ");
        CONSOLE.println(reciveStep);

        // kontrolliere die Dauer, die der Eingang high war
        if ((millis() - timeFunctionButtonHigh) > 1050 ) functionMode = 0; // 
        else if ((millis() - timeFunctionButtonHigh) > 950 ) functionMode = 3; // toggle RC-Mode
        else if ((millis() - timeFunctionButtonHigh) > 150 ) functionMode = 2; // mow motor on
        else if ((millis() - timeFunctionButtonHigh) > 50 ) functionMode = 1; // mow motor off
        reciveStep = 0;
      } else {
          if ((millis() - timeFunctionButtonHigh) > 1200) reciveStep = 0; 
        }
      break;
      
  }
/*
  // lese den Eingang, um gegebenenfalls den RC-Mode zu aktivieren
  if (digitalRead(pinRemoteSwitch) == HIGH){
    
  } else {
    // kontrolliere die Dauer, die der Eingang high war
    if ((millis() - timeFunctionButtonHigh) > 1050 ) functionMode = 0; // 
    else if ((millis() - timeFunctionButtonHigh) > 950 ) functionMode = 3; // toggle RC-Mode
    else if ((millis() - timeFunctionButtonHigh) > 150 ) functionMode = 2; // mow motor on
    else if ((millis() - timeFunctionButtonHigh) > 50 ) functionMode = 1; // mow motor off
    timeFunctionButtonHigh = millis();
  }
*/  
  
  if (t < nextControlTime) return;
  nextControlTime = t + 50;                                       // save CPU resources by running at 20 Hz
  
  if ((stateButton == 3) || (functionMode == 3)){                                           // 3 button beeps
      if (functionMode == 3) functionMode = 0;                     // Reset functionMode
      stateButton = 0;                                             // reset button state
      RC_Mode = !RC_Mode;                                                   // R/C-Mode toggle
      if (RC_Mode)  {                                                       // R/C-Mode ist aktiv
        CONSOLE.println("button mode 3 - RC Mode ON");
        buzzer.sound(SND_ERROR, true);                                      // 3x Piep für R/C aktiv        
        attachInterrupt(digitalPinToInterrupt(pinRemoteMow), get_lin_PPM, CHANGE);// Interrupt aktivieren
        attachInterrupt(digitalPinToInterrupt(pinRemoteSteer), get_ang_PPM, CHANGE);// Interrupt aktivieren 
        xMowMotor = false;
      }
      if (!RC_Mode) {                 
        CONSOLE.println("button mode 3 - RC Mode OFF");                                      // R/C-Mode inaktiv
        buzzer.sound(SND_WARNING, true);                          // 2x Piiiiiiiep für R/C aus
        motor.setLinearAngularSpeed(0, 0);                                 
        detachInterrupt(digitalPinToInterrupt(pinRemoteMow));             // Interrupt deaktivieren
        detachInterrupt(digitalPinToInterrupt(pinRemoteSteer));             // Interrupt deaktivieren
        motor.setMowState(false);                                         // mowmotor off
      }    
  }
  
  if (RC_Mode)    {       
    lin_PPM = 0;
    if (PPM_start_lin < PPM_end_lin) lin_PPM = PPM_end_lin - PPM_start_lin; 
    if (lin_PPM < 2000 && lin_PPM > 1000)   {                               // Wert innerhalb 1100 bis 1900µsec
      float value_l = (lin_PPM - 1500);                                     // PPM auf Bereich +400 bis -400
      value_l = map(value_l, -400, 400, -(MOTOR_MAX_SPEED * 100), (MOTOR_MAX_SPEED * 100)); // PPM mappen
      value_l = value_l / 100;
      if ((value_l < 0.05) && (value_l > -0.05)) value_l = 0;                 // NullLage vergrössern         
      linearPPM = value_l;                                                    // Weitergabe an Debug
    }

    ang_PPM = 0;
    if (PPM_start_ang < PPM_end_ang) ang_PPM = PPM_end_ang - PPM_start_ang; 
    if (ang_PPM < 2000 && ang_PPM > 1000)   {                               // Wert innerhalb 1100 bis 1900µsec
      float value_a = (ang_PPM - 1500) / 600;                                 // PPM auf Bereich +0.75 bis -0.75
      if ((value_a < 0.05) && (value_a > -0.05)) value_a = 0;                 // NullLage vergrössern         
      angularPPM = value_a;                                                   // Weitergabe an Debug
    }


    // if bumper is triggered, DualShockController feedback should be hard 
    // if sonar is triggered, DualShockControler should give little feedback
    if (!sonar.obstacle()){
      timeToggleForcefedback  = millis();
    }
    if (bumper.obstacle()) {
      digitalWrite(pinRemoteSpeed, HIGH);      
    }
    else if (sonar.obstacle()) {
      // der Ausgang wird alle 100ms gewechselt
      if (millis() > timeToggleForcefedback){
        xFeedbackToggle = !xFeedbackToggle;
        digitalWrite(pinRemoteSpeed, xFeedbackToggle);
        timeToggleForcefedback  = timeToggleForcefedback + 100;
      }
    }
    else {
      digitalWrite(pinRemoteSpeed, LOW);
      xFeedbackToggle = false;
    }

  if (RC_Mode){

    // mowmotor on and off
    if (functionMode == 2) {
//    if (digitalRead(pinRemoteSwitch) == HIGH && !xMowMotor){
      xMowMotor = true;
      CONSOLE.println("RC Mode: Mowmotor startet by remote!");
      functionMode = 0;
    }
    if (functionMode == 1) {
//    else if (digitalRead(pinRemoteSwitch) == LOW && xMowMotor){      
      xMowMotor = false;
      CONSOLE.println("RC Mode: Mowmotor stopped by remote!");
      functionMode = 0;
    }
  motor.setMowState(xMowMotor);
  }
  else functionMode = 0;


#ifdef RC_DEBUG
    if (t >= nextOutputTime) {
      nextOutputTime = t + 1000;

      CONSOLE.print("RC: linearPPM= ");
      CONSOLE.print(linearPPM);
      CONSOLE.print("  angularPPM= ");
      CONSOLE.println(angularPPM);
    }
#endif
    // motor.setLinearAngularSpeed(linearPPM, angularPPM, false);                     // R/C Signale an Motor leiten
    // motor.setLinearAngularSpeed(linearPPM, angularPPM, true);                     // R/C Signale an Motor leiten mit linearRamp
  }
#endif
}
