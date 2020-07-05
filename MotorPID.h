#ifndef MOTOR_PID_H
#define MOTOR_PID_H

#include <Arduino.h>

#include <PID_v1.h>
#include "MotorDriverL298N.h"
#include "QuadratureEncoder.h"
#include "teensy_motor_control.h"


class MotorPID{

  private:

     MotorDriverL298N *motor;
     QuadratureEncoder *encoder;

     double Setpoint, Input, Output;
     PID *mPID;
  
  public:

     MotorPID(MotorDriverL298N &motor, QuadratureEncoder &encoder, double Kp, double Ki, double Kd);
     void begin(void);
     void run(void);
     void setVelocity(double velocity);
     float getVelocity(void);
  
  

};





#endif
