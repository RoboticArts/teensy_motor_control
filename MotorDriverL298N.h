#ifndef MOTOR_DRIVER_L298N_H
#define MOTOR_DRIVER_L298N_H



#include <Arduino.h>


#define PWM_16_BITS 16 // 0 - 65535
#define PWM_12_BITS 12 // 0 - 4095
#define PWM_10_BITS 10 // 0 - 1023
#define PWM_8_BITS   8 // 0 - 255



class MotorDriverL298N{ 

  int IN1, IN2, EN;
  int pwm_resolution = 8;
  int pwm_limit;
  int deadzone = 0;
  
  public:

    MotorDriverL298N(int pwm_pin, int in1_pin, int in2_pin, int pwm_resolution = 8);
    void begin(void);
    void setDeadzone(int deadzone);
    void setPWM(int pwm_value);

  private:

    int setLimits(int value, int limit);
    
};

#endif
