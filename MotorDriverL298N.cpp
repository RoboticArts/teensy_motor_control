

  #include "MotorDriverL298N.h"


  MotorDriverL298N::MotorDriverL298N(int pwm_pin, int in1_pin, int in2_pin, int pwm_resolution){


    this->pwm_limit = pow(2,pwm_resolution);
    this->pwm_resolution = pwm_resolution; 

    this->IN1 = in1_pin;
    this->IN2 = in2_pin;    
    this->EN  = pwm_pin;
   
  }

  void MotorDriverL298N::begin(){
    
    pinMode(this->IN1, OUTPUT);
    pinMode(this->IN2, OUTPUT);
    pinMode(this->EN, OUTPUT);

    analogWriteResolution(this->pwm_resolution);

  
  }

  void MotorDriverL298N::setDeadzone(int deadzone){

      this->deadzone = deadzone;
    
  }

  int MotorDriverL298N::setLimits(int value, int limit){
 
    if(value > limit)
        value = limit;

    if(value < -limit)
        value = -limit;

    return value;
    
  }


  void MotorDriverL298N::setPWM(int pwm_value){


    pwm_value = setLimits(pwm_value, this->pwm_limit);

    analogWrite(this->EN, abs(pwm_value));


    if(pwm_value > this->deadzone){
      
        digitalWrite(this->IN1, HIGH);
        digitalWrite(this->IN2, LOW);
    }

    else if(pwm_value < -this->deadzone){
      
        digitalWrite(this->IN1, LOW);
        digitalWrite(this->IN2, HIGH);
    }

    else{
      
        digitalWrite(this->IN1, LOW);
        digitalWrite(this->IN2, LOW);
    }

    
  }
  
  
