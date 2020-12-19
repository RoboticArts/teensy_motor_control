

  #include "MotorPID.h"

  MotorPID::MotorPID(MotorDriverL298N &motor, QuadratureEncoder &encoder, double Kp, double Ki, double Kd){
    
    this->motor = new MotorDriverL298N(motor);
    this->encoder = new QuadratureEncoder(encoder);
    
    mPID = new PID(&this->Input, &this->Output, &this->Setpoint, Kp, Ki, Kd, DIRECT);
  }

  void MotorPID::begin(void){

    // Motor begin
    motor->begin();
    motor->setDeadzone(0);

   // PID begin
    mPID->SetOutputLimits(-4095, 4095);
    mPID->SetSampleTime(5);
    mPID->SetMode(AUTOMATIC);

    // Initial setpoint
    Setpoint = 0.0;
  }

  void MotorPID::run(void){
    
    //this->Input = encoder->getSpeed("m/s");
    //mPID->Compute();
    //motor->setPWM(this->Output); 

    if (abs(Setpoint) >= MINIMUM_SETPOINT){
      
      Input = encoder->getSpeed("m/s");
      mPID->Compute();
  
      if(Setpoint >= 0 && Output < 0)
        Output = 0;
      if(Setpoint < 0 && Output > 0)
        Output = 0;
      
      motor->setPWM(int(Output));
    
    }
      
    else if (abs(Setpoint) >= SETPOINT_THRESHOLD){
    
        Input = encoder->getSpeed("m/s");
        if(Setpoint >= 0)
          Setpoint = MINIMUM_SETPOINT;
        else
          Setpoint = -MINIMUM_SETPOINT;
          
        mPID->Compute();
  
        if(Setpoint >= 0 && Output < 0)
          Output = 0;
        if(Setpoint < 0 && Output >= 0)
           Output = 0;
           
        motor->setPWM(int(Output));
    }
    
    else if (abs(Setpoint) < SETPOINT_THRESHOLD){
    
       Input = 0;
       Setpoint = 0;
       mPID->Compute();
       motor->setPWM(0);
    }

    
  }


  void MotorPID::setVelocity(double velocity){

    this->Setpoint = velocity;
    
  }

  
  float MotorPID::getVelocity(String units){

    //return this->Input;
    return encoder -> getSpeed(units);
  }

  float MotorPID::getPosition(String units){
    
    return encoder->getPosition(units);
  }

  void MotorPID::resetPosition(){

    encoder->resetPosition();
  }
