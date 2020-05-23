
  #include "QuadratureEncoder.h"



  // ------------- Encoder 1 ------------- //
  
  volatile float time_now_1 = 0, last_time_1 = 0;
  volatile bool direction_1 = true;
  
  void hall_A_ISR1()
  {  
     last_time_1 = time_now_1;
     time_now_1 = micros();
  }
  
  
  void hall_B_ISR1()
  {
   
    if((micros()-time_now_1) < (time_now_1-last_time_1)/3.0)
        direction_1 = true;
    else
        direction_1 = false;  
  
  }
  
  
  // ------------- Encoder 2 ------------- //
  
  volatile float time_now_2 = 0, last_time_2 = 0;
  volatile bool direction_2 = true;
  
  void hall_A_ISR2()
  {  
     last_time_2 = time_now_2;
     time_now_2 = micros();
  }
  
  
  void hall_B_ISR2()
  {
   
    if((micros()-time_now_2) < (time_now_2-last_time_2)/3.0)
        direction_2 = true;
    else
        direction_2 = false;  
  
  }
  
  // ------------- Encoder 3 ------------- //
  
  volatile float time_now_3 = 0, last_time_3 = 0;
  volatile bool direction_3 = true;
  
  void hall_A_ISR3()
  {  
     last_time_3 = time_now_3;
     time_now_3 = micros();
  }
  
  
  void hall_B_ISR3()
  {
   
    if((micros()-time_now_3) < (time_now_3-last_time_3)/3.0)
        direction_3 = true;
    else
        direction_3 = false;  
  
  }
  
  
  // ------------- Encoder 4 ------------- //
  
  volatile float time_now_4 = 0, last_time_4 = 0;
  volatile bool direction_4 = true;
  
  void hall_A_ISR4()
  {  
     last_time_4 = time_now_4;
     time_now_4 = micros();
  }
  
  
  void hall_B_ISR4()
  {
   
    if((micros()-time_now_4) < (time_now_4-last_time_4)/3.0)
        direction_4 = true;
    else
        direction_4 = false;  
  
  }


  // --------------------- Quadrature Encoder Class ------------------- //


  QuadratureEncoder::QuadratureEncoder(int encoder, int encoder_pin_a, int encoder_pin_b, int pulses_per_turn, float wheel_radius){
    
      pinMode(encoder_pin_a, INPUT);
      pinMode(encoder_pin_b, INPUT);

      this->encoder = encoder;
      this->pulses_per_turn = pulses_per_turn;
      this->wheel_radius = wheel_radius;
  
      
      if (encoder == ENCODER_1){
        attachInterrupt(digitalPinToInterrupt(encoder_pin_a), hall_A_ISR1, RISING);
        attachInterrupt(digitalPinToInterrupt(encoder_pin_b), hall_B_ISR1, RISING);
      }
  
      if (encoder == ENCODER_2){
        attachInterrupt(digitalPinToInterrupt(encoder_pin_a), hall_A_ISR2, RISING);
        attachInterrupt(digitalPinToInterrupt(encoder_pin_b), hall_B_ISR2, RISING);
      }
  
      if (encoder == ENCODER_3){
        attachInterrupt(digitalPinToInterrupt(encoder_pin_a), hall_A_ISR3, RISING);
        attachInterrupt(digitalPinToInterrupt(encoder_pin_b), hall_B_ISR3, RISING);
      }
  
      if (encoder == ENCODER_4){
        attachInterrupt(digitalPinToInterrupt(encoder_pin_a), hall_A_ISR4, RISING);
        attachInterrupt(digitalPinToInterrupt(encoder_pin_b), hall_B_ISR4, RISING);
      }
  
      
  }


  float QuadratureEncoder::getSpeed(String units){
  
      float velocity_value = 0;
      float period_per_pulse, period_per_turn;
      float f, rpm, w, v;
      float time_now = 0, last_time = 0;
      bool  direction = true;
  
      if (this->encoder == ENCODER_1){
  
          time_now = time_now_1;
          last_time = last_time_1;
          direction = direction_1;
          
      }
  
      if (this->encoder == ENCODER_2){
  
          time_now = time_now_2;
          last_time = last_time_2;
          direction = direction_2;
          
      }
  
      if (this->encoder == ENCODER_3){
  
          time_now = time_now_3;
          last_time = last_time_3;
          direction = direction_3;
          
      }
  
      if (this->encoder == ENCODER_4){
  
          time_now = time_now_4;
          last_time = last_time_4;
          direction = direction_4;
          
      }
  
      period_per_pulse = (time_now - last_time)/1000000; 
      f = 1.0 / period_per_pulse;
      period_per_turn = period_per_pulse * this->pulses_per_turn;
  
      //Wheel velocities
      rpm = f*60; //rpm
      w = 2*PI*(1/period_per_turn); // rad/s
      v = w * this->wheel_radius; // m/s
  
  
      if(units.equals("m/s"))
          velocity_value = v;
      if(units.equals("rad/s"))
          velocity_value = w;
      if(units.equals("Hz"))
          velocity_value = f;
      if(units.equals("rpm"))
          velocity_value = rpm; 
  
  
    if(direction){
      velocity_value = 1*abs(velocity_value);
    }
    else{
      velocity_value = -1*abs(velocity_value);
    }
    
  
  
    float current_period_per_pulse = (micros() - last_time)/1000;
    
    if (current_period_per_pulse > ENCODER_THRESHOLD){
      velocity_value = 0;
    }
  

    return velocity_value;
  }



 
