
  #include "QuadratureEncoder.h"


  volatile float time_now_array[4] = {0,0,0,0};
  volatile float last_time_array[4] = {0,0,0,0};
  volatile bool  direction_array[4] = {true,true,true,true};
  volatile bool  new_speed_array[4] = {false,false,false,false};

  volatile float old_time_now_array[4] = {0,0,0,0};
  volatile float old_last_time_array[4] = {0,0,0,0};
  volatile bool  old_direction_array[4] = {true,true,true,true};  
  

  void measurePulse(int encoder){

     last_time_array[encoder] = time_now_array[encoder];
     time_now_array[encoder] = micros();
    
  }
  
  void measureSign(int encoder){

       
    if((micros()-time_now_array[encoder]) < (time_now_array[encoder]-last_time_array[encoder])/3.0)
        direction_array[encoder] = true;
    else
        direction_array[encoder] = false;  

    old_last_time_array[encoder] = last_time_array[encoder];
    old_time_now_array[encoder]  = time_now_array[encoder];
    old_direction_array[encoder] = direction_array[encoder];
    new_speed_array[encoder]     = true;
    
  }
  
  // ------------- Encoder 1 ------------- //
  
  void hall_A_ISR1()
  {  
     measurePulse(ENCODER_1);
  }
  
  
  void hall_B_ISR1()
  {
     measureSign(ENCODER_1);
  }
  
  // ------------- Encoder 2 ------------- //
  
  void hall_A_ISR2()
  {  
     measurePulse(ENCODER_2);
  }
  
  
  void hall_B_ISR2()
  {
     measureSign(ENCODER_2);
  }
  
  // ------------- Encoder 3 ------------- //
  
  void hall_A_ISR3()
  {  
    measurePulse(ENCODER_3);
  }
  
  
  void hall_B_ISR3()
  {
    measureSign(ENCODER_3);
  }
  
  // ------------- Encoder 4 ------------- //
  
  
  void hall_A_ISR4()
  {  
    measurePulse(ENCODER_4);
  }
  
  void hall_B_ISR4()
  {
    measureSign(ENCODER_4);
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
      float time_now = 0, last_time = 0;
      float period_per_pulse, period_per_turn;
      float f, rpm, w, v;
      bool direction = true;
      int encoder = this->encoder;


      if(new_speed_array[encoder] == true){

          time_now = time_now_array[encoder];
          last_time = last_time_array[encoder];
          direction = direction_array[encoder];
          new_speed_array[encoder] = false;
        
      }

      else{
        
          time_now =  old_time_now_array[encoder];
          last_time = old_last_time_array[encoder];
          direction = old_direction_array[encoder];
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



 
