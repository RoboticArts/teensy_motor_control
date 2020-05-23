
#include "QuadratureEncoder.h"
#include "teensy_motor_control.h"


elapsedMillis timeout_sys;


QuadratureEncoder front_left_encoder (ENCODER_1, ENC_A_FRONT_LEFT, ENC_B_FRONT_LEFT, PULSES_PER_TURN, WHEEL_RADIUS);
QuadratureEncoder front_right_encoder (ENCODER_2, ENC_A_FRONT_RIGHT, ENC_B_FRONT_RIGHT, PULSES_PER_TURN, WHEEL_RADIUS);
QuadratureEncoder rear_right_encoder (ENCODER_3, ENC_A_REAR_RIGHT, ENC_B_REAR_RIGHT, PULSES_PER_TURN, WHEEL_RADIUS);
QuadratureEncoder rear_left_encoder (ENCODER_4, ENC_A_REAR_LEFT, ENC_B_REAR_LEFT, PULSES_PER_TURN, WHEEL_RADIUS);


void setup() {

Serial.begin(9600);

while(!Serial);
}


void loop() {

  if(timeout_sys > 10){
    float w = front_left_encoder.getSpeed("m/s");
    float x = front_right_encoder.getSpeed("m/s");
    float y = rear_right_encoder.getSpeed("m/s");
    float z = rear_left_encoder.getSpeed("m/s");
    
    Serial.println(w,5);
    Serial.println(x,5);
    Serial.println(y,5);
    Serial.println(z,5);
    Serial.println("-------");
 
  }
}
