
#include <PID_v1.h>
#include "MotorDriverL298N.h"
#include "QuadratureEncoder.h"
#include "teensy_motor_control.h"
#include "MotorPID.h"

// Utils
elapsedMillis timeout_sys;
float tic = 0, toc = 0;
double setpoint, input_1, input_2, input_3, input_4;


MotorDriverL298N front_left_motor(PWM_FRONT_LEFT, IN1_FRONT_LEFT, IN2_FRONT_LEFT, PWM_12_BITS);
MotorDriverL298N front_right_motor(PWM_FRONT_RIGHT, IN1_FRONT_RIGHT, IN2_FRONT_RIGHT, PWM_12_BITS);
MotorDriverL298N rear_right_motor(PWM_REAR_RIGHT, IN1_REAR_RIGHT, IN2_REAR_RIGHT, PWM_12_BITS);
MotorDriverL298N rear_left_motor(PWM_REAR_LEFT, IN1_REAR_LEFT, IN2_REAR_LEFT, PWM_12_BITS);

QuadratureEncoder front_left_encoder (ENCODER_1, ENC_A_FRONT_LEFT, ENC_B_FRONT_LEFT, PULSES_PER_TURN, WHEEL_RADIUS);
QuadratureEncoder front_right_encoder (ENCODER_2, ENC_A_FRONT_RIGHT, ENC_B_FRONT_RIGHT, PULSES_PER_TURN, WHEEL_RADIUS);
QuadratureEncoder rear_right_encoder (ENCODER_3, ENC_A_REAR_RIGHT, ENC_B_REAR_RIGHT, PULSES_PER_TURN, WHEEL_RADIUS);
QuadratureEncoder rear_left_encoder (ENCODER_4, ENC_A_REAR_LEFT, ENC_B_REAR_LEFT, PULSES_PER_TURN, WHEEL_RADIUS);


double Kp=6500, Ki=25000, Kd=0;

MotorPID front_right_controller(front_right_motor, front_right_encoder, Kp, Ki, Kd);
MotorPID front_left_controller(front_left_motor, front_left_encoder, Kp, Ki, Kd);
MotorPID rear_right_controller(rear_right_motor, rear_right_encoder, Kp, Ki, Kd);
MotorPID rear_left_controller(rear_left_motor, rear_left_encoder, Kp, Ki, Kd);



void setup() {

Serial.begin(9600);
while(!Serial);

front_right_controller.begin();
front_left_controller.begin();
rear_right_controller.begin();
rear_left_controller.begin();

timeout_sys = 0;
//pinMode(A0,INPUT);
//analogReadRes(12);

}


void loop() {

  //setpoint = map(analogRead(A0), 0, 4095, -60, 60)/100.0;
  setpoint = 0.5;
  
  rear_right_controller.setVelocity(setpoint);
  rear_right_controller.run();
  input_1 = rear_right_controller.getVelocity();

  rear_left_controller.setVelocity(setpoint);
  rear_left_controller.run();
  input_2 = rear_left_controller.getVelocity();

  front_right_controller.setVelocity(setpoint);
  front_right_controller.run();
  input_3 = front_right_controller.getVelocity();

  front_left_controller.setVelocity(setpoint);
  front_left_controller.run();
  input_4 = front_left_controller.getVelocity();


  if(timeout_sys >= 10){

    Serial.print(0.6*100);
    Serial.print(",");
    Serial.print(0.1*100);
    Serial.print(",");
    Serial.print(-0.1*100);
    Serial.print(",");
    Serial.print(input_1*100);
    Serial.print(",");
    Serial.print(input_2*100);
    Serial.print(",");
    Serial.print(input_3*100);
    Serial.print(",");
    Serial.print(input_4*100);
    Serial.print(",");
    Serial.print(setpoint*100);
    Serial.println();

    timeout_sys = 0;

  }
  
}
