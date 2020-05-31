
#include <PID_v1.h>
#include "MotorDriverL298N.h"
#include "QuadratureEncoder.h"
#include "teensy_motor_control.h"
#include "MotorPID.h"


elapsedMillis timeout_sys;
elapsedMillis timeout_setpoint;
bool flag = false;
int count = 0;

MotorDriverL298N front_left_motor(PWM_FRONT_LEFT, IN1_FRONT_LEFT, IN2_FRONT_LEFT, PWM_12_BITS);
MotorDriverL298N front_right_motor(PWM_FRONT_RIGHT, IN1_FRONT_RIGHT, IN2_FRONT_RIGHT, PWM_12_BITS);
MotorDriverL298N rear_right_motor(PWM_REAR_RIGHT, IN1_REAR_RIGHT, IN2_REAR_RIGHT, PWM_12_BITS);
MotorDriverL298N rear_left_motor(PWM_REAR_LEFT, IN1_REAR_LEFT, IN2_REAR_LEFT, PWM_12_BITS);

QuadratureEncoder front_left_encoder (ENCODER_1, ENC_A_FRONT_LEFT, ENC_B_FRONT_LEFT, PULSES_PER_TURN, WHEEL_RADIUS);
QuadratureEncoder front_right_encoder (ENCODER_2, ENC_A_FRONT_RIGHT, ENC_B_FRONT_RIGHT, PULSES_PER_TURN, WHEEL_RADIUS);
QuadratureEncoder rear_right_encoder (ENCODER_3, ENC_A_REAR_RIGHT, ENC_B_REAR_RIGHT, PULSES_PER_TURN, WHEEL_RADIUS);
QuadratureEncoder rear_left_encoder (ENCODER_4, ENC_A_REAR_LEFT, ENC_B_REAR_LEFT, PULSES_PER_TURN, WHEEL_RADIUS);

double Kp=6500, Ki=25000, Kd=0;
MotorPID rear_left_controller(rear_left_motor, rear_left_encoder, Ki, Kp, Kd);



double Setpoint, Input, Output;
//double Kp=200, Ki=800, Kd=1;
//double Kp=35400, Ki=278480, Kd=615;

//double Kp= 300, Ki=3000, Kd=0; // 8 bits
//double Kp=13941, Ki=92940, Kd=0;

//double Kp=6500, Ki=25000, Kd=0; // 5000 20000

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


void setup() {

Serial.begin(9600);
while(!Serial);

delay(500);

front_left_motor.begin();
front_right_motor.begin();
rear_right_motor.begin();
rear_left_motor.begin();

front_left_motor.setDeadzone(0);
front_right_motor.setDeadzone(0);
rear_right_motor.setDeadzone(0);
rear_left_motor.setDeadzone(0);


Input = front_right_encoder.getSpeed("m/s");
Setpoint = 0.0;
myPID.SetOutputLimits(-4095, 4095);
myPID.SetSampleTime(5);
myPID.SetMode(AUTOMATIC);

timeout_sys = 0;
timeout_setpoint = 0;


pinMode(A0,INPUT);
analogReadRes(12);

}


void loop() {





/*
   double Setpoint_1 = map(analogRead(A0), 0, 4095, -60, 60)/100.0;
   Setpoint = Setpoint_1;

  if (abs(Setpoint) >= 0.1){
    
    Input = front_right_encoder.getSpeed("m/s");
    myPID.Compute();

    if(Setpoint >= 0 && Output < 0)
      Output = 0;
    if(Setpoint < 0 && Output > 0)
      Output = 0;
    
    front_right_motor.setPWM(int(Output));
  
  }
    
  else if (abs(Setpoint) >= 0.05){
  
      Input = front_right_encoder.getSpeed("m/s");
      if(Setpoint >= 0)
        Setpoint = 0.1;
      else
        Setpoint = -0.1;
        
      myPID.Compute();

      if(Setpoint >= 0 && Output < 0)
        Output = 0;
      if(Setpoint < 0 && Output >= 0)
         Output = 0;
         
      front_right_motor.setPWM(int(Output));
  }
  
  else if (abs(Setpoint) < 0.05){
  
     Input = 0;
     Setpoint = 0;
     myPID.Compute();
     
   
  }
*/



/*
if(timeout_sys < 10000){
  if (timeout_setpoint > 5){
    timeout_setpoint = 0;
    Serial.print(Input, 4);
    Serial.print(" ------- ");
    count+=10;
    Serial.println(count);
  }
}
*/

/*
 if(timeout_sys > 10){

  timeout_sys = 0;


  //myPID.SetTunings(analogRead(A0)*200, Ki, Kd);
  
  //Serial.println(Output);
  //Serial.println(analogRead(A0)*200);

  //Serial.println(Output);

  
  Serial.print(0.6*100);
  Serial.print(",");
  Serial.print(0.1*100);
  Serial.print(",");
  Serial.print(-0.1*100);
  Serial.print(",");
  Serial.print(Input*100);
  Serial.print(",");
  Serial.print(Setpoint_1*100);
  Serial.println();


 }
*/


/*
if (timeout_setpoint > 3000){

   

    timeout_setpoint = 0;

    if(flag == false){
      Setpoint = 0.0;
      flag = true;
    }
    else{
      Setpoint = 0.2;
      flag = false;  
    }
 }
 */
  
/*
  if(timeout_sys > 10){


    front_left_motor.setPWM(60);
    front_right_motor.setPWM(60);
    rear_right_motor.setPWM(60);
    rear_left_motor.setPWM(60);

    float w = front_left_encoder.getSpeed("m/s");
    float x = front_right_encoder.getSpeed("m/s");
    float y = rear_right_encoder.getSpeed("m/s");
    float z = rear_left_encoder.getSpeed("m/s");
   
    Serial.println(w,5);
    Serial.println(x,5);
    Serial.println(y,5);
    Serial.println(z,5);
    Serial.println("-------");
    delay(500);
 
  }
  */
}
