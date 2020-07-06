#define USE_USBCON

#include <PID_v1.h>
#include "MotorDriverL298N.h"
#include "QuadratureEncoder.h"
#include "teensy_motor_control.h"
#include "MotorPID.h"

#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>



// Global variables
float setpoint[4] = {0,0,0,0};

// Utils variables
elapsedMillis timeout_sys;
float tic = 0, toc = 0;
double input_1, input_2, input_3, input_4;
//double setpoint;

// ROS variables
ros::NodeHandle nh;

void motorSetpointCallback(const std_msgs::Float32MultiArray& msg){
  
  setpoint[MOTOR_FRONT_LEFT]  = msg.data[MOTOR_FRONT_LEFT]  * WHEEL_RADIUS; // rad to m/s
  setpoint[MOTOR_FRONT_RIGHT] = msg.data[MOTOR_FRONT_RIGHT] * WHEEL_RADIUS; // rad to m/s
  setpoint[MOTOR_REAR_LEFT]   = msg.data[MOTOR_REAR_LEFT]   * WHEEL_RADIUS; // rad to m/s
  setpoint[MOTOR_REAR_RIGHT]  = msg.data[MOTOR_REAR_RIGHT]  * WHEEL_RADIUS; // rad to m/s
  
}

ros::Subscriber<std_msgs::Float32MultiArray> motor_sub_setpoint("teensy_motor_control/velocity_setpoint", &motorSetpointCallback);

sensor_msgs::JointState motor_state;

float pos[]={0,0,0,0};
float vel[]={0,0,0,0};
float eff[]={0,0,0,0};

ros::Publisher motor_pub_state("teensy_motor_control/motor_state", &motor_state);



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

front_right_controller.begin();
front_left_controller.begin();
rear_right_controller.begin();
rear_left_controller.begin();

timeout_sys = 0;

nh.getHardware()->setBaud(2000000);
nh.initNode();
nh.advertise(motor_pub_state);
nh.subscribe(motor_sub_setpoint);


motor_state.position= pos;
motor_state.velocity= vel;
motor_state.effort= eff;

motor_state.position_length=4;
motor_state.velocity_length=4;
motor_state.effort_length=4;

}


void loop() {


  
  rear_right_controller.setVelocity(setpoint[MOTOR_REAR_RIGHT]);
  rear_right_controller.run();
  motor_state.position[MOTOR_REAR_RIGHT] = rear_right_controller.getPosition("rad");
  motor_state.velocity[MOTOR_REAR_RIGHT] = rear_right_controller.getVelocity("rad/s");


  rear_left_controller.setVelocity(setpoint[MOTOR_REAR_LEFT]);
  rear_left_controller.run();
  motor_state.position[MOTOR_REAR_LEFT] = rear_left_controller.getPosition("rad");
  motor_state.velocity[MOTOR_REAR_LEFT] = rear_left_controller.getVelocity("rad/s");

  front_right_controller.setVelocity(setpoint[MOTOR_FRONT_RIGHT]);
  front_right_controller.run();
  motor_state.position[MOTOR_FRONT_RIGHT] = front_right_controller.getPosition("rad");
  motor_state.velocity[MOTOR_FRONT_RIGHT] = front_right_controller.getVelocity("rad/s");

  front_left_controller.setVelocity(setpoint[MOTOR_FRONT_LEFT]);
  front_left_controller.run();
  motor_state.position[MOTOR_FRONT_LEFT] = front_left_controller.getPosition("rad");
  motor_state.velocity[MOTOR_FRONT_LEFT] = front_left_controller.getVelocity("rad/s");


  if (timeout_sys >= 50){

     motor_pub_state.publish(&motor_state);
     nh.spinOnce();
     timeout_sys = 0;
  }

 
/*
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
*/
  
}
