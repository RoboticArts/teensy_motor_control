#ifndef TEENSY_MOTOR_CONTROL_H
#define TEENSY_MOTOR_CONTROL_H


// Motor properties
#define GEARBOX_RATIO 78
#define PULSES_PER_TURN 11*GEARBOX_RATIO
#define WHEEL_RADIUS 0.03875
#define RAD_PER_PULSE (2*PI)/float(PULSES_PER_TURN)
#define DEG_PER_PULSE 360/float(PULSES_PER_TURN)

// Motor names
#define MOTOR_FRONT_LEFT   0
#define MOTOR_FRONT_RIGHT  1
#define MOTOR_REAR_LEFT    2
#define MOTOR_REAR_RIGHT   3


// Control Pins defines //

// Front left motor
#define ENC_A_FRONT_LEFT 18
#define ENC_B_FRONT_LEFT 19
#define PWM_FRONT_LEFT 10   
#define IN1_FRONT_LEFT 11
#define IN2_FRONT_LEFT 12

// Front right motor
#define ENC_A_FRONT_RIGHT 23
#define ENC_B_FRONT_RIGHT 22
#define PWM_FRONT_RIGHT 3  
#define IN1_FRONT_RIGHT 2
#define IN2_FRONT_RIGHT 1

// Rear right motor
#define ENC_A_REAR_RIGHT 21
#define ENC_B_REAR_RIGHT 20
#define PWM_REAR_RIGHT 6  
#define IN1_REAR_RIGHT 5
#define IN2_REAR_RIGHT 4

// Rear left motor
#define ENC_A_REAR_LEFT 16
#define ENC_B_REAR_LEFT 17
#define PWM_REAR_LEFT 9   
#define IN1_REAR_LEFT 8
#define IN2_REAR_LEFT 7



// Control parameters
#define SETPOINT_THRESHOLD 0.001 // Setpoints between 0 and SETPOINT_THRESHOLD are considered zero 
#define MINIMUM_SETPOINT 0.002    // Setpoints between SETPOINT_THRESHOLD and MINIMUM_SETPOINT are considered MINIMUM_SETPOINT



#endif
