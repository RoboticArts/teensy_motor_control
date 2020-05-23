#ifndef QUADRATURE_ENCODER_H
#define QUADRATURE_ENCODER_H

#include <Arduino.h>

#define ENCODER_1 1
#define ENCODER_2 2
#define ENCODER_3 3
#define ENCODER_4 4

#define ENCODER_THRESHOLD 250 // Timeout in ms to consider zero speed


class QuadratureEncoder{ 
  
  public:

    int pulses_per_turn = 0;
    float wheel_radius = 0;
    int encoder = 0; 

    QuadratureEncoder(int encoder, int encoder_pin_a, int encoder_pin_b, int pulses_per_turn, float wheel_radius);
    float getSpeed(String units);

};

#endif
