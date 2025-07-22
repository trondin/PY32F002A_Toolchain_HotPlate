#ifndef _PID_H
#define _PID_H

#include <stdint.h>
#include "py32f030x6.h"

typedef struct
{
  float prev_error;     // Previous error
  float integral;       // Integral sum
  float prev_temp;      // Previous temperature
  uint8_t is_overheating; // Overheating flag (0/1)
  uint8_t is_first_call;  // First call flag
} AdaptivePIDState;

void pid_init(AdaptivePIDState* pid);
uint8_t pid_compute(AdaptivePIDState* pid, int16_t setpoint, int16_t current_temp);

#endif