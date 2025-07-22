#include "py32f030x6.h"
#include "pid.h"

// Instructions for manually tuning PID coefficients for the heater:
// 1. Set initial values: KP = xx, KI = xx, KD = xx.
// 2. Test heating to 1500 (150 °C) with KP, setting KI = 0, KD = 0:
//    - If heating is slow, increase KP.
//    - If there are strong oscillations, decrease KP.
// 3. Add KI:
//    - Increase KI if the temperature takes too long to reach the setpoint.
//    - Decrease KI if sustained oscillations occur.
// 4. Add KD:
//    - Increase KD if there are sharp temperature spikes.
//    - Decrease KD if the system becomes too sluggish.
// 5. Repeat tests, adjusting one coefficient at a time, observing heating time (~40 s) and accuracy (±3 °C).
// 6. Verify stability at different setpoints (800–2000) and with heat losses.

#define CONSTRAIN(val, min_val, max_val) ((val) < (min_val) ? (min_val) : ((val) > (max_val) ? (max_val) : (val)))

#define MAX_POWER 19     // Maximum power level (1-19)
#define MIN_POWER 1      // Minimum heating level (0 = off)
#define POWER_OFF 0      // Complete shutdown

#define KP 0.16f         // Proportional coefficient
#define KI 0.030f        // Integral coefficient
#define KD 0.025f        // Derivative coefficient
#define DT 0.1f          // Control period (100 ms)
#define INTEGRAL_MAX 10.0f
#define INTEGRAL_MIN -10.0f

#define COOLING_RATE 0.5f   // Cooling rate coefficient (0.1-1.0)
#define SAFE_MARGIN 20.0f   // Margin to setpoint (2°C)

// Initialize PID
void pid_init(AdaptivePIDState* pid) 
{
  pid->prev_error = 0.0f;
  pid->integral = 0.0f;
  pid->prev_temp = 0.0f;
  pid->is_overheating = 0;
  pid->is_first_call = 1;
}

// Compute control signal
uint8_t pid_compute(AdaptivePIDState* pid, int16_t setpoint, int16_t current_temp) 
{
  float error = (float)(setpoint - current_temp);

  // Overheat prediction
  float temp_rise_rate = 0.0f;
  if (!pid->is_first_call && DT > 0.0f)  temp_rise_rate = (current_temp - pid->prev_temp) / DT;  
  float predicted_temp = current_temp + temp_rise_rate * DT * 5.0f;

  // Overheat protection logic
  if (predicted_temp > setpoint + SAFE_MARGIN) 
  {
    pid->is_overheating = 1;
    pid->integral *= 0.5f;  // More aggressive integral reset
  }
  else if (error < SAFE_MARGIN * 0.5f) pid->is_overheating = 0;  // Hysteresis

  // PID calculation
  float output = 0.0f;
  if (!pid->is_overheating) 
  {
    // Proportional term
    float p_term = KP * error;

    // Integral term with anti-windup
    pid->integral += error * DT;
    pid->integral = CONSTRAIN(pid->integral, INTEGRAL_MIN, INTEGRAL_MAX);
    float i_term = KI * pid->integral;

    // Derivative term with filtering
    float derivative = 0.0f;
    if (!pid->is_first_call) derivative = (error - pid->prev_error) / DT;    
    float d_term = KD * derivative;

    output = p_term + i_term + d_term;
  }

  // Convert to power levels
  uint8_t power = POWER_OFF;
  if (!pid->is_overheating) 
  {
    power = (uint8_t)(output / 20.0f + 0.5f);  // Rounding
    if (power >= MIN_POWER && predicted_temp > setpoint)
    {
      power = POWER_OFF;
      pid->is_overheating = 1;
    }
    power = CONSTRAIN(power, MIN_POWER, MAX_POWER);
  }

  // Update state
  pid->prev_error = error;
  pid->prev_temp = current_temp;
  pid->is_first_call = 0;

  return power;
}
