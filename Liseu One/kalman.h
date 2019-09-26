#ifndef KALMAN_H
#define KALMAN_H

//   kalman_t z_axis = { .processNoise = 0.01, .measurementNoise = 0.25, .error = 1, .gain = 0, .value = 0 };



typedef struct {
  float processNoise;
  float measurementNoise;
  float value;
  float error;
  float gain;
} kalman_t;



void kalmanUpdate(kalman_t *instance, float measurement)
{
   instance->error   = instance->error + instance->processNoise;
   instance->gain    = instance->error / (instance->error + instance->measurementNoise);
   instance->value  = instance->value + instance->gain * (measurement - instance->value);
   instance->error   = (1 - instance->gain) * instance->error;
}




#endif /* KALMAN_H */
