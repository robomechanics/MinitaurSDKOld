#include "Interpolator.h"

#define TIMESTEPS 84
#define MOTORS 9

Interpolator::Interpolator() {}

float Interpolator::getSingleZOH(float * targets, float * times, int t) {
  float pos;
  index=0;
  for(int j=0;j<TIMESTEPS;j++)
  {
    if(times[j]<=t && t<times[j+1]) index = j;
  }
  pos = targets[index];
  return pos;
}

float Interpolator::getSingleLinearInterp(float * targets, float * times, int t) {
  float pos;
  index=0;
  for(int j=0;j<TIMESTEPS;j++)
  {
    if(times[j]<=t && t<times[j+1]) index = j;
  }
  pos = targets[index] + (targets[index+1]-targets[index])*((float)(t-times[index]))/((float)(times[index+1]-times[index]));
  return pos;
}

float Interpolator::getSingleCubicSplineInterp(float * pos,float * vel, float * times, int t){
  for(int i=0;i<TIMESTEPS;i++) vel[i] = vel[i]*0.001; //Rescale from [dist]/ms to [dist]/s
  index=0;
  for(int j=0;j<TIMESTEPS;j++)
  {
    if(times[j]<=t && t<times[j+1]) index = j;
  }
  float s = float(t - times[index])/(times[index+1] - times[index]);
  a = vel[index]*(times[index+1] - times[index]) - (pos[index+1] - pos[index]);
  b = -vel[index+1]*(times[index+1] - times[index]) + (pos[index+1] - pos[index]);
  float P = (1 - s)*pos[index] + s*pos[index+1] + s*(1-s)*(a*(1-s) + b*s);
  return P;
}

void Interpolator::getMultipleZOH(float targets[][MOTORS], float times[], float t, float cmds[]){
  index=0;
  for(int j=0;j<TIMESTEPS;j++)
  {
    if(times[j]<=t && t<times[j+1]) index = j;
  }

  for (int j = 0; j < MOTORS; j++)
    {
      cmds[j] = targets[index][j];
    }

}

void Interpolator::getMultipleLinearInterp(float targets[][MOTORS], float times[], float t, float cmds[]){
  index=0;
  for(int j=0;j<TIMESTEPS;j++)
  {
    if(times[j]<=t && t<times[j+1]) index = j;
  }

  for (int j = 0; j < MOTORS; j++)
    {
      cmds[j] = targets[index][j] + (targets[index + 1][j] - targets[index][j])*(t - times[index]) / (times[index + 1] - times[index]);
    }

}

  void Interpolator::getMultipleCubicSplineInterp(float pos[][MOTORS], float vel[][MOTORS], float times[], float t, float cmds[]){
    // float t = (float) t_ms / 1000;
    index = 0;
    for (int j = 0; j<TIMESTEPS; j++)
    {
      if (times[j] <= t && t<times[j + 1]) 
        {
          index = j;
          break;
        }
    }

    for (int j = 0; j < MOTORS; j++)
    {
      s = float(t - times[index])/(times[index+1] - times[index]);
      a = vel[index][j]*(times[index+1] - times[index]) - (pos[index+1][j] - pos[index][j]);
      b = -vel[index+1][j]*(times[index+1] - times[index]) + (pos[index+1][j] - pos[index][j]);
      cmds[j] = (1 - s)*pos[index][j] + s*pos[index+1][j] + s*(1-s)*(a*(1-s) + b*s);

    }
    // return P;
  }