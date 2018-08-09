// Joe Norby

#ifndef INTERPOLATOR_H
#define INTERPOLATOR_H

#include <cmath>
using namespace std;

class Interpolator {
 public:
  Interpolator(const int a)
  {
    lengthTime = a;
  }

  float * getInterps(float * targets, float * times)
  {
    int tf = times[lengthTime-1];
    float pos[tf];
    for(int i=0;i<tf;++i)
    {    
      index=0;
      int t = i;
        for(int j=0;j<lengthTime;j++)
        {
          if(times[j]<=t && t<times[j+1]) index = j;
        }
        pos[i] = targets[index] + (targets[index+1]-targets[index])*((float)(t-times[index]))/((float)(times[index+1]-times[index]));
    }
    float* ptr = pos;
    return ptr;
  }

  float getfastsingleInterp(float * targets, float * times, int t)
  {
    float pos;
    index=0;
    for(int j=0;j<lengthTime;j++)
    {
      if(times[j]<=t && t<times[j+1]) index = j;
    }
    pos = targets[index] + (targets[index+1]-targets[index])*((float)(t-times[index]))/((float)(times[index+1]-times[index]));
    return pos;
  }

  float getfastsingleZOH(float * targets, float * times, int t) {
    float pos;
    index=0;
    for(int j=0;j<lengthTime;j++)
    {
      if(times[j]<=t && t<times[j+1]) index = j;
    }
    pos = targets[index];
    return pos;
  }

  float getfastsinglePVTInterp(float * pos,float * vel, float * times, int t){
    for(int i=0;i<lengthTime;i++) vel[i] = vel[i]*0.001; //Rescale from [dist]/ms to [dist]/s
    index=0;
    for(int j=0;j<lengthTime;j++)
    {
      if(times[j]<=t && t<times[j+1]) index = j;
    }
    float s = float(t - times[index])/(times[index+1] - times[index]);
    a = vel[index]*(times[index+1] - times[index]) - (pos[index+1] - pos[index]);
    b = -vel[index+1]*(times[index+1] - times[index]) + (pos[index+1] - pos[index]);
    float P = (1 - s)*pos[index] + s*pos[index+1] + s*(1-s)*(a*(1-s) + b*s);

    return P;
  }

 private:
  int lengthTime;
  int index;
  float a,b;
};

#endif
