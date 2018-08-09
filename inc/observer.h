#ifndef observer_h
#define observer_h

#include "SMath.h"

/**
 * Online leg observer. Ignore Corioslis, joint friction, motor friction (static/kinetic/viscous), used mid-point integration.
 * 
 * @ Vbatt: battery voltage, read using S->batt.voltage
 * @ PWM1: PWM of the inner motor from getOpenLoop()
 * @ PWM2: PWM of the outer motor from getOpenLoop()
 * @ index: index of the leg, 0-3
 * @ step: integration time step size
 * 
 * Output:
 * @ pos: expected position
 * @ vel: expected velocity
 * 
*/
void obs(float* pos, float* vel, float Vbatt, float PWM1, float PWM2, int index, float step)
{
  int indexIn = 2*index;
  int indexOut = 2*index+1;

  float t1 = pos[indexIn];
  float t2 = pos[indexOut];
  float t1dot = vel[indexIn];
  float t2dot = vel[indexOut];

  float f1 = fastsin(t1);  // faster than standard sin/cos
  float f2 = fastcos(t1);
  float f3 = fastsin(t2);
  float f4 = fastcos(t2);

  float s1 = f2*f4+f1*f3;  // fastcos(t1-t2);
  float s2, f5;
  float c1 = fmodf_mpi_pi((t1-t2)/2);  // wrap to -pi to pi
  if(c1 >= HALF_PI)  // positive sin, negative cos
  {
    s2 = -sqrtf((1+s1)/2);  // fastcos(t1*(1.0/2.0)-t2*(1.0/2.0));
    f5 = sqrtf((1-s1)/2);   // fastsin(t1*(1.0/2.0)-t2*(1.0/2.0));
  } 
  else if(c1 < HALF_PI && c1 >= 0)  // positive sin/cos
  {
    s2 = sqrtf((1+s1)/2);  // sqrtf is slightly faster than fastsqrt, see the header file
    f5 = sqrtf((1-s1)/2);
  }
  else if(c1 < 0 && c1 >= -HALF_PI)  // negative sin, positive cos
  {
    s2 = sqrtf((1+s1)/2);
    f5 = -sqrtf((1-s1)/2);   
  }
  else  // negative sin/cos
  {
    s2 = -sqrtf((1+s1)/2);
    f5 = -sqrtf((1-s1)/2);   
  }
  float s3 = sqrtf(s1+7.0);
  float f6 = 0.3535*s3;  // cos(asin(f5*0.5));

  float s5 = f6*s2-(f5*0.5)*f5;  // fastcos(s4+t1*(1.0/2.0)-t2*(1.0/2.0))
  float s6 = (f5*0.5)*s2+f6*f5;  // fastsin(s4+t1*(1.0/2.0)-t2*(1.0/2.0))

  float p1 = 1.414*1.0/s3;
  float p2 = f2*s2*s5*8.1E1-f4*s2*s5*8.1E1;
  float p3 = s2*f1*s6*8.1E1;
  float p4 = s2*f3*s6*8.1E1+1.414*f4*s3*1.01E2+1.414*f2*s5*s3*4.05E1+1.414*f4*s5*s3*4.05E1+1.414*f1*s6*s3*4.05E1-1.414*f3*s6*s3*4.05E1;

  // mass matrix
  float s7 = f1*f4-f2*f3;  // fastsin(t1-t2)
  float s8 = s1*s2-s7*f5;  // fastcos(1.5*(t1-t2)) expand
  // float M1 = (s1*2.015E-3+1.414*s2*s3*1.62E-3+1.0175E-2)/(s1+7.0)+6.0E-5;  // M(1,1) and M(2,2)
  // float M2 = (s1*2.025E-4+s1*2.835E-3+1.414*s2*s3*1.4175E-3+1.414*s8*s3*2.025E-4+4.1325E-3)/(s1+7.0);  // M(1,2) and M(2,1)
  float M1 = (s1*2.225E-3+1.414*s2*s3*1.62E-3+1.1105E-2)/(s1+7.0)+5.0E-5;  // M(1,1) and M(2,2)
  float M2 = (s1*2.025E-4+s1*2.835E-3+1.414*s2*s3*1.4175E-3+1.414*s8*s3*2.025E-4+4.6725E-3)/(s1+7.0);  // M(1,2) and M(2,1)

  // gravity
  float N1 = p1*(p2-p3-p4)*(-4.905E-4);
  float N2 = p1*(p2+p3+p4)*(-4.905E-4);

  float R = 0.180;  // motor resistance
  float Kt = 0.0959;  // torque constant
  float B1 = Kt*(0.95*Vbatt*PWM1 - Kt*t1dot)/R - N1;
  float B2 = Kt*(0.95*Vbatt*PWM2 - Kt*t2dot)/R - N2;  // tau - N

  // solve for the acceleration, two equations and two variables
  float t1dotdot = (B1-M2*((B1+B2)/(M1+M2)))/(M1-M2);
  float t2dotdot = (B1-t1dotdot*M1)/M2;

  vel[indexIn]  = t1dot+t1dotdot*step;
  vel[indexOut] = t2dot+t2dotdot*step;
  // pos[indexIn]  = t1+vel[indexIn]*step;
  // pos[indexOut] = t2+vel[indexOut]*step;
  pos[indexIn]  = t1+(vel[indexIn]+t1dot)*0.5*step;
  pos[indexOut] = t2+(vel[indexOut]+t2dot)*0.5*step;
  return;
}

/**
 * Online leg observer. Ignore Corioslis, joint friction, motor friction (static/kinetic/viscous), used Verlet integration.
 * 
 * @ Vbatt: battery voltage, read using S->batt.voltage
 * @ PWM1: PWM of the inner motor from getOpenLoop()
 * @ PWM2: PWM of the outer motor from getOpenLoop()
 * @ index: index of the leg, 0-3
 * @ step: integration time step size
 * 
 * Output:
 * @ pos: expected position
 * @ vel: expected velocity
 * 
*/void obs_ver(float* pos, float* vel, float Vbatt, float PWM1, float PWM2, int index, float step, float* prev)
{
  int indexIn = 2*index;
  int indexOut = 2*index+1;

  float t1 = pos[indexIn];
  float t2 = pos[indexOut];
  float t1dot = vel[indexIn];
  float t2dot = vel[indexOut];
  float t1_prev = prev[indexIn];
  float t2_prev = prev[indexOut];

  float f1 = fastsin(t1);  // faster than standard sin/cos
  float f2 = fastcos(t1);
  float f3 = fastsin(t2);
  float f4 = fastcos(t2);

  float s1 = f2*f4+f1*f3;  // fastcos(t1-t2);
  float s2, f5;
  float c1 = fmodf_mpi_pi((t1-t2)/2);  // wrap to -pi to pi
  if(c1 >= HALF_PI)  // positive sin, negative cos
  {
    s2 = -sqrtf((1+s1)/2);  // fastcos(t1*(1.0/2.0)-t2*(1.0/2.0));
    f5 = sqrtf((1-s1)/2);   // fastsin(t1*(1.0/2.0)-t2*(1.0/2.0));
  } 
  else if(c1 < HALF_PI && c1 >= 0)  // positive sin/cos
  {
    s2 = sqrtf((1+s1)/2);  // sqrtf is slightly faster than fastsqrt, see the header file
    f5 = sqrtf((1-s1)/2);
  }
  else if(c1 < 0 && c1 >= -HALF_PI)  // negative sin, positive cos
  {
    s2 = sqrtf((1+s1)/2);
    f5 = -sqrtf((1-s1)/2);   
  }
  else  // negative sin/cos
  {
    s2 = -sqrtf((1+s1)/2);
    f5 = -sqrtf((1-s1)/2);   
  }
  float s3 = sqrtf(s1+7.0);
  float f6 = 0.3535*s3;  // cos(asin(f5*0.5));

  float s5 = f6*s2-(f5*0.5)*f5;  // fastcos(s4+t1*(1.0/2.0)-t2*(1.0/2.0))
  float s6 = (f5*0.5)*s2+f6*f5;  // fastsin(s4+t1*(1.0/2.0)-t2*(1.0/2.0))

  float p1 = 1.414*1.0/s3;
  float p2 = f2*s2*s5*8.1E1-f4*s2*s5*8.1E1;
  float p3 = s2*f1*s6*8.1E1;
  float p4 = s2*f3*s6*8.1E1+1.414*f4*s3*1.01E2+1.414*f2*s5*s3*4.05E1+1.414*f4*s5*s3*4.05E1+1.414*f1*s6*s3*4.05E1-1.414*f3*s6*s3*4.05E1;

  // mass matrix
  float s7 = f1*f4-f2*f3;  // fastsin(t1-t2)
  float s8 = s1*s2-s7*f5;  // fastcos(1.5*(t1-t2)) expand
  // float M1 = (s1*2.015E-3+1.414*s2*s3*1.62E-3+1.0175E-2)/(s1+7.0)+6.0E-5;  // M(1,1) and M(2,2)
  // float M2 = (s1*2.025E-4+s1*2.835E-3+1.414*s2*s3*1.4175E-3+1.414*s8*s3*2.025E-4+4.1325E-3)/(s1+7.0);  // M(1,2) and M(2,1)
  float M1 = (s1*2.225E-3+1.414*s2*s3*1.62E-3+1.1105E-2)/(s1+7.0)+5.0E-5;  // M(1,1) and M(2,2)
  float M2 = (s1*2.025E-4+s1*2.835E-3+1.414*s2*s3*1.4175E-3+1.414*s8*s3*2.025E-4+4.6725E-3)/(s1+7.0);  // M(1,2) and M(2,1)

  // gravity
  float N1 = p1*(p2-p3-p4)*(-4.905E-4);
  float N2 = p1*(p2+p3+p4)*(-4.905E-4);

  float R = 0.180;
  float Kt = 0.0959;
  float B1 = Kt*(0.95*Vbatt*PWM1 - Kt*t1dot)/R - N1;
  float B2 = Kt*(0.95*Vbatt*PWM2 - Kt*t2dot)/R - N2;  // tau - N

  float t1dotdot = (B1-M2*((B1+B2)/(M1+M2)))/(M1-M2);
  float t2dotdot = (B1-t1dotdot*M1)/M2;

  vel[indexIn]  = t1dot+t1dotdot*step;
  vel[indexOut] = t2dot+t2dotdot*step;

  // Verlet integration
  pos[indexIn] = 2*t1-t1_prev+t1dotdot*step*step;
  pos[indexOut] = 2*t2-t2_prev+t2dotdot*step*step;
  return;
}

#endif

  // float PWM1 = 0.95*constrain((kp*(t1c-t1)+kd*(-t1dot)),-1,1);
  // float PWM2 = 0.95*constrain((kp*(t2c-t2)+kd*(-t2dot)),-1,1);

  // float N1 = 1.414*1.0/fastsqrt(fastcos(t1-t2)+7.0)*
  // (fastcos(t1)*fastcos(t1*(1.0/2.0)-t2*(1.0/2.0))*
  //  fastcos(asin(fastsin(t1*(1.0/2.0)-t2*(1.0/2.0))*(1.0/2.0))+
  //  t1*(1.0/2.0)-t2*(1.0/2.0))*-8.1E1+fastcos(t2)*
  //  fastcos(t1*(1.0/2.0)-t2*(1.0/2.0))*
  //  fastcos(asin(fastsin(t1*(1.0/2.0)-t2*(1.0/2.0))*(1.0/2.0))+t1*(1.0/2.0)-t2*(1.0/2.0))*8.1E1
  //  -
  //  fastcos(t1*(1.0/2.0)-t2*(1.0/2.0))*fastsin(t1)*fastsin(asin(fastsin(t1*(1.0/2.0)-
  //  t2*(1.0/2.0))*(1.0/2.0))+t1*(1.0/2.0)-t2*(1.0/2.0))*8.1E1
  //  -
  //  fastcos(t1*(1.0/2.0)-t2*(1.0/2.0))*fastsin(t2)*fastsin(asin(fastsin(t1*(1.0/2.0)-t2*(1.0/2.0))*(1.0/2.0))+
  //  t1*(1.0/2.0)-t2*(1.0/2.0))*8.1E1+1.414*fastcos(t1)*fastsqrt(fastcos(t1-t2)+7.0)*1.01E2+
  //  1.414*fastcos(t1)*fastcos(asin(fastsin(t1*(1.0/2.0)-t2*(1.0/2.0))*(1.0/2.0))+
  //  t1*(1.0/2.0)-t2*(1.0/2.0))*fastsqrt(fastcos(t1-t2)+7.0)*(8.1E1/2.0)+
  //  1.414*fastcos(t2)*fastcos(asin(fastsin(t1*(1.0/2.0)-t2*(1.0/2.0))*(1.0/2.0))+
  //  t1*(1.0/2.0)-t2*(1.0/2.0))*fastsqrt(fastcos(t1-t2)+7.0)*(8.1E1/2.0)+
  //  1.414*fastsin(t1)*fastsin(asin(fastsin(t1*(1.0/2.0)-
  //  t2*(1.0/2.0))*(1.0/2.0))+t1*(1.0/2.0)-t2*(1.0/2.0))*fastsqrt(fastcos(t1-t2)+7.0)*(8.1E1/2.0)-
  //  1.414*fastsin(t2)*fastsin(asin(fastsin(t1*(1.0/2.0)-
  //  t2*(1.0/2.0))*(1.0/2.0))+t1*(1.0/2.0)-t2*(1.0/2.0))*fastsqrt(fastcos(t1-t2)+7.0)*(8.1E1/2.0))*(-4.905E-4);



  // float N2 = 1.414*1.0/fastsqrt(fastcos(t1-t2)+7.0)*
  // (fastcos(t1)*fastcos(t1*(1.0/2.0)-t2*(1.0/2.0))*
  //  fastcos(asin(fastsin(t1*(1.0/2.0)-t2*(1.0/2.0))*(1.0/2.0))+
  //  t1*(1.0/2.0)-t2*(1.0/2.0))*8.1E1-fastcos(t2)*
  //  fastcos(t1*(1.0/2.0)-t2*(1.0/2.0))*
  //  fastcos(asin(fastsin(t1*(1.0/2.0)-t2*(1.0/2.0))*(1.0/2.0))+t1*(1.0/2.0)-t2*(1.0/2.0))*8.1E1
  //  +
  //  fastcos(t1*(1.0/2.0)-t2*(1.0/2.0))*fastsin(t1)*fastsin(asin(fastsin(t1*(1.0/2.0)-
  //  t2*(1.0/2.0))*(1.0/2.0))+t1*(1.0/2.0)-t2*(1.0/2.0))*8.1E1
  //  +
  //  fastcos(t1*(1.0/2.0)-t2*(1.0/2.0))*fastsin(t2)*fastin(asin(fastsin(t1*(1.0/2.0)-t2*(1.0/2.0))*(1.0/2.0))+
  //  t1*(1.0/2.0)-t2*(1.0/2.0))*8.1E1+1.414*fastcos(t2)*fastsqrt(fastcos(t1-t2)+7.0)*1.01E2+
  //  1.414*fastcos(t1)*fastcos(asin(fastsin(t1*(1.0/2.0)-t2*(1.0/2.0))*(1.0/2.0))+
  //  t1*(1.0/2.0)-t2*(1.0/2.0))*fastsqrt(fastcos(t1-t2)+7.0)*(8.1E1/2.0)+
  //  1.414*fastcos(t2)*fastcos(asin(fastsin(t1*(1.0/2.0)-t2*(1.0/2.0))*(1.0/2.0))+
  //  t1*(1.0/2.0)-t2*(1.0/2.0))*fastsqrt(fastcos(t1-t2)+7.0)*(8.1E1/2.0)+
  //  1.414*fastsin(t1)*fastsin(asin(fastsin(t1*(1.0/2.0)-
  //  t2*(1.0/2.0))*(1.0/2.0))+t1*(1.0/2.0)-t2*(1.0/2.0))*fastsqrt(fastcos(t1-t2)+7.0)*(8.1E1/2.0)-
  //  1.414*fastsin(t2)*fastsin(asin(fastsin(t1*(1.0/2.0)-
  //  t2*(1.0/2.0))*(1.0/2.0))+t1*(1.0/2.0)-t2*(1.0/2.0))*fastsqrt(fastcos(t1-t2)+7.0)*(8.1E1/2.0))*(-4.905E-4);