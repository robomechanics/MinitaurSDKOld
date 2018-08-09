#ifndef func_h
#define func_h

#include "SMath.h"

/**
  Forward kinematics, from motor angles to x/y in Cartesian space
*/
void fk(float* out, float q1, float q2)
{
  float qAvg = (q1-q2)/2.0;
  float qDiff = (q1+q2)/2.0;

  float r = -0.1*fastcos(qDiff)+fastsqrt(0.04-0.01*pow(fastsin(qDiff),2));
  out[0] = -r*fastsin(qAvg);  // x
  out[1] = -r*fastcos(qAvg);  // y
  return;
}

/**
  Inverse kinematics, from x/y in Cartesian space to motor angles
*/
void ik(float* out, float x, float y)
{
  float r = fastsqrt(pow(x,2)+pow(y,2));
  float theta = -atan2(y,x)-PI/2;

  float diffAng = PI-acos((pow(r,2)-0.03)/(0.2*r));

  out[0] = diffAng+theta;
  out[1] = diffAng-theta;
  return;
}

void trajVel(float *out, float x,  float y, float xdot, float ydot)
{
  float jac_11 = x/fastsqrt(pow(x,2)+pow(y,2));
  float jac_12 = y/fastsqrt(pow(x,2)+pow(y,2));
  float jac_21 = -1/(y*(pow(x,2)/pow(y,2))+1);
  float jac_22 = x/(pow(y,2)*(pow(x,2)/pow(y,2))+1);

  out[0] = jac_11*xdot+jac_12*ydot;
  out[1] = jac_21*xdot+jac_22*ydot;
  return;
}

/**
  Convert extension/angle to x/y
*/
void extToX(float *out, float ext, float ang)
{
  out[0] = ext*fastsin(-ang);
  out[1] = -ext*fastcos(-ang);
  return;
}

/**
  Convert extension/angle to motor angles
*/
void extToQ(float *out, float ext, float ang)
{
  float x = ext*fastsin(-ang);
  float y = -ext*fastcos(-ang);
  float r = fastsqrt(pow(x,2)+pow(y,2));
  float theta = -atan2(y,x)-PI/2;
  float diffAng = PI-acos((pow(r,2)-0.03)/(0.2*r));

  out[0] = diffAng+theta;
  out[1] = diffAng-theta;
  return;
}

/**
  Convert theta1 (gen coord) to q1 (motor angle)
*/
float toQ1(float theta)
{
  float q = HALF_PI+theta;
  return q;
}

/**
  Convert theta2 (gen coord) to q2 (motor angle)
*/
float toQ2(float theta)
{
  float q = 3*PI/2-theta;
  return q;
}

/**
  Convert q1 (motor angle) to theta1 (gen coord)
*/
float toT1(float q)
{
  float theta = -HALF_PI+q;
  return theta;
}

/**
  Convert q2 (motor angle) to theta2 (gen coord)
*/
float toT2(float q)
{
  float theta = 3*PI/2-q;
  return theta;
}

/**
  Get triangular trajectory using interpolation based on parameters
  @ ap, apex height
  @ td, touchdown height
  @ ang, half of the top angle of the triangle
  @ t1, half of the time at the bottom/ground
  @ t2, time at one side

  Output: qIn_ref, qOut_ref
*/
void getTrajInterp(float* qIn_ref, float* qOut_ref, float ap, float td, float ang, int t1, int t2)
{
  ang = ang*PI/180;

  float out[2];
  float x_ref;
  float y_ref;
  float l1 = td*fastsin(ang);
  float r = fastsqrt(pow(td,2)-pow(l1,2))-ap;

  float deltaX_bot = l1/t1;
  // left bottom
  for(int i = 0; i < t1; i++)
  {
    x_ref = -deltaX_bot*i;
    y_ref = -(ap+r);
    ik(out, x_ref, y_ref);
    qIn_ref[i] = out[0];
    qOut_ref[i] = out[1];
  }

  float deltaX_side = l1/t2;
  float deltaY_side = r/t2;

  // left side
  for(int i = 0; i < t2; i++)
  {
    x_ref = -l1+deltaX_side*i;
    y_ref = -(ap+r)+deltaY_side*i;
    ik(out, x_ref, y_ref);
    qIn_ref[t1+i] = out[0];
    qOut_ref[t1+i] = out[1];
  }

  // right side
  for(int i = 0; i < t2; i++)
  {
    x_ref = deltaX_side*i;
    y_ref = -ap-deltaY_side*i;
    ik(out, x_ref, y_ref);
    qIn_ref[t1+t2+i] = out[0];
    qOut_ref[t1+t2+i] = out[1];
  }

  // right bottom
  for(int i = 0; i < t1; i++)
  {
    x_ref = l1-deltaX_bot*i;
    y_ref = -(ap+r);
    ik(out, x_ref, y_ref);
    qIn_ref[t1+t2+t2+i] = out[0];
    qOut_ref[t1+t2+t2+i] = out[1];
  }
  return;
}

/**
  Get triangular trajectory for trotting on the stair. The trajectory is rotated to match the slope of the stairs.

  Note that the method is not mathematically correct. Just happened to work for a top angle of 30 degrees.

  @ ap, apex height
  @ td, touchdown height
  @ ang, half of the top angle of the triangle
  @ t1, half of the time at the bottom/ground
  @ t2, time at one side

  Output: qIn_ref, qOut_ref
*/
void getTrajRotInterp(float* qIn_ref, float* qOut_ref, float ap, float td, float ang, int t1, int t2)
{
  ang = ang*PI/180;

  float l1 = td*fastsin(ang);
  float r = fastsqrt(pow(td,2)-pow(l1,2))-ap;
  float rot = -atan2(6, 12);  // stair angle, used to rotate the triangle

  float deltaX_bot = l1/t1;
  // left bottom
  for(int i = 0; i < t1; i++)
  {
    float x_ori = -deltaX_bot*i;
    float y_ori = -(ap+r);

    float ori = atan2(-x_ori, y_ori);
    float dia = sqrt(pow(x_ori,2)+pow(y_ori,2));
    float x_ref = dia*sin(ori+rot);
    float y_ref = dia*cos(ori+dia);

    float out[2];
    ik(out, x_ref, y_ref);
    qIn_ref[i] = out[0];
    qOut_ref[i] = out[1];
  }

  float deltaX_side = l1/t2;
  float deltaY_side = r/t2;

  // left side
  for(int i = 0; i < t2; i++)
  {
    float x_ori = -l1+deltaX_side*i;
    float y_ori = -(ap+r)+deltaY_side*i;

    float ori = atan2(-x_ori, y_ori);
    float dia = sqrt(pow(x_ori,2)+pow(y_ori,2));
    float x_ref = dia*sin(ori+rot);
    float y_ref = dia*cos(ori+dia);

    float out[2];
    ik(out, x_ref, y_ref);
    qIn_ref[t1+i] = out[0];
    qOut_ref[t1+i] = out[1];
  }

  // right side
  for(int i = 0; i < t2; i++)
  {
    float x_ori = deltaX_side*i;
    float y_ori = -ap-deltaY_side*i;

    float ori = atan2(-x_ori, y_ori);
    float dia = sqrt(pow(x_ori,2)+pow(y_ori,2));
    float x_ref = dia*sin(ori+rot);
    float y_ref = dia*cos(ori+dia);

    float out[2];
    ik(out, x_ref, y_ref);
    qIn_ref[t1+t2+i] = out[0];
    qOut_ref[t1+t2+i] = out[1];
  }

  // right bottom
  for(int i = 0; i < t1; i++)
  {
    float x_ori = l1-deltaX_bot*i;
    float y_ori = -(ap+r);

    float ori = atan2(-x_ori, y_ori);
    float dia = sqrt(pow(x_ori,2)+pow(y_ori,2));
    float x_ref = dia*sin(ori+rot);
    float y_ref = dia*cos(ori+dia);

    float out[2];
    ik(out, x_ref, y_ref);
    qIn_ref[t1+t2+t2+i] = out[0];
    qOut_ref[t1+t2+t2+i] = out[1];
  }
  return;
}

float toRad(float ext)
{
  float rad = 3.14-acos((0.03-pow(ext,2))/(-0.2*ext));
  return rad;
}

#endif