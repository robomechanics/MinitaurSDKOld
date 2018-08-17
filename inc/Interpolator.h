// Joe Norby

#ifndef INTERPOLATOR_H
#define INTERPOLATOR_H


class Interpolator {
 public:
  Interpolator(int);
  float * getInterps(float *, float *);
  float getSingleInterp(float *, float *, int);
  float getSingleZOH(float *, float *, int);
  float getSinglePVTInterp(float *, float *, float *, int);

 private:
  int lengthTime;
  int index;
  float a,b;
};

#endif
