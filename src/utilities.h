#ifndef UTILITIES_H
#define UTILITIES_H

#include <gsim/gs_vec.h>
#include <gsim/kn_ik_manipulator.h>

#define LEFT_IK 0
#define RIGHT_IK 1
#define ROOT_IK 2
KnIk::Result setIK(float deg,GsVec pos,int IK);
KnIk::Result updateIK();

double get_time ();
void wait ( double seconds );
GsVec midpoint(GsVec a, GsVec b);
float rnd(float j);
long int fact(int n);
GsVec interp(GsVec start, GsVec end , float t);
float interp(float start,float end, float t);
float interp_cubic ( float t, float tmin, float tmax );


#endif