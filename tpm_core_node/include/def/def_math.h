#pragma once
#include "math.h"

#define _2PI	 6.283185307179586f // 2*PI
#define PI		 3.141592653589793f	// PI
#define PI_2	 1.570796326794897f	// PI/2
#define DEG2RAD	 0.017453292519943f	// PI/180
#define RAD2DEG 57.295779513082321f	// 180/PI

static inline void  Vec3_Sub		( FLT outV[3], FLT v1[3], FLT v2[3])
{
	outV[0] = v1[0] - v2[0];
	outV[1] = v1[1] - v2[1];
	outV[2] = v1[2] - v2[2];
}
static inline FLT  Vec3_Dot		( FLT v1  [3], FLT v2[3])
{
	return	v1[0] * v2[0] + 
			v1[1] * v2[1] + 
			v1[2] * v2[2] ;
}
static inline FLT  Vec3_Length		( FLT v1  [3]) 
{
	return sqrt(
			v1[0] * v1[0] +
			v1[1] * v1[1] +
			v1[2] * v1[2] 
			);
}
static inline FLT  Vec3_NormalizeSelf(FLT outV[3])
{
	FLT len = Vec3_Length(outV);
	outV[0] /= len;
	outV[1] /= len;
	outV[2] /= len;	
	return len;
}
static inline void Vec3_Rotate	( FLT outV[3], FLT v1[3], FLT vN[3], FLT angleRad)
{
	//'outV' = 'v1' rotates around vector 'vN' by angle 'angleRad'

	FLT C = cos( angleRad );
  FLT S = sin( angleRad );
	FLT mC = 1 - C;

	Vec3_NormalizeSelf(vN);

	// todo: replace with Matrix?
  outV[0] =	(C + mC * vN[0] * vN[0] )		  * v1[0] +
				(mC * vN[0] * vN[1] - vN[2] * S ) * v1[1] +
				(mC * vN[0] * vN[2] + vN[1] * S ) * v1[2];

	outV[1] =	(mC * vN[0] * vN[1] + vN[2] * S ) * v1[0] +
				( C + mC * vN[1] * vN[1] )		  * v1[1] +
				(mC * vN[1] * vN[2] - vN[0] * S ) * v1[2];

	outV[2] =	(mC * vN[0] * vN[2] - vN[1] * S ) * v1[0] +
				(mC * vN[1] * vN[2] + vN[0] * S ) * v1[1] +
				( C + mC * vN[2] * vN[2] )		  * v1[2] ;
}
static inline void Vec3_RotX		(FLT outV[3], FLT v1[3], FLT angleRad)
{
	FLT vN[3] = { 1,0,0 };
	Vec3_Rotate(outV, v1, vN, angleRad);
}
static inline void Vec3_RotY		(FLT outV[3], FLT v1[3], FLT angleRad)
{
	FLT vN[3] = { 0,1,0 };
	Vec3_Rotate(outV, v1, vN, angleRad);
}
static inline void Vec3_RotZ		(FLT outV[3], FLT v1[3], FLT angleRad)
{
	FLT vN[3] = { 0,0,1 };
	Vec3_Rotate(outV, v1, vN, angleRad);
}

static inline void Vec3_RotZYX  (FLT outV[3], FLT v1[3], FLT radZ, FLT radY, FLT radX)
{
  FLT tmp1[3], tmp2[3];
  Vec3_RotZ(tmp1, v1,   radZ);
  Vec3_RotY(tmp2, tmp1, radY);
  Vec3_RotX(outV, tmp2, radX);
}