////////////////////////////////////////////////////////////////////////////////
// [M.V.] That are my additions to linmath library adding functions I needed
// for the autopilot.

#ifndef LINMATH2_H
#define LINMATH2_H

#include "linmath.h"

#define vecToString_st(x) arrayWithDimToStr_st((x), DIM(x))
#define quatToString_st(x) arrayWithDimToStr_st((x), 4)
#define vec2ToString_st(x) arrayWithDimToStr_st((x), 2)
#define vec3ToString_st(x) arrayWithDimToStr_st((x), 3)
#define vec4ToString_st(x) arrayWithDimToStr_st((x), 4)
#define PRINT_VEC_ARRAY(x, size) {printf("%s[%d]:\n", #x, size);int _i; for(_i=0;_i<size;_i++) {printf("%s\n", vecToString_st(x[_i]));} printf("\n");}

typedef vec3 mat3x3[3];

enum rotationsSequence {RS_ZYX, RS_ZYZ, RS_ZXY, RS_ZXZ, RS_YXZ, RS_YXY, RS_YZX, RS_YZY, RS_XYZ, RS_XYX, RS_XZY, RS_XZX};

//////////////////

LINMATH_H_FUNC void mat3x3_mul_vec3(vec3 r, mat3x3 M, vec3 v)
{
	int i, j;
	for(j=0; j<3; ++j) {
		r[j] = 0.f;
		for(i=0; i<3; ++i)
			r[j] += M[i][j] * v[i];
	}
}


LINMATH_H_FUNC void mat3x3_from_quat(mat3x3 r, quat q) {
    double q0, q1, q2, q3;

    q0 = q[0];
    q1 = q[1];
    q2 = q[2];
    q3 = q[3];
     
    // First row of the rotation matrix
    r[0][0] = 2 * (q0 * q0 + q1 * q1) - 1;
    r[1][0] = 2 * (q1 * q2 - q0 * q3);
    r[2][0] = 2 * (q1 * q3 + q0 * q2);
     
    // Second row of the rotation matrix
    r[0][1] = 2 * (q1 * q2 + q0 * q3);
    r[1][1] = 2 * (q0 * q0 + q2 * q2) - 1;
    r[2][1] = 2 * (q2 * q3 - q0 * q1);
     
    // Third row of the rotation matrix
    r[0][2] = 2 * (q1 * q3 - q0 * q2);
    r[1][2] = 2 * (q2 * q3 + q0 * q1);
    r[2][2] = 2 * (q0 * q0 + q3 * q3) - 1;

#if 0 
1 - 2*qy2 - 2*qz2	2*qx*qy - 2*qz*qw	2*qx*qz + 2*qy*qw
2*qx*qy + 2*qz*qw	1 - 2*qx2 - 2*qz2	2*qy*qz - 2*qx*qw
2*qx*qz - 2*qy*qw	2*qy*qz + 2*qx*qw	1 - 2*qx2 - 2*qy2
#endif
}

LINMATH_H_FUNC void twoaxisrot(double r11, double r12, double r21, double r31, double r32, double res[]){
  res[0] = atan2( r11, r12 );
  res[1] = acos ( r21 );
  res[2] = atan2( r31, r32 );
}

LINMATH_H_FUNC void threeaxisrot(double r11, double r12, double r21, double r31, double r32, double res[]){
  res[0] = atan2( r31, r32 );
  res[1] = asin ( r21 );
  res[2] = atan2( r11, r12 );
}

LINMATH_H_FUNC void quat_inverse(quat r, quat q) {
    double f;
    f = 1 / (q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    r[0] = -q[0]*f;
    r[1] = -q[1]*f;
    r[2] = -q[2]*f;
    r[3] = q[3]*f;
}

LINMATH_H_FUNC void quat_div(quat r, quat p, quat q) {
    quat ii;
    quat_inverse(ii, q);
    quat_mul(r, p, ii);
}

LINMATH_H_FUNC void quaternion_to_euler(quat q, double res[], int rotSeq) {
    double x,y,z,w;

    x = q[0];
    y = q[1];
    z = q[2];
    w = q[3];
    
    switch(rotSeq){
    case RS_ZYX:
      threeaxisrot( 2*(x*y + w*z),
                     w*w + x*x - y*y - z*z,
                    -2*(x*z - w*y),
                     2*(y*z + w*x),
                     w*w - x*x - y*y + z*z,
                     res);
      break;

    case RS_ZYZ:
      twoaxisrot( 2*(y*z - w*x),
                   2*(x*z + w*y),
                   w*w - x*x - y*y + z*z,
                   2*(y*z + w*x),
                  -2*(x*z - w*y),
                  res);
      break;

    case RS_ZXY:
      threeaxisrot( -2*(x*y - w*z),
                      w*w - x*x + y*y - z*z,
                      2*(y*z + w*x),
                     -2*(x*z - w*y),
                      w*w - x*x - y*y + z*z,
                      res);
      break;

    case RS_ZXZ:
      twoaxisrot( 2*(x*z + w*y),
                  -2*(y*z - w*x),
                   w*w - x*x - y*y + z*z,
                   2*(x*z - w*y),
                   2*(y*z + w*x),
                   res);
      break;

    case RS_YXZ:
      threeaxisrot( 2*(x*z + w*y),
                     w*w - x*x - y*y + z*z,
                    -2*(y*z - w*x),
                     2*(x*y + w*z),
                     w*w - x*x + y*y - z*z,
                     res);
      break;

    case RS_YXY:
      twoaxisrot( 2*(x*y - w*z),
                   2*(y*z + w*x),
                   w*w - x*x + y*y - z*z,
                   2*(x*y + w*z),
                  -2*(y*z - w*x),
                  res);
      break;

    case RS_YZX:
      threeaxisrot( -2*(x*z - w*y),
                      w*w + x*x - y*y - z*z,
                      2*(x*y + w*z),
                     -2*(y*z - w*x),
                      w*w - x*x + y*y - z*z,
                      res);
      break;

    case RS_YZY:
      twoaxisrot( 2*(y*z + w*x),
                  -2*(x*y - w*z),
                   w*w - x*x + y*y - z*z,
                   2*(y*z - w*x),
                   2*(x*y + w*z),
                   res);
      break;

    case RS_XYZ:
      threeaxisrot( -2*(y*z - w*x),
                    w*w - x*x - y*y + z*z,
                    2*(x*z + w*y),
                   -2*(x*y - w*z),
                    w*w + x*x - y*y - z*z,
                    res);
      break;

    case RS_XYX:
      twoaxisrot( 2*(x*y + w*z),
                  -2*(x*z - w*y),
                   w*w + x*x - y*y - z*z,
                   2*(x*y - w*z),
                   2*(x*z + w*y),
                   res);
      break;

    case RS_XZY:
      threeaxisrot( 2*(y*z + w*x),
                     w*w - x*x + y*y - z*z,
                    -2*(x*y - w*z),
                     2*(x*z + w*y),
                     w*w + x*x - y*y - z*z,
                     res);
      break;

    case RS_XZX:
      twoaxisrot( 2*(x*z - w*y),
                   2*(x*y + w*z),
                   w*w + x*x - y*y - z*z,
                   2*(x*z + w*y),
                  -2*(x*y - w*z),
                  res);
      break;
    default:
	printf("%s:%d: Internal error: Unknown rotation sequence %d\n", __FILE__, __LINE__, rotSeq);
      break;
   }
}


#endif
