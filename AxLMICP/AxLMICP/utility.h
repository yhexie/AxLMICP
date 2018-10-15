#ifndef  EULER
#define  EULER


#include <windows.h>
#define  _USE_MATH_DEFINES
#include <math.h>
/**
 * Converts a pose into a RT matrix
 * @param *rPos Pointer to the position (double[3])
 * @param *rPosTheta Pointer to the angles (double[3])
 * @param *alignxf The calculated matrix
 *注意是列主序
 */
inline void EulerToMatrix4(const double *rPos, const double *rPosTheta, double *alignxf)
{
	double sx = sin(rPosTheta[0]);
	double cx = cos(rPosTheta[0]);
	double sy = sin(rPosTheta[1]);
	double cy = cos(rPosTheta[1]);
	double sz = sin(rPosTheta[2]);
	double cz = cos(rPosTheta[2]);

	alignxf[0] = cy*cz;
	alignxf[1] = sx*sy*cz + cx*sz;
	alignxf[2] = -cx*sy*cz + sx*sz;
	alignxf[3] = 0.0;
	alignxf[4] = -cy*sz;
	alignxf[5] = -sx*sy*sz + cx*cz;
	alignxf[6] = cx*sy*sz + sx*cz;
	alignxf[7] = 0.0;
	alignxf[8] = sy;
	alignxf[9] = -sx*cy;
	alignxf[10] = cx*cy;

	alignxf[11] = 0.0;

	alignxf[12] = rPos[0];
	alignxf[13] = rPos[1];
	alignxf[14] = rPos[2];
	alignxf[15] = 1;
}

/**
 * Converts a 4x4 matrix to Euler angles.
 *
 * @param alignxf    input 4x4 matrix
 * @param rPosTheta  output 3-vector of Euler angles
 * @param rPos       output vector of trnaslation (position) if set
 *
 */
static inline void Matrix4ToEuler(const double *alignxf, double *rPosTheta, double *rPos = 0)
{

	double _trX, _trY;

	// Calculate Y-axis angle 
	if (alignxf[0] > 0.0) {
		rPosTheta[1] = asin(alignxf[8]);
	}
	else {
		rPosTheta[1] = M_PI - asin(alignxf[8]);
	}

	double  C = cos(rPosTheta[1]);
	if (fabs(C) > 0.005)  {                 // Gimball lock? 
		_trX = alignxf[10] / C;             // No, so get X-axis angle 
		_trY = -alignxf[9] / C;
		rPosTheta[0] = atan2(_trY, _trX);
		_trX = alignxf[0] / C;              // Get Z-axis angle 
		_trY = -alignxf[4] / C;
		rPosTheta[2] = atan2(_trY, _trX);
	}
	else {                                    // Gimball lock has occurred 
		rPosTheta[0] = 0.0;                       // Set X-axis angle to zero 
		_trX = alignxf[5];  //1                // And calculate Z-axis angle 
		_trY = alignxf[1];  //2
		rPosTheta[2] = atan2(_trY, _trX);
	}

	rPosTheta[0] = rPosTheta[0];
	rPosTheta[1] = rPosTheta[1];
	rPosTheta[2] = rPosTheta[2];

	if (rPos != 0) {
		rPos[0] = alignxf[12];
		rPos[1] = alignxf[13];
		rPos[2] = alignxf[14];
	}
}

#endif // ! EULER