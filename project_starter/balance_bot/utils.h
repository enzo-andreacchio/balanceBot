#ifndef UTILS_H_
#define UTILS_H_

#include <Eigen/Dense>  // for Matrix3d and Vector3d
#include <cmath>        // for std::abs, etc.

using namespace std;
using namespace Eigen;


Matrix3d crossProductOperator(Vector3d& v);

Vector3d momentsToPositions(Vector3d& zP, Vector3d& M, double m, double g);

float sgn(float x);


#endif // UTILS_H_
