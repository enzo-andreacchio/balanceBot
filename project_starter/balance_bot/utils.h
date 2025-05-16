#ifndef UTILS_H_
#define UTILS_H_

#include <Eigen/Dense>  // for Matrix3d and Vector3d
#include <cmath>        // for std::abs, etc.
#include <stdio.h>
#include <math.h>

using namespace std;
using namespace Eigen;


Matrix3d crossProductOperator(Vector3d& v);

Vector3d momentsToPositions(Vector3d& zP, Vector3d& M, double m, double g);

float sgn(float x);

std::tuple<VectorXd, VectorXd> drawSpiralAlternate(float freq, float time, float r1, float r2, float num_coils);

std::tuple<VectorXd, VectorXd> drawCircleAlternate(float freq, float time, float radius);

std::tuple<VectorXd, VectorXd> drawCircle(float freq, float time, float radius);

std::tuple<VectorXd, VectorXd> drawTripleSpiral(float freq, float time, float radius);

#endif // UTILS_H_
