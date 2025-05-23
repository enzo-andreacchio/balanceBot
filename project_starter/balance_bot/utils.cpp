#include "utils.h"
#include <stdio.h>
#include <math.h>

using namespace std;
using namespace Eigen;


// Cross-product matrix operator
Matrix3d crossProductOperator(Vector3d& v) {
	Matrix3d cross;
	cross <<     0, -v.z(),  v.y(),
			v.z(),     0, -v.x(),
			-v.y(),  v.x(),     0;
	return cross;
}


// Computes position vector from moment vector
Vector3d momentsToPositions(Vector3d& zP, Vector3d& M, double m, double g) {
	zP = zP.normalized(); // Normalize normal vector in case it's not normalized
	Vector3d e_z(0, 0, 1);

	Matrix3d proj = zP * zP.transpose(); // Outer product (projection matrix)
	Vector3d F = proj * (-1.0 * m * g * e_z);
	Vector3d Q_vec = -F;
	Matrix3d Q = crossProductOperator(Q_vec);

	// Compute pseudoinverse using SVD
	JacobiSVD<Matrix3d> svd(Q, ComputeFullU | ComputeFullV);
	Vector3d S = svd.singularValues();
	Matrix3d S_inv = Matrix3d::Zero();
	for (int i = 0; i < 3; ++i) {
		if (S(i) > 1e-6) {
			S_inv(i,i) = 1.0 / S(i);
		}
	}
	Matrix3d Q_pinv = svd.matrixV() * S_inv * svd.matrixU().transpose();

	Vector3d x = Q_pinv * M;
	return x;
}

float sgn(float x) {
	if (x > 0) {
		return 1;
	} else if (x < 0) {
		return -1;
	} else {
		return 0;
	}
}

std::tuple<VectorXd, VectorXd> drawTripleSpiral(float freq, float time, float radius) {


	float period = 1/freq;
	time = std::fmod(time, period);
	float theta = 2 * M_PI *time / period;
	
	
	float r = radius* cos(3*theta);
	
	
	float x = r * cos(theta);
	float y = r * sin(theta);
	
	
	VectorXd pos(2);
	VectorXd vel(2);
	
	
	pos << x, y;
	
	
	float dthetdt = 2 * M_PI/ period;
	
	
	float drdt = -1* radius*sin(3* theta) *3*dthetdt;
	
	
	float dxdt = drdt*cos(theta) + r*-1*sin(theta)*dthetdt;
	float dydt = drdt*sin(theta) + r*cos(theta)*dthetdt;
	
	
	vel << dxdt, dydt;
	
	
	std::tuple<VectorXd, VectorXd> pos_vel = std::make_tuple(pos, vel);
	
	
	return pos_vel;
}
	
	
std::tuple<VectorXd, VectorXd> drawCircle(float freq, float time, float radius) {
	
	
	float period = 1/freq;
	time = std::fmod(time, period);
	float theta = 2 * M_PI *time / period;
	
	
	float x = radius * cos(theta);
	float y = radius * sin(theta);
	
	
	float dthetdt = 2 * M_PI/ period;
	
	
	VectorXd pos(2);
	VectorXd vel(2);
	
	
	pos << x, y;
	
	
	float dxdt = -1*radius*sin(theta)*dthetdt;
	float dydt = -1*radius*cos(theta)*dthetdt;
	
	
	vel << dxdt, dydt;
	
	
	std::tuple<VectorXd, VectorXd> pos_vel = std::make_tuple(pos, vel);
	
	
	return pos_vel;
}
	
std::tuple<VectorXd, VectorXd> drawCircleAlternate(float freq, float time, float radius) {
	
	
	float period = 1/freq;
	float count = (static_cast<int>(time/ period))%2;
	time = std::fmod(time, period);
	
	
	if (count == 1) {
		time = 1 - time;
	}
	float theta = 2 * M_PI *time / period;
	
	
	float x = radius * cos(theta);
	float y = radius * sin(theta);
	
	
	float dthetdt = 2 * M_PI/ period;
	
	
	if (count == 1) {
	dthetdt *= -1;
	}
	
	
	VectorXd pos(2);
	VectorXd vel(2);
	
	
	pos << x, y;
	
	
	float dxdt = -1*radius*sin(theta)*dthetdt;
	float dydt = -1*radius*cos(theta)*dthetdt;
	
	
	vel << dxdt, dydt;
	
	
	std::tuple<VectorXd, VectorXd> pos_vel = std::make_tuple(pos, vel);
	
	
	return pos_vel;
}
	
std::tuple<VectorXd, VectorXd> drawSpiralAlternate(float freq, float time, float r1, float r2, float num_coils) {
	
	
	float period = 1/freq;
	float count = (static_cast<int>(time/ period))%2;
	time = std::fmod(time, period);
	
	
	if (count == 1) {
		time = 1 - time;
	}
	
	
	float theta = 2 * M_PI *num_coils * time / period;
	
	
	float r = r1 + (time/period) * (r2 - r1);
	
	
	float x = r * cos(theta);
	float y = r * sin(theta);
	
	
	VectorXd pos(2);
	VectorXd vel(2);
	
	
	pos << x, y;
	
	
	float drdt = (r2 - r1)/period;
	
	
	float dthetdt = 2 * M_PI * num_coils/ period;
	
	
	if (count == 1) {
	dthetdt *= -1;
	}
	
	
	float dxdt = drdt*cos(theta) + r*-1*sin(theta)*dthetdt;
	float dydt = drdt*sin(theta) + r*cos(theta)*dthetdt;
	
	
	vel << dxdt, dydt;
	
	
	std::tuple<VectorXd, VectorXd> pos_vel = std::make_tuple(pos, vel);
	
	
	return pos_vel;
}
	
	