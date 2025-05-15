#include "utils.h"

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