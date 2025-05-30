#ifndef UTILS_H_
#define UTILS_H_

#include <Eigen/Dense>  // for Matrix3d and Vector3d
#include <cmath>        // for std::abs, etc.
#include <stdio.h>
#include <math.h>
#include <deque>

using namespace std;
using namespace Eigen;

class CartesianBuffer {
public:

    CartesianBuffer(int bufferSize);
    int getSize() const;
    Vector3d getMovingAverage() const;
    void addToBuffer(float time, Vector3d val);
    Vector3d getCurrentValue() const;
    Vector3d getDelta() const;
    Vector3d getInst() const;
    Vector3d getStandardDeviation() const;
    void visualize() const;

private:

    std::deque<Vector3d> _buffer;
    std::deque<float> _timesBuffer;
    int _bufferSize;
    Vector3d _bufferSum;
};

//This function makes sure that force and moment readings are near 0 for all configurations of the end effector
void tearOffToolForcesAndMoments(Matrix3d & R_world_sensor, VectorXd & sensed_force_moment_local_frame, const Vector3d & tool_com, const float tool_mass);

Matrix3d crossProductOperator(Vector3d& v);

Vector3d momentsToPositions(Vector3d& zP, Vector3d& M, double m, double g);

float sgn(float x);

std::tuple<VectorXd, VectorXd> drawSpiralAlternate(float freq, float time, float r1, float r2, float num_coils);

std::tuple<VectorXd, VectorXd> drawCircleAlternate(float freq, float time, float radius);

std::tuple<VectorXd, VectorXd> drawCircle(float freq, float time, float radius);

std::tuple<VectorXd, VectorXd> drawTripleSpiral(float freq, float time, float radius);

#endif // UTILS_H_
