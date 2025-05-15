/**
 * @file controller.cpp
 * @brief Controller file
 * 
 */

#include <SaiModel.h>
#include "SaiPrimitives.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>

using namespace std;
using namespace Eigen;
using namespace SaiPrimitives;

#include <signal.h>
bool runloop = false;
void sighandler(int){runloop = false;}

const double m = 0.17;
const double g = 9.81;
const double timeStep = 0.3;
const float kp = 30.0;
const float kv = 20;
const float Fz_thr = 1.0;
const float plate_thickness = 0.01;
const float saturation_angle = M_PI / 6;


#include "redis_keys.h"

// States 
enum State {
	IDLE = 0,
	BALANCE
};

// Cross-product matrix operator
Matrix3d crossProductOperator(const Vector3d& v) {
	Matrix3d cross;
	cross <<     0, -v.z(),  v.y(),
			v.z(),     0, -v.x(),
			-v.y(),  v.x(),     0;
	return cross;
}

// Computes position vector from moment vector
Vector3d momentsToPositions(Vector3d& zP, const Vector3d& M) {
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

int main() {
	// Location of URDF files specifying world and robot information
	static const string robot_file = string(CS225A_URDF_FOLDER) + "/panda/panda_arm_balance_bot.urdf";

	// initial state 
	int state = IDLE;
	string controller_status = "1";
	
	// start redis client
	auto redis_client = SaiCommon::RedisClient();
	redis_client.connect();

	// set the redis key for force to 0
	redis_client.setEigen(FORCE_SENSOR_KEY, Vector3d::Zero());

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots, read current state and update the model
	auto robot = std::make_shared<SaiModel::SaiModel>(robot_file, false);
	robot->setQ(redis_client.getEigen(JOINT_ANGLES_KEY));
	robot->setDq(redis_client.getEigen(JOINT_VELOCITIES_KEY));
	robot->updateModel();

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);  // panda + gripper torques 
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// arm task
	const string control_link = "link7";
	const Vector3d control_point = Vector3d(0, 0, 0.17);
	Affine3d compliant_frame = Affine3d::Identity();
	compliant_frame.translation() = control_point;
	auto pose_task = std::make_shared<SaiPrimitives::MotionForceTask>(robot, control_link, compliant_frame);
	pose_task->setPosControlGains(400, 40, 0);
	pose_task->setOriControlGains(400, 40, 0);

	Vector3d ee_pos;
	Matrix3d ee_ori;

	// joint task
	auto joint_task = std::make_shared<SaiPrimitives::JointTask>(robot);
	joint_task->setGains(400, 40, 0);

	VectorXd q_desired(dof);
	q_desired << -73.05, -57.93, 64.52, -144.73, -128.29, 77.12, 40.70;
	q_desired *= M_PI / 180.0;
	joint_task->setGoalPosition(q_desired);

	VectorXd ee_pos_desired(3); 

	Vector3d ball_position_prev = Vector3d::Zero();
	Vector3d ball_velocity = Vector3d::Zero();

	// create a loop timer
	runloop = true;
	double control_freq = 1000;
	SaiCommon::LoopTimer timer(control_freq, 1e6);

	double previousTime;
	previousTime = 0.0;

	while (runloop) {
		timer.waitForNextLoop();
		const double time = timer.elapsedSimTime();

		// update robot 
		robot->setQ(redis_client.getEigen(JOINT_ANGLES_KEY));
		robot->setDq(redis_client.getEigen(JOINT_VELOCITIES_KEY));
		robot->updateModel();

		Vector3d ball_position = redis_client.getEigen(BALL_POS_KEY);
		Vector3d ball_velocity_real = redis_client.getEigen(BALL_VEL_KEY);

		Vector3d force = redis_client.getEigen(FORCE_SENSOR_KEY);
		float Fz = force(2);

		Vector3d ee_pos = robot->position(control_link, control_point);
		Matrix3d ee_ori = robot->rotation(control_link);

		Vector3d moment = redis_client.getEigen(MOMENT_SENSOR_KEY);
		Vector3d zP = ee_ori.col(2); // Z-axis of the frame of the plate

	

		// Compute the position vector from the moment vector
		Vector3d M = -moment; // Moment vector
		Vector3d x = momentsToPositions(zP, M);
		Vector3d offset(0.4, 0.0, 0.65-0.01514);

		Vector3d ball_position_predicted = x + offset;


		// Compute the ball velocity using a time step
		double elapsedTime = time - previousTime;
		if (elapsedTime > timeStep) {
			ball_velocity = (ball_position_predicted - ball_position_prev) / elapsedTime;
			previousTime = time;
			ball_position_prev = ball_position_predicted;
		}

		VectorXd inputForces(3);

	
		if (state == IDLE) {
			// update task model
			N_prec.setIdentity();
			pose_task->updateTaskModel(N_prec);

			pose_task->setGoalPosition(offset);
			pose_task->setGoalOrientation(Matrix3d::Identity());

			command_torques = pose_task->computeTorques();


			if (Fz > Fz_thr) {
				cout << "Idle to Balance" << endl;

				pose_task->reInitializeTask();
				joint_task->reInitializeTask();

				pose_task->setGoalPosition(offset);

				state = BALANCE;
			}

		} else if (state == BALANCE) {

			Vector3d n;
			n = zP;
			
			// // Linear test
			Vector3d x_desired;
			x_desired = offset + Vector3d(0.0, 0.09, 0.0);


			Vector3d s;
            s << n(0)/n(2), n(1)/n(2), -1* (n(1)*n(1) + n(0)*n(0))/(n(2)*n(2));
            s = s/s.norm();
            MatrixXd proj = s * s.transpose();
            VectorXd F_g(3);
            F_g << 0, 0, -1*m*g;
            Vector3d F_g_prll;
			F_g_prll = proj * F_g;

			MatrixXd Kp = kp * MatrixXd::Identity(3, 3);
			MatrixXd Kv = kv * MatrixXd::Identity(3, 3);
			
			// inputForces = m*(-Kp*(ball_position_predicted - x_desired) - Kv*(ball_velocity- v_desired) );
			inputForces = m*(-Kp*(ball_position_predicted - x_desired) - Kv*(ball_velocity)) - F_g_prll;

			// F rotated in the frame of the plate
			Vector3d F_rotated = ee_ori.transpose() * inputForces;	

			float Fdx = F_rotated(0);
			float Fdy = F_rotated(1);

			float thr_zero_force = 1e-7;

			Vector3d n_Pi;

			n_Pi = Vector3d(-Fdy, Fdx, 0)/sqrt(Fdx*Fdx + Fdy*Fdy);;

			float force_magnitude = sqrt(Fdx * Fdx + Fdy * Fdy);

			float theta_npi;
			theta_npi = force_magnitude*0.05;
			
			if (theta_npi > saturation_angle){
				theta_npi = saturation_angle;
			} else if (theta_npi < -saturation_angle) {
				theta_npi = -saturation_angle;
			} 

			Eigen::AngleAxisd rotation(theta_npi, n_Pi);
			Eigen::Matrix3d R = rotation.toRotationMatrix();

			pose_task->setGoalOrientation(R);

			// update task model
			N_prec.setIdentity();
			pose_task->updateTaskModel(N_prec);
			
			joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());

			command_torques = pose_task->computeTorques() + joint_task->computeTorques();
		}

		// execute redis write callback
		redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, command_torques);

	}

	timer.stop();
	cout << "\nSimulation loop timer stats:\n";
	timer.printInfoPostRun();
	redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, 0 * command_torques);  // back to floating

	return 0;
}
