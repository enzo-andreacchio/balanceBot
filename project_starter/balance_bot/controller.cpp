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
float kp = 50.0;
float kv = 50;
float k_theta = 0.05;
const float Fz_thr = 0.1;
const float plate_thickness = 0.01;
const float saturation_angle = M_PI / 4;

bool sim = true;


#include "redis_keys.h"
#include "utils.h"

// States 
enum State {
	IDLE = 0,
	BALANCE
};



int main(int argc, char** argv) {
	// Location of URDF files specifying world and robot information
	static const string robot_file = string(CS225A_URDF_FOLDER) + "/panda/panda_arm_balance_bot.urdf";

	// check for command line arguments
    if (argc != 2) {
        cout << "Incorrect number of command line arguments" << endl;
        cout << "Expected usage: ./controller.cpp {NUMBER}" << endl;
        return 1;
    }

	 // convert char to int, check for correct controller number input
    string arg = argv[1];
    int controller_number;
    try {
        size_t pos;
        controller_number = stoi(arg, &pos);
        if (pos < arg.size()) {
            cerr << "Trailing characters after number: " << arg << '\n';
            return 1;
        }
        else if (controller_number < 0 || controller_number > 4) {
            cout << "Incorrect controller number" << endl;
            return 1;
        }
    } catch (invalid_argument const &ex) {
        cerr << "Invalid number: " << arg << '\n';
        return 1;
    } catch (out_of_range const &ex) {
        cerr << "Number out of range: " << arg << '\n';
        return 1;
    }

	// initial state 
	int state = IDLE;
	
	// start redis client
	auto redis_client = SaiCommon::RedisClient();
	redis_client.connect();



	if (sim) {
		force_sensor_key = FORCE_SENSOR_KEY;
		moment_sensor_key = MOMENT_SENSOR_KEY;
	}
	else {
		force_torque_sensor_key = ACTUAL_FORCE_TORQUE_SENSOR_KEY;
	}

	if (sim){
		// set the redis key for force to 0
		redis_client.setEigen(force_sensor_key, Vector3d::Zero());
	}
	

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
	const Vector3d control_point = Vector3d(0, 0, 0.17+plate_thickness);
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

	VectorXd ee_pos_desired(3); 

	Vector3d ball_position_prev = Vector3d::Zero();
	Vector3d ball_velocity = Vector3d::Zero();

	// create a loop timer
	runloop = true;
	double control_freq = 1000;
	SaiCommon::LoopTimer timer(control_freq, 1e6);

	double previousTime;
	previousTime = 0.0;

	Vector3d force = Vector3d::Zero();
	Vector3d moment = Vector3d::Zero();

	while (runloop) {

		if (sim) {
			// read force and moment from redis
			force = redis_client.getEigen(FORCE_SENSOR_KEY);
			moment = redis_client.getEigen(MOMENT_SENSOR_KEY);
		} else {
			// read force and moment from redis
			VectorXd force_moment = redis_client.getEigen(ACTUAL_FORCE_TORQUE_SENSOR_KEY);
			force << force_moment(0), force_moment(1), force_moment(2);
			moment << force_moment(3), force_moment(4), force_moment(5);
		}

		timer.waitForNextLoop();
		const double time = timer.elapsedSimTime();

		// update robot 
		robot->setQ(redis_client.getEigen(JOINT_ANGLES_KEY));
		robot->setDq(redis_client.getEigen(JOINT_VELOCITIES_KEY));
		robot->updateModel();

		Vector3d ball_position = redis_client.getEigen(BALL_POS_KEY);

		float Fz = force(2);

		Vector3d ee_pos = robot->position(control_link, control_point);
		Matrix3d ee_ori = robot->rotation(control_link);
		Vector3d ee_acceleration = robot->acceleration6d(control_link, control_point).head(3);
		Vector3d plate_velocity = robot->velocity6d(control_link, control_point).head(3);

		

		Vector3d zP = ee_ori.col(2); // Z-axis of the frame of the plate

	

		// Compute the position vector from the moment vector
		Vector3d M = -moment; // Moment vector
		Vector3d x = momentsToPositions(zP, M, m, g);
		Vector3d offset(0.4, 0.0, 0.65-0.01514);

		Vector3d ball_position_predicted = x + offset;


		// Compute the ball velocity using a time step
		double elapsedTime = time - previousTime;
		if (elapsedTime > timeStep) {
			ball_velocity = (ball_position_predicted - ball_position_prev) / elapsedTime;
			previousTime = time;
			ball_position_prev = ball_position_predicted;
		}

		Vector3d ball_velocity_in_plate = ball_velocity - plate_velocity;
		Vector3d ball_position_in_plate = ball_position - ee_pos;
		// dot product between ball_velocity_in_plate & ball_position_in_plate
		float direction = ball_velocity_in_plate.dot(ball_position_in_plate);



		VectorXd inputForces(3);

	
		if (state == IDLE) {
			// update task model
			N_prec.setIdentity();
			pose_task->updateTaskModel(N_prec);

			pose_task->setGoalPosition(offset);
			pose_task->setGoalOrientation(Matrix3d::Identity());

			command_torques = pose_task->computeTorques();


			if (Fz > Fz_thr && controller_number > 0) {
				cout << "Idle to Balance" << endl;

				pose_task->reInitializeTask();
				joint_task->reInitializeTask();

				pose_task->setGoalPosition(offset);

				state = BALANCE;
			}

		} else if (state == BALANCE) {

			Vector3d n;
			n = zP;

			
			
			// center in one sense and then in the other
			// double T = 16; // seconds
			// double t_mod = fmod(time, 2.0*T);
			// double angle;
			// if (t_mod < T) {
			// 	angle = 2.0 * M_PI * t_mod / T;
			// } else {
			// 	angle = 2.0 * M_PI * (2.0*T-t_mod) / T;
			// }

			Vector3d x_desired;
			Vector3d dx_desired;


			if (controller_number == 1) {
				kp = 50.0;
				kv = 50.0;
				x_desired = offset;
			} else if (controller_number == 2) {
				kp = 30.0;
				kv = 30.0;
				x_desired = offset + Vector3d(0.1*cos(time), 0.1*sin(time),0.0);
				dx_desired = Vector3d(-0.1*sin(time), 0.1*cos(time), 0.0);
			} else if (controller_number == 3) {
				kp = 50.0;
				kv = 50.0;
				x_desired = offset;
			} else if (controller_number == 4) {
				kp = 50.0;
				kv = 50.0;
				x_desired = offset;
		
			}
			


			Vector3d plate_position_desired;

			if (controller_number == 1){
				plate_position_desired = Vector3d(0.4, 0.0, 0.65);
			} else if (controller_number == 2) {
				plate_position_desired = Vector3d(0.4, 0.0, 0.65);
			} else if (controller_number == 3) {
				plate_position_desired = Vector3d(0.4, 0.0, 0.65+0.05*cos(time));
			} else if (controller_number == 4) {
				plate_position_desired = Vector3d(0.4, 0.0+0.05*sin(time), 0.65+0.05*cos(time));
			}

			// plate_position_desired = Vector3d(0.4, 0.0 + 0.1*sin(2*time), 0.65+0.1*cos(2*time));
			
			pose_task->setGoalPosition(plate_position_desired);


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

			
			
			inputForces = m*(-Kp*(ball_position_predicted - x_desired) - Kv*(ball_velocity)) - F_g_prll;
			// inputForces = m*(-Kp*(ball_position_predicted - x_desired) - Kv*(ball_velocity-dx_desired)) - F_g_prll;
			// inputForces = m*(-Kp*(ball_position_predicted - x_desired) - Kv*(ball_velocity) - ee_acceleration) - F_g_prll;

			// F rotated in the frame of the plate
			Vector3d F_rotated = ee_ori.transpose() * inputForces;	

			float Fdx = F_rotated(0);
			float Fdy = F_rotated(1);

			float thr_zero_force = 1e-7;

			Vector3d n_Pi;

			n_Pi = Vector3d(-Fdy, Fdx, 0)/sqrt(Fdx*Fdx + Fdy*Fdy);;

			float force_magnitude = sqrt(Fdx * Fdx + Fdy * Fdy);

			float theta_npi;

			theta_npi = force_magnitude*k_theta;

			// if (direction > 0.7) {
			// 	theta_npi = force_magnitude*k_theta*ball_velocity_in_plate.norm()*3/0.1;
			// }

			
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

			if (Fz < Fz_thr) {
				cout << "Balance to Idle" << endl;

				pose_task->reInitializeTask();
				joint_task->reInitializeTask();

				pose_task->setGoalPosition(offset);


				state = IDLE;
			}
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
