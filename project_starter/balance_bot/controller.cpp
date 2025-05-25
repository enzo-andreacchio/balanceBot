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

const double m = 0.170;
const double g = 9.81;
const double timeStep = 0.3;
float kp = 50.0;
float kv = 50;
float k_theta = 0.1;
const float Fz_thr = 0.4;
const float plate_thickness = 0.012;
const float saturation_angle = M_PI / 12;

bool sim = false;


#include "redis_keys.h"
#include "utils.h"

// States 
enum State {
	IDLE = 0,
	BALANCE
};



int main(int argc, char** argv) {

	if (sim){
		cout << "SIMULATION TRUE" << endl;
		JOINT_ANGLES_KEY = "sai::sim::PANDA::sensors::q";
		JOINT_VELOCITIES_KEY = "sai::sim::PANDA::sensors::dq";
		JOINT_TORQUES_COMMANDED_KEY = "sai::sim::PANDA::actuators::fgc";
	} else{
		cout << "SIMULATION FALSE" << endl;
		JOINT_TORQUES_COMMANDED_KEY = "sai::commands::FrankaRobot::control_torques";
		JOINT_VELOCITIES_KEY = "sai::sensors::FrankaRobot::joint_velocities";
		JOINT_ANGLES_KEY = "sai::sensors::FrankaRobot::joint_positions";
		MASS_MATRIX_KEY = "sai::sensors::FrankaRobot::model::mass_matrix";
	}
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
        else if (controller_number < 0 || controller_number > 101) {
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




	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots, read current state and update the model
	auto robot = std::make_shared<SaiModel::SaiModel>(robot_file, false);
	robot->setQ(redis_client.getEigen(JOINT_ANGLES_KEY));
	robot->setDq(redis_client.getEigen(JOINT_VELOCITIES_KEY));
	MatrixXd Mass = robot->M();
	if(!sim) {
		Mass= redis_client.getEigen(MASS_MATRIX_KEY);
		// bie addition
		Mass(4,4) += 0.2;
		Mass(5,5) += 0.2;
		Mass(6,6) += 0.2;
	}
	robot->updateModel(Mass);

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);  // panda + gripper torques 
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// arm task
	const string control_link = "link7";
	const Vector3d control_point = Vector3d(0, 0, 0.047+plate_thickness);
	Affine3d compliant_frame = Affine3d::Identity();
	compliant_frame.translation() = control_point;
	auto pose_task = std::make_shared<SaiPrimitives::MotionForceTask>(robot, control_link, compliant_frame);
	pose_task->setPosControlGains(100, 15, 0);
	pose_task->setOriControlGains(100, 15, 0);
	//pose_task->disableInternalOtg();

	Vector3d ee_pos;
	Matrix3d ee_ori;

	// joint task
	auto joint_task = std::make_shared<SaiPrimitives::JointTask>(robot);
	joint_task->setGains(100, 15, 0);

	VectorXd ee_pos_desired(3); 

	Vector3d ball_position_prev = Vector3d::Zero();
	Vector3d ball_velocity = Vector3d::Zero();

	// create a loop timer
	runloop = true;
	double control_freq = 1000;
	SaiCommon::LoopTimer timer(control_freq, 1e6);

	double previousTime;
	previousTime = 0.0;

	Affine3d sensor_tranformation_in_link;
	Vector3d sensor_position_in_link;

	Vector3d force = Vector3d::Zero();
	Vector3d moment = Vector3d::Zero();

	VectorXd init_force_moment = VectorXd::Zero(6);
	bool first_loop = true;

	int count = 0;

	while (runloop) {
		count += 1;

		if (sim) {
			// read force and moment from redis
			force = redis_client.getEigen(FORCE_SENSOR_KEY);
			moment = redis_client.getEigen(MOMENT_SENSOR_KEY);
		} else {
			Matrix3d R_link_sensor = Matrix3d::Identity();
			sensor_position_in_link = Vector3d(0,0,0.047);
			sensor_tranformation_in_link.translation() = sensor_position_in_link;
			sensor_tranformation_in_link.linear() = R_link_sensor;

			VectorXd sensed_force_moment_local_frame = VectorXd::Zero(6);
			VectorXd sensed_force_moment_world_frame = VectorXd::Zero(6);

			VectorXd force_bias = VectorXd::Zero(6);
			double tool_mass = 0;
			Vector3d tool_com = Vector3d::Zero();


			// Properties
			force_bias << 0, 0, 0, 0, 0, 0;
			tool_mass = 1.169;
			tool_com = Vector3d(0,0,0.00746);

			// read force and moment from redis
			sensed_force_moment_local_frame = redis_client.getEigen(ACTUAL_FORCE_TORQUE_SENSOR_KEY);

			

			// Add bias and ee weight to sensed forces
			sensed_force_moment_local_frame -= force_bias;


			Matrix3d R_world_sensor;
			Matrix3d R_world_link;
			R_world_link = robot->rotationInWorld("link7", Matrix3d::Identity());
			R_world_sensor = R_world_link * R_link_sensor;
			Vector3d p_tool_local_frame = tool_mass * R_world_sensor.transpose() * Vector3d(0,0,-9.81);

			sensed_force_moment_local_frame.head(3) += p_tool_local_frame;
			

			sensed_force_moment_local_frame.tail(3) += tool_com.cross(p_tool_local_frame);

			if (first_loop) {
				init_force_moment = sensed_force_moment_local_frame;
				first_loop = false;
			}

			sensed_force_moment_local_frame -= init_force_moment;


			force = R_world_sensor * sensed_force_moment_local_frame.head(3);
			moment = R_world_sensor * sensed_force_moment_local_frame.tail(3);

			// cout << moment.transpose() << endl;

		}



		timer.waitForNextLoop();
		const double time = timer.elapsedSimTime();

		// update robot 
		robot->setQ(redis_client.getEigen(JOINT_ANGLES_KEY));
		robot->setDq(redis_client.getEigen(JOINT_VELOCITIES_KEY));
		Mass = robot->M();
		Vector3d M;
		if(!sim) {
			Mass = redis_client.getEigen(MASS_MATRIX_KEY);
			// bie addition
			Mass(4,4) += 0.2;
			Mass(5,5) += 0.2;
			Mass(6,6) += 0.2;

			M = -moment; // Moment vector
		}
		else {
			M = -moment; // Moment vector
		}
		robot->updateModel(Mass);

		Vector3d ball_position = redis_client.getEigen(BALL_POS_KEY);

		

		float Fz = force(2);

		// cout << force.transpose() << endl;
		// cout << local_force.transpose() << endl;

		Vector3d ee_pos = robot->position(control_link, control_point);
		Matrix3d ee_ori = robot->rotation(control_link);
		Vector3d ee_acceleration = robot->acceleration6d(control_link, control_point).head(3);
		Vector3d plate_velocity = robot->velocity6d(control_link, control_point).head(3);

		// cout << ee_ori.transpose() << endl;

		Vector3d zP = ee_ori.col(2); // Z-axis of the frame of the plate

		// cout << zP.transpose() << endl;

		// Compute the position vector from the moment vector
		
		Vector3d x = momentsToPositions(zP, M, m, g);
		Vector3d offset(0.4, 0.0, 0.55);

		Vector3d ball_position_predicted = x + offset;

		redis_client.setEigen(INFERRED_BALL_POSITION, x);

		// cout << x.transpose() << endl;



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

	


		// cout << "Fz " << Fz << endl;
		// if (Fz > Fz_thr && time > 1) {
		// 	cout << "in balance state" << endl;
		// }


		if (state == IDLE) {
			// cout << "IDLE STATE" << endl;
			// update task model
			N_prec.setIdentity();
			pose_task->updateTaskModel(N_prec);

			pose_task->setGoalPosition(offset);




			// pose_task->setGoalOrientation(Matrix3d::Identity());

			Matrix3d rotation;
			rotation = AngleAxisd(0*M_PI/180, Vector3d(1,0,0)).toRotationMatrix();


			// if (time > 4) {
			// 	rotation = AngleAxisd(5*M_PI/180, Vector3d(1,0,0)).toRotationMatrix();
			// }
			// else if (time > 3) {
			// 	rotation = AngleAxisd(10*M_PI/180, Vector3d(1,0,0)).toRotationMatrix();
			// }
			// else if (time > 2) {
			// 	rotation = AngleAxisd(15*M_PI/180, Vector3d(1,0,0)).toRotationMatrix();
			// }
			// else if (time > 1) {
			// 	rotation = AngleAxisd(20*M_PI/180, Vector3d(1,0,0)).toRotationMatrix();
			// }



			pose_task->setGoalOrientation(rotation);

			command_torques = pose_task->computeTorques();

			if (Fz > Fz_thr && controller_number > 0 && time > 1) {
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
				kp = 12.0;
				kv = 5.0;
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
			
			redis_client.setEigen(BALL_GOAL_POSITION, x_desired-offset);

			Vector3d plate_position_desired;

			if (controller_number == 1){
				plate_position_desired = offset;
			} else if (controller_number == 2) {
				plate_position_desired = offset;
			} else if (controller_number == 3) {
				plate_position_desired = offset+Vector3d(0, 0, 0.05*cos(time));
			} else if (controller_number == 4) {
				plate_position_desired = offset+Vector3d(0, 0.05*sin(time), 0.05*cos(time));
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

			// cout << ball_velocity.transpose() << endl;
			inputForces = -m*Kp*(ball_position_predicted - x_desired)- F_g_prll;
			// inputForces = m*(-Kp*(ball_position_predicted - x_desired) - Kv*(ball_velocity));
			// inputForces = m*(-Kp*(ball_position_predicted - x_desired) - Kv*(ball_velocity)) - F_g_prll;
			// inputForces = m*(-Kp*(ball_position_predicted - x_desired) - Kv*(ball_velocity-dx_desired)) - F_g_prll;
			// inputForces = m*(-Kp*(ball_position_predicted - x_desired) - Kv*(ball_velocity) - ee_acceleration) - F_g_prll;

			

			// F rotated in the frame of the plate
			Vector3d F_rotated = ee_ori.transpose() * inputForces;

			

			

			

			float Fdx = F_rotated(0);
			float Fdy = F_rotated(1);

			Vector2d F_plate = Vector2d(Fdx, Fdy);

			redis_client.setEigen(DESIRED_BALL_FORCE, F_plate);

			cout << redis_client.getEigen(DESIRED_BALL_FORCE).transpose() << endl;

			float thr_zero_force = 1e-7;

			Vector3d n_Pi;

			n_Pi = Vector3d(-Fdy, Fdx, 0)/sqrt(Fdx*Fdx + Fdy*Fdy);


			Vector2d n_Pi_plate = Vector2d(n_Pi(0), n_Pi(1));
			redis_client.setEigen(N_PI, n_Pi_plate);


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

			// cout << theta_npi*180/M_PI << endl;
			


			Eigen::AngleAxisd rotation(theta_npi, ee_ori * n_Pi);

			// cout << (ee_ori * n_Pi).transpose() << endl;
			Eigen::Matrix3d R = rotation.toRotationMatrix();

			if (count%100 == 0) {
				pose_task->setGoalOrientation(R);
			}
			

			// update task model
			N_prec.setIdentity();
			pose_task->updateTaskModel(N_prec);
			
			joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());

			command_torques = pose_task->computeTorques() + joint_task->computeTorques();

			//  if (Fz < Fz_thr) {
			//  	cout << "Balance to Idle" << endl;

			//  	pose_task->reInitializeTask();
			// 	joint_task->reInitializeTask();

			// 	pose_task->setGoalPosition(offset);


			//  	state = IDLE;
			// }
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
