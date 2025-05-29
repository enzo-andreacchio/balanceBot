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

const double m = 0.056; // tennis ball
// const double m = 0.170; // pool ball
const double g = 9.81;
const double timeStep = 0.3;
float kp = 50.0;
float kv = 50;
float k_theta = 0.1;
float foresight;
const float Fz_thr = 0.4;
const float plate_thickness = 0.012;
const float saturation_angle = M_PI / 12;
const int POS_BUFFER_SIZE = 100;
const int VEL_BUFFER_SIZE = 100;
const Vector3d SENSOR_POS_IN_LINK = Vector3d(0,0,0.047);
const Vector3d offset = Vector3d(0.4, 0.0, 0.55);

const int TILT_PERIOD = 100;//this is the period of time to pass before the robot tilts

//These are properties of the tools
const float PLATE_MASS = 1.169;
const Vector3d TOOL_COM = Vector3d(0,0,0.00746);
VectorXd init_force_moment = VectorXd::Zero(6);
//These are all the constant properties for the tools

bool sim = false;
int controller = -1;


#include "redis_keys.h"
#include "utils.h"

// States 
enum State {
	IDLE = 0,
	BALANCE
};

void initializeKeys(bool sim,
	std::string& JOINT_ANGLES_KEY,
	std::string& JOINT_VELOCITIES_KEY,
	std::string& JOINT_TORQUES_COMMANDED_KEY,
	std::string& MASS_MATRIX_KEY) {
	if (sim) {
		std::cout << "SIMULATION TRUE" << std::endl;
		JOINT_ANGLES_KEY = "sai::sim::PANDA::sensors::q";
		JOINT_VELOCITIES_KEY = "sai::sim::PANDA::sensors::dq";
		JOINT_TORQUES_COMMANDED_KEY = "sai::sim::PANDA::actuators::fgc";
		MASS_MATRIX_KEY.clear();  // Not used in simulation
	} else {
		std::cout << "SIMULATION FALSE" << std::endl;
		JOINT_TORQUES_COMMANDED_KEY = "sai::commands::FrankaRobot::control_torques";
		JOINT_VELOCITIES_KEY = "sai::sensors::FrankaRobot::joint_velocities";
		JOINT_ANGLES_KEY = "sai::sensors::FrankaRobot::joint_positions";
		MASS_MATRIX_KEY = "sai::sensors::FrankaRobot::model::mass_matrix";
	}
}

void setGlobalArgs(int argc, char* argv[]) {
	if (argc != 3) {
		throw std::runtime_error("Expected usage: ./controller sim|real {1-4}\n");
    }

    // Parse sim mode
    std::string mode_arg = argv[1];
    if (mode_arg == "sim") {
        sim = true;
    } else if (mode_arg == "real") {
        sim = false;
    } else {
        throw std::runtime_error("expected sim or real as first argument");
    }

    std::string arg = argv[2];
    try {
        size_t pos;
        int controller_number = std::stoi(arg, &pos);
        if (pos < arg.size()) {
            throw std::runtime_error("Trailing values after number");
        }
        if (controller_number < 0 || controller_number > 101) {
            throw std::runtime_error("Number out of range");
        }
        controller = controller_number;
    } catch (const std::invalid_argument& ex) {
        throw std::runtime_error("Invalid Number");
    } catch (const std::out_of_range& ex) {
        throw std::runtime_error("Number out of range");
    }
}



int main(int argc, char** argv) {

	setGlobalArgs(argc, argv);

	initializeKeys(sim, JOINT_ANGLES_KEY, JOINT_VELOCITIES_KEY, JOINT_TORQUES_COMMANDED_KEY, MASS_MATRIX_KEY);
	// Location of URDF files specifying world and robot information
	static const string robot_file = string(CS225A_URDF_FOLDER) + "/panda/panda_arm_balance_bot.urdf";

	// initial state 
	int state = IDLE;
	
	// start redis client
	auto redis_client = SaiCommon::RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	//----------------------------------------INIT---------------------------------------------------------

	// load robots, read current state and update the model
	auto robot = std::make_shared<SaiModel::SaiModel>(robot_file, false);
	robot->setQ(redis_client.getEigen(JOINT_ANGLES_KEY));
	robot->setDq(redis_client.getEigen(JOINT_VELOCITIES_KEY));
	MatrixXd Mass = robot->M();
	if(!sim) {
		Mass= redis_client.getEigen(MASS_MATRIX_KEY);
		// bie addition
		Mass(4,4) += 0.3;
		Mass(5,5) += 0.3;
		Mass(6,6) += 0.3;
	}
	robot->updateModel(Mass);

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);  // panda torques
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// arm task
	const string control_link = "link7";
	const Vector3d control_point = Vector3d(0, 0, 0.047+plate_thickness);
	Affine3d compliant_frame = Affine3d::Identity();
	compliant_frame.translation() = control_point;
	auto pose_task = std::make_shared<SaiPrimitives::MotionForceTask>(robot, control_link, compliant_frame);
	pose_task->setPosControlGains(100, 30, 0);
	pose_task->setOriControlGains(100, 15, 0);
	//we need to disable internal otg, because tilting will be unstable if we have waypoints in between orientations
	pose_task->disableInternalOtg();
	//let us set the velocity saturation for the pose task
	pose_task->enableVelocitySaturation(0.2, 90* M_PI/180);

	// joint task
	auto joint_task = std::make_shared<SaiPrimitives::JointTask>(robot);
	joint_task->setGains(100, 15, 0);

	// create a loop timer
	runloop = true;
	double control_freq = 1000;
	SaiCommon::LoopTimer timer(control_freq, 1e6);

	Vector3d force = Vector3d::Zero();
	Vector3d moment = Vector3d::Zero();

	//This is our counter that is useful for keeping track of the iteration of the simloop
	int count = 0;
	//This is the counter that starts as soon as the ball is sensed
	int balanceCount = 0;
	//counter -----------------------

	CartesianBuffer positionBuffer = CartesianBuffer(POS_BUFFER_SIZE);
	CartesianBuffer velocityBuffer = CartesianBuffer(VEL_BUFFER_SIZE);

	//----------------------------------------GAINS---------------------------------------------------------

	if (controller == 1) {
		kp = 80.0;
		kv = 10.0;
		foresight = 0.01;
	} else if (controller == 2) {
		kp = 30.0;
		kv = 30.0;
	} else if (controller == 3) {
		kp = 50.0;
		kv = 50.0;
	} else if (controller == 4) {
		kp = 50.0;
		kv = 50.0;
	}

	//----------------------------------------INIT---------------------------------------------------------

	//-----------------------------------------SIM_LOOP----------------------------------------------------

	while (runloop) {

		//Timer
		timer.waitForNextLoop();
		const double time = timer.elapsedSimTime();
		//Timer

		if (sim) {
			// read force and moment from redis
			force = redis_client.getEigen(FORCE_SENSOR_KEY);
			moment = redis_client.getEigen(MOMENT_SENSOR_KEY);
		} else {
			//this is true because the sensor is just identity offset from link 7, so it will have the same rotation matrix
			Matrix3d R_world_sensor = robot->rotationInWorld("link7", Matrix3d::Identity());
			VectorXd sensed_force_moment_local_frame = redis_client.getEigen(ACTUAL_FORCE_TORQUE_SENSOR_KEY);

			//Let us tear off any added values that the plate adds to force sensor readings
			tearOffToolForcesAndMoments(R_world_sensor, sensed_force_moment_local_frame, TOOL_COM, PLATE_MASS);

			if (count == 0) {
				init_force_moment = sensed_force_moment_local_frame;
			}

			sensed_force_moment_local_frame -= init_force_moment;

			force = R_world_sensor * sensed_force_moment_local_frame.head(3);
			moment = R_world_sensor * sensed_force_moment_local_frame.tail(3);

			//THis is just a caveat change, the moments we require are flipped
			moment *= -1;
		}

		// update robot ------------------------------------
		robot->setQ(redis_client.getEigen(JOINT_ANGLES_KEY));
		robot->setDq(redis_client.getEigen(JOINT_VELOCITIES_KEY));
		Mass = robot->M();
		if(!sim) {
			Mass = redis_client.getEigen(MASS_MATRIX_KEY);
			// bie addition
			Mass(4,4) += 0.3;
			Mass(5,5) += 0.3;
			Mass(6,6) += 0.3;

		}
		robot->updateModel(Mass);
		// update robot --------------------------------------


		//Calculating the ball's positions -------------------------------------------
		float Fz = force(2);

		Matrix3d ee_ori = robot->rotation(control_link);
		Vector3d zP = ee_ori.col(2); // Z-axis of the frame of the plate
		
		Vector3d sensed_ball_position = momentsToPositions(zP, moment, m, g);
		
		positionBuffer.addToBuffer(time, sensed_ball_position);
		velocityBuffer.addToBuffer(time, positionBuffer.getDelta());

		redis_client.setEigen(INFERRED_BALL_POSITION, positionBuffer.getMovingAverage());
		redis_client.setEigen(INFERRED_BALL_VELOCITY, velocityBuffer.getMovingAverage());

		// if (count % 1000 == 0) {
		// 	velocityBuffer.visualize();
		// }
		
		//------------------------------------------------------------------------------

		if (state == IDLE) {
			cout << "Currently in state: IDLE" << std::endl;
			// update task model
			N_prec.setIdentity();
			pose_task->updateTaskModel(N_prec);
			pose_task->setGoalPosition(offset);

			Matrix3d rotation;
			rotation = AngleAxisd(0*M_PI/180, Vector3d(1,0,0)).toRotationMatrix();

			pose_task->setGoalOrientation(rotation);

			command_torques = pose_task->computeTorques();

			if (Fz > Fz_thr && controller > 0 && time > 1) {
				cout << "Idle to Balance" << endl;
				pose_task->reInitializeTask();
				joint_task->reInitializeTask();
				pose_task->setGoalPosition(offset);
				state = BALANCE;
			}

		} else if (state == BALANCE) {

			Vector3d n;
			n = zP;

			Vector3d x_desired = offset;
			redis_client.setEigen(BALL_GOAL_POSITION, x_desired-offset);
			pose_task->setGoalPosition(offset);

			MatrixXd Kp = kp * MatrixXd::Identity(3, 3);
			MatrixXd Kv = kv * MatrixXd::Identity(3, 3);

			VectorXd inputForces(3);
			Vector3d ball_position = positionBuffer.getMovingAverage();
			Vector3d ball_velocity = velocityBuffer.getMovingAverage();


			float vnorm = ball_velocity.norm();

			if (vnorm < 0.1 || balanceCount < 500) {
				inputForces = -m*Kp*(ball_position);
			} else {
				Vector3d target_position = ball_position;

				if (vnorm > 0.1 && balanceCount > 500) {
					Vector3d ball_velocity_normalized = ball_velocity / vnorm;
					target_position = ball_position + ball_velocity_normalized*foresight;
				}

				inputForces = -m*Kp*(target_position) - m*Kv*(ball_velocity);
			}

			


			


			
			

			

			
			// F rotated in the frame of the plate
			Vector3d F_rotated = ee_ori.transpose() * inputForces;

			float Fdx = F_rotated(0);
			float Fdy = F_rotated(1);

			Vector2d F_plate = Vector2d(Fdx, Fdy);
			redis_client.setEigen(DESIRED_BALL_FORCE, F_plate);
			float thr_zero_force = 1e-7;
			Vector3d n_Pi;

			n_Pi = Vector3d(-Fdy, Fdx, 0)/sqrt(Fdx*Fdx + Fdy*Fdy);

			Vector2d n_Pi_plate = Vector2d(n_Pi(0), n_Pi(1));
			redis_client.setEigen(N_PI, n_Pi_plate);


			float force_magnitude = sqrt(Fdx * Fdx + Fdy * Fdy);
			float theta_npi;
			theta_npi = force_magnitude*k_theta;
			
			if (theta_npi > saturation_angle){
				theta_npi = saturation_angle;
				cout << "Angle saturation" << endl;
			} else if (theta_npi < -saturation_angle) {
				theta_npi = -saturation_angle;
				cout << "Angle saturation" << endl;
			} 

			Eigen::AngleAxisd rotation(theta_npi, ee_ori * n_Pi);
			Eigen::Matrix3d R = rotation.toRotationMatrix();

			if (count%TILT_PERIOD == 0) {
				pose_task->setGoalOrientation(R);
			}
			
			// update task model
			N_prec.setIdentity();
			pose_task->updateTaskModel(N_prec);
			
			joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());
			command_torques = pose_task->computeTorques() + joint_task->computeTorques();

			balanceCount++;
		}

		// execute redis write callback
		redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		count += 1;
	}

	timer.stop();
	cout << "\nSimulation loop timer stats:\n";
	timer.printInfoPostRun();
	redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, 0 * command_torques);  // back to floating

	return 0;
}


