/**
 * @file redis_keys.h
 * @brief Contains all redis keys for simulation and control.
 * 
 */

std::string JOINT_ANGLES_KEY = "sai::sim::panda::sensors::q";
std::string JOINT_VELOCITIES_KEY = "sai::sim::panda::sensors::dq";
std::string JOINT_TORQUES_COMMANDED_KEY = "sai::sim::panda::actuators::fgc";
const std::string CONTROLLER_RUNNING_KEY = "sai::sim::panda::controller";
const std::string BALL_POS_KEY = "sai::sim::ball::position";
const std::string BALL_VEL_KEY = "sai::sim::ball::velocity";
const std::string FORCE_SENSOR_KEY = "sai::sim::panda::sensors::F";
const std::string MOMENT_SENSOR_KEY = "sai::sim::panda::sensors::M";
const std::string ACTUAL_FORCE_TORQUE_SENSOR_KEY = "sai2::ATIGamma_Sensor::Romeo::force_torque";
const std::string INFERRED_BALL_POSITION = "sai::real::inferred_ball_position";
const std::string DESIRED_BALL_FORCE = "sai::real::desired_ball_force";
const std::string BALL_GOAL_POSITION = "sai::real::ball_goal_position";
const std::string N_PI = "sai::real::n_pi";

std::string MASS_MATRIX_KEY;
