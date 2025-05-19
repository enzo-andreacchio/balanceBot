/**
 * @file redis_keys.h
 * @brief Contains all redis keys for simulation and control.
 * 
 */

const std::string JOINT_ANGLES_KEY = "sim::panda::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sim::panda::sensors::dq";
const std::string JOINT_TORQUES_COMMANDED_KEY = "sim::panda::actuators::fgc";
const std::string CONTROLLER_RUNNING_KEY = "sim::panda::controller";
const std::string FORCE_SENSOR_KEY = "sim::panda::sensors::F";
const std::string MOMENT_SENSOR_KEY = "sim::panda::sensors::M";
const std::string BALL_POS_KEY = "sim::ball::position";
const std::string BALL_VEL_KEY = "sim::ball::velocity";
