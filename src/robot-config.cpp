#include "robot-config.h"

vex::brain Brain;
vex::controller con;

// ================ INPUTS ================
// Digital sensors
vex::inertial imu(vex::PORT8);

// Analog sensors

// ================ OUTPUTS ================
// Motors

// 1 in front, 4 in back
vex::motor left1(vex::PORT1, vex::gearSetting::ratio18_1, false);
vex::motor left2(vex::PORT2, vex::gearSetting::ratio18_1, false);
vex::motor left3(vex::PORT3, vex::gearSetting::ratio18_1, false);
vex::motor left4(vex::PORT4, vex::gearSetting::ratio18_1, false);

vex::motor right1(vex::PORT11, vex::gearSetting::ratio18_1, true);
vex::motor right2(vex::PORT12, vex::gearSetting::ratio18_1, true);
vex::motor right3(vex::PORT13, vex::gearSetting::ratio18_1, true);
vex::motor right4(vex::PORT14, vex::gearSetting::ratio18_1, true);

vex::motor_group left_motors = {left1, left2, left3, left4};
vex::motor_group right_motors = {right1, right2, right3, right4};

// ================ SUBSYSTEMS ================
PID::pid_config_t drive_pid = {};
PID::pid_config_t drive_correction_pid = {};

PID::pid_config_t turn_pid = {};

robot_specs_t robot_cfg = {
    .robot_radius = 8.0,
    .odom_wheel_diam = 2.0,
    .odom_gear_ratio = 1,
    .dist_between_wheels = 12.0,

    .drive_correction_cutoff = 3.0,
    .drive_feedback = new PID(drive_pid),
    .turn_feedback = new PID(turn_pid),
    .correction_pid = drive_correction_pid,
};

OdometryTank odom(left_motors, right_motors, robot_cfg, &imu);

TankDrive drive_sys(left_motors, right_motors, robot_cfg, &odom);

// ======== SUBSYSTEMS ========

// ================ UTILS ================

/**
 * Main robot initialization on startup. Runs before opcontrol and autonomous
 * are started.
 */
void robot_init() {}