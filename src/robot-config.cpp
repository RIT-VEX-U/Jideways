#include "robot-config.h"
#include "../core/include/subsystems/fun/video.h"
#include <map>
// Ports 10, 6 are bad

vex::brain Brain;
vex::controller con;

// ================ INPUTS ================
// Digital sensors
vex::inertial imu(vex::PORT1);

vex::digital_out left_wing(Brain.ThreeWirePort.A);
vex::digital_out right_wing(Brain.ThreeWirePort.B);

// Analog sensors

// ================ OUTPUTS ================
// Motors
vex::motor left_intake(vex::PORT2, vex::gearSetting::ratio18_1, false);
vex::motor right_intake(vex::PORT9, vex::gearSetting::ratio18_1, true);

// 1 in front, 4 in back
vex::motor left_front_bottom(vex::PORT3, vex::gearSetting::ratio6_1,
                             false); // wire sucks
vex::motor left_back_top(vex::PORT1, vex::gearSetting::ratio6_1, true);
vex::motor left_bottom_middle(vex::PORT4, vex::gearSetting::ratio6_1, false);
vex::motor left_bottom_back(vex::PORT11, vex::gearSetting::ratio6_1, true);

vex::motor right_front_bottom(vex::PORT8, vex::gearSetting::ratio6_1, true);
vex::motor right_back_top(vex::PORT21, vex::gearSetting::ratio6_1, false);
vex::motor right_bottom_middle(vex::PORT7, vex::gearSetting::ratio6_1, true);
vex::motor right_bottom_back(vex::PORT20, vex::gearSetting::ratio6_1, false);

vex::motor_group left_motors = {left_front_bottom, left_back_top, left_bottom_middle, left_bottom_back};
vex::motor_group right_motors = {right_front_bottom, right_back_top, right_bottom_middle, right_bottom_back};

vex::motor_group intake_motors{left_intake, right_intake};

// clang-format off
std::map<std::string, vex::motor &> motor_names = {
  {"left front", left_front_bottom}, 
  {"left mid", left_bottom_middle},  
  {"left back", left_bottom_back},
  {"left top", left_back_top},

  {"right front", right_front_bottom}, 
  {"right top", right_back_top},
  {"right mid", right_bottom_middle},  
  {"right back", right_bottom_back}, 

  // Intake
  {"left int", left_intake},           
  {"right int", right_intake},

};
// clang-format on

// ================ SUBSYSTEMS ================
PID::pid_config_t drive_pid = {
  .p = 0,
  .i = 0,
  .d = 0,
  .deadband = 1.0,
};

MotionController::m_profile_cfg_t drive_mc_cfg{
  .max_v = 0,
  .accel = 0,
  .pid_cfg = drive_pid,
  .ff_cfg = FeedForward::ff_config_t{},
};
MotionController drive_mc{drive_mc_cfg};

PID::pid_config_t drive_correction_pid = {
  .p = 0,
  .i = 0,
  .d = 0,
  .deadband = 1.0,
};

PID::pid_config_t turn_pid = {
  .p = 0, .i = 0, .d = 0, .deadband = 1.0,
  // .error_method =
};

robot_specs_t robot_cfg = {
  .robot_radius = 8.0,
  .odom_wheel_diam = 2.0,
  .odom_gear_ratio = 1,
  .dist_between_wheels = 12.0,

  .drive_correction_cutoff = 3.0,
  .drive_feedback = &drive_mc,
  .turn_feedback = new PID(turn_pid),
  .correction_pid = drive_correction_pid,
};

OdometryTank odom(left_motors, right_motors, robot_cfg, &imu);
TankDrive drive_sys(left_motors, right_motors, robot_cfg, &odom);

// ======== SUBSYSTEMS ========
const double intake_volts = 10.0;
void intake(double volts) { intake_motors.spin(vex::fwd, volts, vex::volt); };
void intake() { intake_motors.spin(vex::fwd, intake_volts, vex::volt); };

void outtake(double volts) { intake_motors.spin(vex::reverse, volts, vex::volt); };
void outtake() { intake_motors.spin(vex::reverse, intake_volts, vex::volt); };

// ================ UTILS ================

/**
 * Main robot initialization on startup. Runs before opcontrol and autonomous
 * are started.
 */
void robot_init() {
  set_video("cj2.mpeg");
  screen::start_screen(Brain.Screen, {new VideoPlayer(), new screen::StatsPage(motor_names)});
}