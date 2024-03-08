#include "competition/opcontrol.h"
#include "competition/autonomous.h"
#include "robot-config.h"
#include "vex.h"
#include <atomic>

std::atomic<bool> tank{false};
std::atomic<bool> do_drive{true};
std::atomic<TankDrive::BrakeType> brake_type{TankDrive::BrakeType::None};

const vex::controller::button &intake_button = con.ButtonR1;
const vex::controller::button &outtake_button = con.ButtonR2;

const vex::controller::button &drive_mode_button = con.ButtonX;

const vex::controller::button &left_wing_button = con.ButtonDown;
const vex::controller::button &right_wing_button = con.ButtonB;

const vex::controller::button &both_wing_button = con.ButtonL1;
const vex::controller::button &brake_mode_button = con.ButtonL2;

const vex::controller::button &climb_wing_button = con.ButtonUp;
/**
 * Main entrypoint for the driver control period
 */

const double fold_out_time = 0.25;
void opcontrol() {

  // odom.set_position({.x = 40, .y = 12, .rot = 90});
  while (imu.isCalibrating()) {
    vexDelay(1);
  }
  con.ButtonA.pressed([]() {
    do_drive = false;
    CommandController cc{
      drive_sys.DriveForwardCmd(24, vex::reverse, 0.5),
    };
    cc.run();
    do_drive = true;
  });
  vex::timer drop_timer;
  brake_mode_button.pressed([]() {
    if (brake_type == TankDrive::BrakeType::None) {
      brake_type = TankDrive::BrakeType::Smart;
    } else {
      brake_type = TankDrive::BrakeType::None;
    }
  });

  // ================ INIT ================
  // Wings
  left_wing_button.pressed([]() { left_wing.set(!left_wing); });
  right_wing_button.pressed([]() { right_wing.set(!right_wing); });
  both_wing_button.pressed([]() {
    right_wing.set(!right_wing);
    left_wing.set(!left_wing);
  });

  // Intake
  intake_button.pressed([]() { intake(intake_volts); });
  outtake_button.pressed([]() { outtake(intake_volts); });

  // Misc
  con.ButtonLeft.pressed([]() { screen::prev_page(); });
  con.ButtonRight.pressed([]() { screen::next_page(); });

  climb_wing_button.pressed([]() { climb_wing.set(!climb_wing); });
  drive_mode_button.pressed([]() { tank = !tank; });

  // ================ PERIODIC ================
  while (true) {
    // intake motors
    if (!intake_button.pressing() && !outtake_button.pressing() && drop_timer.value() > fold_out_time) {
      intake_motors.stop(vex::brakeType::hold);
    }

    // drive
    if (do_drive) {
      if (tank) {
        double left = (double)con.Axis4.position() / 100.0;
        double right = (double)con.Axis3.position() / 100.0;
        drive_sys.drive_tank(left, right, 1, brake_type);
      } else {
        double forward = (double)con.Axis3.position() / 100.0;
        double turny = (double)con.Axis1.position() / 100.0;

        drive_sys.drive_arcade(forward, .75 * turny, 1, brake_type);
      }
    }
    vexDelay(20);
  }
}