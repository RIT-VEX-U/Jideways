#include "competition/opcontrol.h"
#include "robot-config.h"
#include "vex.h"
#include <atomic>

std::atomic<bool> tank{false};
// std::atomic<TankDrive::BrakeType> brake_type = TankDrive::BrakeType::None;

const vex::controller::button &intake_button = con.ButtonR1;
const vex::controller::button &outtake_button = con.ButtonR2;

const vex::controller::button &drive_mode_button = con.ButtonX;

const vex::controller::button &left_wing_button = con.ButtonDown;
const vex::controller::button &right_wing_button = con.ButtonB;
/**
 * Main entrypoint for the driver control period
 */
const double intake_volts = 10.0;
void opcontrol() {
  // ================ INIT ================
  // Wings
  left_wing_button.pressed([]() { left_wing.set(!left_wing); });
  right_wing_button.pressed([]() { right_wing.set(!right_wing); });

  // Intake
  intake_button.pressed([]() { intake_motors.spin(vex::fwd, intake_volts, vex::volt); });
  outtake_button.pressed([]() { intake_motors.spin(vex::reverse, intake_volts, vex::volt); });
  // Misc
  drive_mode_button.pressed([]() { tank = !tank; });
  con.ButtonLeft.pressed([]() { screen::prev_page(); });
  con.ButtonRight.pressed([]() { screen::next_page(); });

  // ================ PERIODIC ================
  while (true) {
    // intake motors
    if (!intake_button.pressing() && !outtake_button.pressing()) {
      intake_motors.stop(vex::brakeType::hold);
    }

    // drive
    if (tank) {
      double left = (double)con.Axis3.position() / 100.0;
      double right = (double)con.Axis2.position() / 100.0;
      drive_sys.drive_tank(left, right);
    } else {
      double forward = (double)con.Axis3.position() / 100.0;
      double turny = (double)con.Axis1.position() / 100.0;

      drive_sys.drive_arcade(forward, .75 * turny);
    }
    vexDelay(20);
  }
}