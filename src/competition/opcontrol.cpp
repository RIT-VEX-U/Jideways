#include "competition/opcontrol.h"
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
/**
 * Main entrypoint for the driver control period
 */

struct inp {
  double time;
  double fwd;
  double turn;
  bool operator==(const inp &o) { return time == o.time && fwd == o.fwd && turn == o.turn; }
};

std::vector<inp> map1 = {
  {0.00, 1.00, 0.00},  {0.04, 1.00, 0.00},  {0.82, 1.00, 0.00},  {0.84, 1.00, -0.87}, {0.86, 1.00, -1.00},
  {1.30, 1.00, -1.00}, {1.32, 1.00, -0.56}, {1.34, 1.00, 0.00},  {1.37, 1.00, 0.00},  {1.39, 1.00, 0.00},
  {1.41, 1.00, 0.00},  {1.44, 1.00, 0.00},  {1.46, 1.00, 0.00},  {1.48, 1.00, 0.00},  {1.50, 1.00, 0.00},
  {1.51, 1.00, 0.00},  {1.53, 1.00, 0.00},  {1.56, 1.00, 0.00},  {1.58, 1.00, 0.00},  {1.60, 1.00, 0.00},
  {1.62, 1.00, -0.85}, {1.64, 1.00, -1.00}, {1.67, 1.00, -1.00}, {1.69, 1.00, -1.00}, {1.71, 1.00, -1.00},
  {1.73, 1.00, -1.00}, {1.75, 1.00, -1.00}, {1.77, 1.00, -1.00}, {1.79, 1.00, -1.00}, {1.82, 0.34, 0.00},
  {1.84, 0.34, 0.00},  {1.86, 0.00, 0.00},  {1.88, 0.00, 0.00},  {1.90, 0.00, 0.00},  {1.92, 0.00, 0.00},
};

const double fold_out_time = 0.25;
void opcontrol() {
  std::vector<inp> map = map1;
  if (map.size() > 1) {
    vex::timer tim;
    inp &prev = map[0];
    printf("step");
    for (inp &next : map) {
      if (next == prev) {
        continue;
      }
      while (1) {
        double t = tim.value();
        double param = (t - prev.time) / (next.time - prev.time);
        double fwd = lerp(prev.fwd, next.fwd, param);
        double turn = lerp(prev.turn, next.turn, param);
        printf("%.2f < %.2f < %.2f    %.2f, %.2f\n", prev.time, t, next.time, fwd, turn);

        drive_sys.drive_arcade(fwd, turn);
        if (t > next.time) {
          break;
        }

        vexDelay(10);
      }
    }
  }
  vex::timer drop_timer;
  // outtake();

  con.ButtonA.pressed([]() {
    do_drive = false;
    CommandController cc{
      drive_sys.DriveForwardCmd(24),
    };
    // left_motors.stop(vex::brakeType::hold);
    // right_motors.stop(vex::brakeType::hold);
    cc.run();
    do_drive = true;
  });

  // ================ INIT ================
  // Wings
  left_wing_button.pressed([]() { left_wing.set(!left_wing); });
  right_wing_button.pressed([]() { right_wing.set(!right_wing); });

  // Intake
  intake_button.pressed([]() { intake(intake_volts); });
  outtake_button.pressed([]() { outtake(intake_volts); });

  // Misc
  drive_mode_button.pressed([]() { tank = !tank; });
  con.ButtonLeft.pressed([]() { screen::prev_page(); });
  con.ButtonRight.pressed([]() { screen::next_page(); });

  // ================ PERIODIC ================
  while (true) {
    // intake motors
    if (!intake_button.pressing() && !outtake_button.pressing() && drop_timer.value() > fold_out_time) {
      intake_motors.stop(vex::brakeType::hold);
    }

    // drive
    if (do_drive) {
      if (tank) {
        double left = (double)con.Axis3.position() / 100.0;
        double right = (double)con.Axis2.position() / 100.0;
        drive_sys.drive_tank(left, right);
      } else {
        double forward = (double)con.Axis3.position() / 100.0;
        double turny = (double)con.Axis1.position() / 100.0;
        printf("{%.2f, %.2f, %.2f}\n", drop_timer.value(), forward, turny);

        drive_sys.drive_arcade(forward, .75 * turny);
      }
    }
    vexDelay(20);
  }
}