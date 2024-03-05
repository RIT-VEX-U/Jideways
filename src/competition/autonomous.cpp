#include "competition/autonomous.h"
#include "robot-config.h"

void just_auto();
void autonomous() {
  while (imu.isCalibrating()) {
    vexDelay(1);
  }
  just_auto();
}
AutoCommand *intake_cmd(double amt = 8.0) {
  return new FunctionCommand([=]() {
    intake(amt);
    return true;
  });
}
AutoCommand *outtake_cmd(double amt = 8.0) {
  return new FunctionCommand([=]() {
    outtake(amt);
    return true;
  });
}

class DebugCommand : public AutoCommand {
public:
  bool run() override {
    drive_sys.stop();
    pose_t pos = odom.get_position();
    printf("ODO X: %.2f, Y: %.2f, R:%.2f, ", pos.x, pos.y, pos.rot);
    while (true) {
      double f = con.Axis3.position() / 200.0;
      double s = con.Axis1.position() / 200.0;
      drive_sys.drive_arcade(f, s, 1, TankDrive::BrakeType::None);
      pose_t pos = odom.get_position();
      printf("ODO X: %.2f, Y: %.2f, R:%.2f\n", pos.x, pos.y, pos.rot);
      // printf("GPS X: %.2f, Y: %.2f, R: %.2f Q: %d\n",
      //     gps_sensor.xPosition(distanceUnits::in)+72,
      //     gps_sensor.yPosition(distanceUnits::in)+72,
      //     gps_sensor.heading(), gps_sensor.quality());
      //   );
      vexDelay(100);
    }
    return false;
  }
};
AutoCommand *toggle_wing() {
  return new FunctionCommand([]() {
    right_wing.set(!right_wing);
    return true;
  });
}
AutoCommand *stop_intake() {
  return new FunctionCommand([]() {
    intake(0);
    return true;
  });
}
void just_auto() {
  // clang-format off
  CommandController cc{
    odom.SetPositionCmd({.x = 40, .y = 7, .rot = 90}),     
    drive_sys.DriveToPointCmd({40, 35}, vex::fwd, 0.5),
    drive_sys.TurnToPointCmd(59.5, 51.7, vex::fwd, 0.5),
    // pick up first
    intake_cmd(),
    drive_sys.DriveToPointCmd({58.5, 51.7}, vex::fwd, 0.5), 
    drive_sys.DriveToPointCmd({38.5, 35.5}, vex::reverse, 0.5),
    stop_intake(),
    drive_sys.TurnToPointCmd(37.9, 19.5, vex::fwd, 0.5),
    drive_sys.DriveToPointCmd({37.9, 19.5}, vex::fwd, 0.5), 

    // Drop First
    drive_sys.TurnToHeadingCmd(340, 0.5),
    outtake_cmd(),
    new DelayCommand(600),
    stop_intake(),
    drive_sys.TurnToHeadingCmd(-90, 0.5),
    drive_sys.DriveToPointCmd({39.7, 34.5}, vex::reverse, 0.5),

    // pick up second
    drive_sys.TurnToPointCmd(47.2, 54.7, vex::fwd, 0.5),
    intake_cmd(12.0),
    drive_sys.DriveToPointCmd({47.63, 54.5}, vex::fwd, 0.5), 
    drive_sys.DriveToPointCmd({37.7, 34.5}, vex::reverse, 0.5),
    stop_intake(),
    drive_sys.TurnToPointCmd(37.5, 20.0, vex::fwd, 0.5),
    drive_sys.DriveToPointCmd({37.5, 20.0}, vex::fwd, 0.5),

    // Drop Second
    drive_sys.TurnToHeadingCmd(340, 0.5),
    outtake_cmd(),
    new DelayCommand(600),
    stop_intake(),

    // new DebugCommand(),

    // get close to wall
    drive_sys.TurnToHeadingCmd(240, 0.5),
    drive_sys.DriveToPointCmd({33.5,12.5}, vex::fwd,0.25),

    // turn and back up to bar
    drive_sys.TurnToPointCmd(22.0,16.0, vex::reverse,0.5),
    drive_sys.DriveToPointCmd({22,16.0},vex::reverse,0.15),
    new RepeatUntil({
        toggle_wing(),
        new DelayCommand(300),
        toggle_wing(),
        new DelayCommand(800),
    }, (new IfTimePassed(35))->Or(new TimesTestedCondition(10))),
    
    // outtake_cmd(),
    new DebugCommand(),
  };
    cc.run();
  // clang-format on
}