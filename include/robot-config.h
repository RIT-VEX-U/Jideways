#pragma once
#include "core.h"
#include "vex.h"

extern vex::brain brain;
extern vex::controller con;

// ================ INPUTS ================
// Digital sensors
extern vex::digital_out left_wing;
extern vex::digital_out right_wing;
// Analog sensors

// ================ OUTPUTS ================
// Motors

// Pneumatics

// ================ SUBSYSTEMS ================
extern vex::motor_group intake_motors;

extern OdometryTank odom;
extern TankDrive drive_sys;
extern vex::motor_group left_motors;
extern vex::motor_group right_motors;

extern const double intake_volts;

void intake(double volts);
void intake();
void outtake(double volts);
void outtake();

// ================ UTILS ================

void robot_init();