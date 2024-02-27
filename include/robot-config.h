#pragma once
#include "core.h"
#include "vex.h"

extern vex::brain brain;
extern vex::controller con;

// ================ INPUTS ================
// Digital sensors

// Analog sensors

// ================ OUTPUTS ================
// Motors

// Pneumatics

// ================ SUBSYSTEMS ================
extern vex::motor_group intake_motors;

extern OdometryTank odom;
extern TankDrive drive_sys;

// ================ UTILS ================

void robot_init();