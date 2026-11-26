#pragma once
#include "pros/adi.hpp"
#include "rw/api.h"

// Your own devices (besides the ones RW uses, which are in rw/devices.h), defined in src/rw-config.cpp
extern pros::adi::Pneumatics claw_piston;
extern pros::MotorGroup lift;
extern pros::MotorGroup toggle;
extern pros::Distance Auto_clamp;

// Your autonomous routines (src/rw-autons.cpp). Add each one to the selector in src/rw-user.cpp
void exampleAuton();

// Called from src/main.cpp (src/rw-user.cpp)
void runPreAutonomous();
void runAutonomous();
void runDriver();
