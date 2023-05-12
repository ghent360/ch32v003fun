#pragma once

// variables to hold latest sensor data values 
extern float ax, ay, az;
extern float gx, gy, gz;
extern float mx, my, mz;

// vector to hold quaternion
extern float q[4];

// vector to hold integral error for Mahony method
extern float eInt[3];

// How much time since the last update in seconds
extern float deltat;

void MadgwickQuaternionUpdate();
void MahonyQuaternionUpdate();
