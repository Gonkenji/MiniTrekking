#pragma once
#include <cstdint>

void imu_init();
void imu_update(float dt, float &yaw);