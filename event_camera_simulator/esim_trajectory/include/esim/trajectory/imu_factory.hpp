#pragma once

#include <gflags/gflags.h>
#include <ze/vi_simulation/trajectory_simulator.hpp>
#include <ze/vi_simulation/imu_simulator.hpp>

namespace event_camera_simulator {

ze::ImuSimulator::Ptr loadImuSimulatorFromGflags(const ze::TrajectorySimulator::Ptr &trajectory);

} // namespace event_camera_simulator
