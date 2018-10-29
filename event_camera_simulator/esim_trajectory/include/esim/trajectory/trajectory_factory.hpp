#pragma once

#include <gflags/gflags.h>
#include <ze/vi_simulation/trajectory_simulator.hpp>

namespace event_camera_simulator {

std::tuple<ze::TrajectorySimulator::Ptr, std::vector<ze::TrajectorySimulator::Ptr>> loadTrajectorySimulatorFromGflags();

} // namespace event_camera_simulator
