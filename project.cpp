//
//  project.cpp
//  project
//
//  Created by Samuel Atha on 9/23/25.
//

#include "project.hpp"

// SGPath lives in SimGear, installed under the JSBSim include prefix
#include <JSBSim/simgear/misc/sg_path.hxx>

// Installed headers live under the JSBSim prefix when built via CMake install
#include <JSBSim/FGFDMExec.h>
#include <JSBSim/initialization/FGInitialCondition.h>
#include <JSBSim/models/FGPropagate.h>

using namespace JSBSim;

FlightSim::FlightSim(const std::string& root_dir, const std::string& model_xml)
: fdm_(nullptr), root_dir_(root_dir), model_xml_(model_xml) {}

FlightSim::~FlightSim() = default;

bool FlightSim::load() {
  fdm_ = std::make_unique<FGFDMExec>();
  fdm_->SetRootDir(SGPath(root_dir_));
  if (!fdm_->LoadModel(model_xml_)) return false;
  return true;
}

bool FlightSim::runIC() {
  if (!fdm_) return false;
  return fdm_->RunIC();
}

bool FlightSim::step() {
  if (!fdm_) return false;
  return fdm_->Run();
}

double FlightSim::altitude_m() const {
  return fdm_ ? fdm_->GetPropagate()->GetAltitudeASLmeters() : 0.0;
}
double FlightSim::roll_rad() const {
  return fdm_ ? fdm_->GetPropagate()->GetEuler(1) : 0.0;   // phi
}
double FlightSim::pitch_rad() const {
  return fdm_ ? fdm_->GetPropagate()->GetEuler(2) : 0.0;   // theta
}
double FlightSim::yaw_rad() const {
  return fdm_ ? fdm_->GetPropagate()->GetEuler(3) : 0.0;   // psi
}
