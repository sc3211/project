// project.hpp — clean header
// Created by Samuel Atha on 9/23/25

#pragma once

#include <cstdint>
#include <string>
#include <memory>

// Forward declaration so this header does not require JSBSim headers.
namespace JSBSim { class FGFDMExec; }

/**
 * Minimal flight simulator wrapper interface.
 * The implementation (.cpp) includes real JSBSim headers once the library is linked.
 */
class FlightSim {
public:
  // @param root_dir   Absolute path to the JSBSim data root (contains aircraft/, engine/, systems/)
  // @param model_xml  Path (relative to root_dir) to an aircraft XML (e.g., "aircraft/c172/c172.xml")
  FlightSim(const std::string& root_dir, const std::string& model_xml);
  ~FlightSim();

  bool load();   // load model files
  bool runIC();  // initialize initial conditions
  bool step();   // advance one step

  // Common state accessors
  double altitude_m() const;  // ASL altitude
  double roll_rad()   const;
  double pitch_rad()  const;
  double yaw_rad()    const;

private:
  std::unique_ptr<JSBSim::FGFDMExec> fdm_;
  std::string root_dir_;
  std::string model_xml_;
};
