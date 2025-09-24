//
//  tiny.cpp
//  project
//
//  Created by Samuel Atha on 9/23/25.
//

#include "tiny.hpp"
#include "project.hpp"
#include <iostream>

int main() {
  // Homebrew installs JSBSim’s data here by default:
  //   /opt/homebrew/share/jsbsim
  // That dir contains aircraft/, engine/, systems/
  FlightSim sim("/opt/homebrew/share/jsbsim", "aircraft/c172/c172.xml");

  if (!sim.load())   { std::cerr << "Load failed\n"; return 1; }
  if (!sim.runIC())  { std::cerr << "RunIC failed\n"; return 1; }

  for (int i = 0; i < 500; ++i) sim.step();

  std::cout << "Altitude (m): " << sim.altitude_m() << "\n"
            << "Roll (rad):   " << sim.roll_rad()     << "\n"
            << "Pitch (rad):  " << sim.pitch_rad()    << "\n"
            << "Yaw (rad):    " << sim.yaw_rad()      << "\n";
  return 0;
}
