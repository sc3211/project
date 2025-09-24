# project
JBSIM
# Flight Sim Project

Flight Sim (JSBSim + Xcode, macOS)

Minimal C++ wrapper around JSBSim (a flight dynamics engine) with an Xcode project for macOS (Apple Silicon).
The goal: compile, run, and print real flight values (altitude, airspeed, heading, etc.) with the fewest moving parts.

⸻

Contents
	•	What this is
	•	Prerequisites
	•	Install & build JSBSim
	•	Xcode project setup
	•	Project layout
	•	Quick start (run it)
	•	Reading flight values
	•	Applying controls
	•	Troubleshooting
	•	Make it portable (no Homebrew needed)
	•	FAQ
	•	License

⸻

What this is
	•	A tiny wrapper class (FlightSim) that hides JSBSim details:
	•	load() → creates FGFDMExec, sets data root, loads a model
	•	runIC() → runs initial conditions
	•	step() → advances the simulation by one time step
	•	getters (altitude_m(), airspeed_mps(), etc.) → read live sim state
	•	A minimal main.cpp (example below) that prints values while the sim runs.

⸻

Prerequisites
	•	macOS on Apple Silicon (M1/M2/M3)
	•	Xcode (Command Line Tools installed)
	•	Homebrew (package manager)
	•	CMake (to build JSBSim)

Install tools:
xcode-select --install
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
echo 'eval "$(/opt/homebrew/bin/brew shellenv)"' >> ~/.zprofile
eval "$(/opt/homebrew/bin/brew shellenv)"
brew install cmake
Install & build JSBSim

Build and install JSBSim as a static library to the Homebrew prefix:
git clone https://github.com/JSBSim-Team/jsbsim.git
cd jsbsim
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DCMAKE_OSX_ARCHITECTURES=arm64 \
         -DCMAKE_INSTALL_PREFIX=/opt/homebrew
cmake --build . --config Release
sudo make install

What that gives you:
	•	Headers at: /opt/homebrew/include/JSBSim/...
	•	Static lib at: /opt/homebrew/lib/libJSBSim.a
	•	Data root at: /opt/homebrew/share/jsbsim (contains aircraft/, engine/, systems/, etc.)
Xcode project setup

Open the Xcode project and set:

Build Settings (Target):
	•	Header Search Paths : Debug/ Not release
/opt/homebrew/include
/opt/homebrew/include/JSBSim

Library search path:
/opt/homebrew/lib

Other linker flags:
-lJSBSim

Architecture:
arm64 

^^^^^^^^^^^^^^^^^^^ XCode build settings

Project layout
project/
  project.xcodeproj
  project.hpp          # FlightSim wrapper (header)
  project.cpp          # FlightSim wrapper (implementation)
  main.cpp             # (you add this) demo entry point

  ^^^^^^^^^^The wrapper avoids including JSBSim headers in the .hpp by forward-declaring FGFDMExec. Implementation (project.cpp) includes the real headers.

  Create main.cpp in the project and paste:
  #include "project.hpp"
#include <iostream>
#include <iomanip>

int main() {
  // Homebrew default data root:
  FlightSim sim("/opt/homebrew/share/jsbsim", "aircraft/c172/c172.xml");
  // You can also just pass the ID: FlightSim sim("/opt/homebrew/share/jsbsim", "c172");

  if (!sim.load() || !sim.runIC()) {
    std::cerr << "Load/init failed\n";
    return 1;
  }

  for (int i = 0; i < 500; ++i) {
    // Optional: basic control inputs so you see motion:
    // sim.set_throttle_norm(0.7);
    // sim.set_elevator_norm(0.1);

    sim.step();

    if (i % 50 == 0) {
      std::cout << std::fixed << std::setprecision(3)
                << "Alt(m): " << sim.altitude_m()
                << " TAS(m/s): " << sim.airspeed_mps()
                << " GS(m/s): "  << sim.groundspeed_mps()
                << " Lat: "      << sim.latitude_deg()
                << " Lon: "      << sim.longitude_deg()
                << " Hdg(deg): " << sim.heading_deg()
                << "\n";
    }
  }
  return 0;
}

Build & Run (Debug config is fine).
You should see altitude/airspeed/heading values print in the console.

eading flight values

The wrapper exposes common state after each step():
	•	altitude_m() → altitude above sea level (meters)
	•	roll_rad(), pitch_rad(), yaw_rad() → Euler angles (radians)
	•	airspeed_mps() → true airspeed (m/s)
	•	groundspeed_mps() → ground speed (m/s)
	•	latitude_deg(), longitude_deg() → geodetic position (degrees)
	•	heading_deg() → heading/ψ (degrees)

(Depending on your JSBSim version, these map to: GetAltitudeASLmeters(), GetEuler(i), GetTas(), GetVground(), GetLatitudeDeg(), GetLongitudeDeg(), GetPsiDeg() respectively.)

Applying controls

JSBSim uses normalized control properties (–1..+1 for surfaces, 0..1 for throttle).
In your loop (before step()):
// via wrapper (recommended) — implement these methods if not present yet
// sim.set_throttle_norm(0.7);
// sim.set_elevator_norm(0.1);

// or direct (if you have access to the underlying FGFDMExec):
// fdm_->SetPropertyValue("fcs/throttle-cmd-norm", 0.7);
// fdm_->SetPropertyValue("fcs/elevator-cmd-norm", 0.1);
