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
#include <filesystem>
#include <cstdlib>
#include <unordered_map>
#include <string>

using namespace JSBSim;

namespace {
struct EmaFilter {
  double alpha{0.2};
  double y{0.0};
  bool initialized{false};
  double update(double x) {
    if (!initialized) { y = x; initialized = true; }
    else { y = alpha * x + (1.0 - alpha) * y; }
    return y;
  }
};

// Read an environment variable as double, with a default fallback.
double getEnvDouble(const char* name, double defval) {
  if (const char* s = std::getenv(name)) {
    char* end = nullptr;
    double v = std::strtod(s, &end);
    if (end != s) return v;
  }
  return defval;
}

// Keep a per-signal EMA filter map so multiple signals don't interfere.
std::unordered_map<std::string, EmaFilter> kFilters;

// Apply telemetry error adjustment: scale, bias, then low‑pass filter.
// Param names are read from env to allow tuning at runtime without code changes.
// Keys (for env var suffixes): "ALT", "ROLL", "PITCH", "YAW".
double adjustTelemetry(const char* key,
                       double raw_value,
                       double default_bias,
                       double default_scale,
                       double default_alpha) {
  // Build env var names: e.g., TELEMETRY_ALT_BIAS, TELEMETRY_ALT_SCALE, TELEMETRY_ALT_ALPHA
  std::string prefix = std::string("TELEMETRY_") + key + "_";
  double bias  = getEnvDouble((prefix + "BIAS").c_str(),  default_bias);
  double scale = getEnvDouble((prefix + "SCALE").c_str(), default_scale);
  double alpha = getEnvDouble((prefix + "ALPHA").c_str(), default_alpha);

  double corrected = raw_value * scale + bias;

  // Per-key EMA smoothing
  auto& f = kFilters[std::string(key)];
  f.alpha = alpha;
  return f.update(corrected);
}
} // namespace

FlightSim::FlightSim(const std::string& root_dir, const std::string& model_xml)
: fdm_(nullptr), root_dir_(root_dir), model_xml_(model_xml) {}

FlightSim::~FlightSim() = default;

bool FlightSim::load() {
  fdm_ = std::make_unique<FGFDMExec>();
  fdm_->SetRootDir(SGPath(root_dir_));

  namespace fs = std::filesystem;

  // Resolve model path to something JSBSim can actually open relative to root_dir_
  fs::path root(root_dir_);
  fs::path rel(model_xml_);

  // If "root/rel" doesn't exist, try common JSBSim layout: aircraft/<id>/<id>.xml
  if (!fs::exists(root / rel)) {
    if (rel.extension() != ".xml" && rel.string().find('/') == std::string::npos) {
      fs::path alt = fs::path("aircraft") / rel / (rel.string() + ".xml");
      if (fs::exists(root / alt)) {
        rel = alt;
      }
    }
  }

  // If an absolute path was provided under root, make it relative to root (JSBSim expects relative)
  if (rel.is_absolute()) {
    std::error_code ec;
    fs::path maybe_rel = fs::relative(rel, root, ec);
    if (!ec && !maybe_rel.empty()) rel = maybe_rel;
  }

  return fdm_->LoadModel(rel.string());
}

bool FlightSim::runIC() {
  if (!fdm_) return false;
  return fdm_->RunIC();
}

bool FlightSim::step() {
  if (!fdm_) return false;
  return fdm_->Run();
}

// Telemetry error adjustment:
// Each getter applies: corrected = EMA( raw * SCALE + BIAS )
// Tunable via environment variables at runtime (no recompile):
//   TELEMETRY_ALT_BIAS   (meters), TELEMETRY_ALT_SCALE,  TELEMETRY_ALT_ALPHA
//   TELEMETRY_ROLL_BIAS  (radians), TELEMETRY_ROLL_SCALE, TELEMETRY_ROLL_ALPHA
//   TELEMETRY_PITCH_BIAS (radians), TELEMETRY_PITCH_SCALE, TELEMETRY_PITCH_ALPHA
//   TELEMETRY_YAW_BIAS   (radians), TELEMETRY_YAW_SCALE,  TELEMETRY_YAW_ALPHA
// Alpha ∈ [0,1]: higher = more responsive, lower = smoother.
double FlightSim::altitude_m() const {
  if (!fdm_) return 0.0;
  double raw = fdm_->GetPropagate()->GetAltitudeASLmeters();
  // Default: no bias, no scale change, light smoothing. Override via:
  //   TELEMETRY_ALT_BIAS (meters), TELEMETRY_ALT_SCALE, TELEMETRY_ALT_ALPHA (0..1)
  return adjustTelemetry("ALT", raw, /*bias*/0.0, /*scale*/1.0, /*alpha*/0.2);
}
double FlightSim::roll_rad() const {
  if (!fdm_) return 0.0;
  double raw = fdm_->GetPropagate()->GetEuler(1); // phi
  // Default bias 0 rad, scale 1, smoothing alpha 0.25. Override via TELEMETRY_ROLL_*
  return adjustTelemetry("ROLL", raw, /*bias*/0.0, /*scale*/1.0, /*alpha*/0.25);
}
double FlightSim::pitch_rad() const {
  if (!fdm_) return 0.0;
  double raw = fdm_->GetPropagate()->GetEuler(2); // theta
  // Override via TELEMETRY_PITCH_* (bias in radians)
  return adjustTelemetry("PITCH", raw, /*bias*/0.0, /*scale*/1.0, /*alpha*/0.25);
}
double FlightSim::yaw_rad() const {
  if (!fdm_) return 0.0;
  double raw = fdm_->GetPropagate()->GetEuler(3); // psi
  // Override via TELEMETRY_YAW_* (bias in radians)
  return adjustTelemetry("YAW", raw, /*bias*/0.0, /*scale*/1.0, /*alpha*/0.3);
}
