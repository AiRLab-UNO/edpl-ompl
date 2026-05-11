#ifndef FIRMCP_STL_ROBUSTNESS_H
#define FIRMCP_STL_ROBUSTNESS_H

#include <armadillo>
#include <array>
#include <memory>
#include <string>
#include <vector>

namespace STLRom { class STLDriver; }

namespace firmcp_stl
{

// Thin wrapper around STLRom::STLDriver that scores trajectories
// against an STL spec. Trajectory points are (x, y); time is filled
// uniformly with the driver's dt.
//
// The driver expects samples shaped as `[time, x1, y1, ox]` matching the
// `signal x1, y1, ox` declaration in stl_bow_v2's spec convention.
//   * x1, y1 -- robot position
//   * ox     -- nearest-obstacle distance, in metres. Computed automatically
//               from the obstacles vector passed at construction. With no
//               obstacles, `ox` defaults to a large constant (1e3) so any
//               `obs_critical := ox < d_safe` predicate evaluates as safe.
class STLRobustness
{
public:
    STLRobustness(const std::string&                          specText,
                  double                                      dt        = 0.1,
                  const std::vector<std::array<double, 2>>&   obstacles = {});
    ~STLRobustness();

    // Score the trajectory against the "phi" monitor.
    double score(const std::vector<arma::vec2>& xy) const;

    // Convenience scorers for the standard sub-monitors.
    double scoreReach(const std::vector<arma::vec2>& xy) const { return scoreMonitor(xy, "phi_reach"); }
    double scoreSafe (const std::vector<arma::vec2>& xy) const { return scoreMonitor(xy, "phi_safe");  }

    // Score against a specific monitor name. Returns -inf if missing/empty.
    double scoreMonitor(const std::vector<arma::vec2>& xy, const std::string& monitor) const;

    const std::string& specText() const { return specText_; }

private:
    double minObstacleDistance(double x, double y) const;

    std::string                                specText_;
    double                                     dt_;
    std::vector<std::array<double, 2>>         obstacles_;
    std::unique_ptr<STLRom::STLDriver>         driverTemplate_;
};

} // namespace firmcp_stl

#endif // FIRMCP_STL_ROBUSTNESS_H
