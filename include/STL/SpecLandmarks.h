#ifndef FIRMCP_STL_SPEC_LANDMARKS_H
#define FIRMCP_STL_SPEC_LANDMARKS_H

#include <armadillo>
#include <string>
#include <vector>
#include <unordered_map>

namespace firmcp_stl
{

// Lightweight parser for stl_bow_v2 .spec files.
// Extracts:
//   - param table  (rx, ry, rtheta, gx*/gy*, ox*/oy*, d_target, d_safe, T, ...)
//   - target list  (gx_i, gy_i)  -> "landmarks"
//   - obstacle list (ox_i, oy_i) -> point obstacles for QuadtreeCollisionChecker
class SpecLandmarks
{
public:
    SpecLandmarks() = default;
    explicit SpecLandmarks(const std::string& specFile) { load(specFile); }

    bool load(const std::string& specFile);

    const std::unordered_map<std::string, double>& params() const { return params_; }
    double param(const std::string& key, double fallback = 0.0) const
    {
        auto it = params_.find(key);
        return (it == params_.end()) ? fallback : it->second;
    }

    // [id, x, y] arma::colvec landmark format compatible with HeadingBeaconObservationModel
    const std::vector<arma::colvec>& landmarks() const { return landmarks_; }

    // Plain (x, y) obstacle centres for the collision lib's quadtree backend
    const std::vector<std::array<double, 2>>& obstacles() const { return obstacles_; }

    // Robot start pose [x, y, theta]
    arma::vec3 startPose() const { return arma::vec3{param("rx"), param("ry"), param("rtheta")}; }

    // Sequential goal poses (one entry per (gxN, gyN) pair, in numeric order of N)
    const std::vector<std::array<double, 2>>& orderedGoals() const { return orderedGoals_; }

    // Raw spec text (so STLRom can re-parse it)
    const std::string& specText() const { return specText_; }

private:
    std::unordered_map<std::string, double> params_;
    std::vector<arma::colvec>               landmarks_;
    std::vector<std::array<double, 2>>      obstacles_;
    std::vector<std::array<double, 2>>      orderedGoals_;
    std::string                             specText_;
};

} // namespace firmcp_stl

#endif // FIRMCP_STL_SPEC_LANDMARKS_H
