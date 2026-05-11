#include "STL/STLRobustness.h"

#include "stl_driver.h"

#include <cmath>
#include <limits>

namespace firmcp_stl
{

STLRobustness::STLRobustness(const std::string&                        specText,
                             double                                    dt,
                             const std::vector<std::array<double, 2>>& obstacles)
    : specText_(specText)
    , dt_(dt)
    , obstacles_(obstacles)
    , driverTemplate_(std::make_unique<STLRom::STLDriver>())
{
    driverTemplate_->parse_string(specText_);
}

STLRobustness::~STLRobustness() = default;

double STLRobustness::minObstacleDistance(double x, double y) const
{
    if (obstacles_.empty())
        return 1e3; // effectively "no obstacles in range"
    double best = std::numeric_limits<double>::infinity();
    for (const auto& o : obstacles_)
    {
        double dx = x - o[0];
        double dy = y - o[1];
        double d  = std::sqrt(dx * dx + dy * dy);
        if (d < best) best = d;
    }
    return best;
}

double STLRobustness::score(const std::vector<arma::vec2>& xy) const
{
    return scoreMonitor(xy, "phi");
}

double STLRobustness::scoreMonitor(const std::vector<arma::vec2>& xy, const std::string& monitor) const
{
    if (xy.empty())
        return -std::numeric_limits<double>::infinity();

    STLRom::STLDriver driver;
    driver.parse_string(specText_);

    std::vector<double> sample(4, 0.0);
    double t = 0.0;
    for (const auto& p : xy)
    {
        sample[0] = t;
        sample[1] = p(0);
        sample[2] = p(1);
        sample[3] = minObstacleDistance(p(0), p(1));
        driver.add_sample(sample);
        t += dt_;
    }
    driver.set_param("T", t);

    try
    {
        auto m = driver.get_monitor(monitor);
        return m.eval_rob();
    }
    catch (...)
    {
        return -std::numeric_limits<double>::infinity();
    }
}

} // namespace firmcp_stl
