#ifndef SMOOTH_APPROX_H
#define SMOOTH_APPROX_H

#include <vector>
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace STLRom {

// Selects which smooth max/min approximation to use.
// As tau -> infinity, both approximations converge to exact max/min.
enum class SmoothType {
    EXACT,    // exact max/min (no smoothing, same as compute_robustness)
    SOFTMAX,  // softmax weighted average: sum(xi * exp(tau*xi)) / sum(exp(tau*xi))
    LSE       // log-sum-exp: (1/tau) * log(sum(exp(tau*xi)))
};

// Numerically stable smooth max via the log-sum-exp trick (subtract max before summing).
inline double smooth_max(const std::vector<double>& x, double tau, SmoothType type) {
    if (x.empty()) return 0.0;
    if (type == SmoothType::EXACT || tau <= 0.0)
        return *std::max_element(x.begin(), x.end());

    double xmax = *std::max_element(x.begin(), x.end());

    if (type == SmoothType::LSE) {
        // max_LSE(x) = xmax + (1/tau) * log( sum_i exp(tau*(xi - xmax)) )
        double sum = 0.0;
        for (double xi : x) sum += std::exp(tau * (xi - xmax));
        return xmax + std::log(sum) / tau;
    } else { // SOFTMAX
        // max_soft(x) = sum_i xi*exp(tau*(xi-xmax)) / sum_i exp(tau*(xi-xmax))
        double sum_exp = 0.0, sum_x_exp = 0.0;
        for (double xi : x) {
            double e = std::exp(tau * (xi - xmax));
            sum_exp   += e;
            sum_x_exp += xi * e;
        }
        return sum_x_exp / sum_exp;
    }
}

// smooth_min(x) = -smooth_max(-x)
inline double smooth_min(const std::vector<double>& x, double tau, SmoothType type) {
    if (x.empty()) return 0.0;
    if (type == SmoothType::EXACT || tau <= 0.0)
        return *std::min_element(x.begin(), x.end());
    std::vector<double> neg(x.size());
    for (size_t i = 0; i < x.size(); ++i) neg[i] = -x[i];
    return -smooth_max(neg, tau, type);
}

// Two-value convenience wrappers
inline double smooth_max2(double a, double b, double tau, SmoothType type) {
    return smooth_max({a, b}, tau, type);
}

inline double smooth_min2(double a, double b, double tau, SmoothType type) {
    return smooth_min({a, b}, tau, type);
}

} // namespace STLRom
#endif // SMOOTH_APPROX_H
