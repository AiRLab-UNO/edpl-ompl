#include "STLRom/stdafx.h"
#include <STLRom/transducer.h>
#include <STLRom/smooth_approx.h>
#include <algorithm>
#include <vector>
#include <cmath>

namespace STLRom {

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

// Returns the interpolated value of a piecewise-linear signal at time t.
// The signal must be non-empty. If t is outside the signal domain, the
// nearest endpoint value is returned.
static double signal_value_at(const Signal& sig, double t) {
    if (sig.empty()) return 0.0;
    const Sample* active = &sig.front();
    for (const auto& s : sig) {
        if (s.time <= t) active = &s;
        else break;
    }
    return active->valueAt(t);
}

// Collects representative values of a piecewise-linear signal in [t_lo, t_hi].
// Returns interpolated values at the endpoints plus values at all breakpoints
// strictly inside the window. For smooth max/min, these are the points where
// the exact extremum can be achieved.
static std::vector<double> extract_signal_values(const Signal& sig,
                                                  double t_lo, double t_hi) {
    std::vector<double> vals;
    if (sig.empty() || t_lo >= t_hi) return vals;

    // Clamp to signal domain
    double domain_lo = sig.front().time;
    double domain_hi = sig.endTime;
    t_lo = std::max(t_lo, domain_lo);
    t_hi = std::min(t_hi, domain_hi);
    if (t_lo >= t_hi) return vals;

    // Value at t_lo (interpolated)
    vals.push_back(signal_value_at(sig, t_lo));

    // All breakpoints strictly inside (t_lo, t_hi)
    for (const auto& s : sig) {
        if (s.time > t_lo && s.time < t_hi)
            vals.push_back(s.value);
    }

    // Value at t_hi (interpolated)
    vals.push_back(signal_value_at(sig, t_hi));

    return vals;
}

// ---------------------------------------------------------------------------
// Boolean operators
// ---------------------------------------------------------------------------

// not: negation passes through without approximation
double not_transducer::compute_smooth_robustness(double tau, SmoothType type) {
    return -child->compute_smooth_robustness(tau, type);
}

// and <-> smooth min of two children
double and_transducer::compute_smooth_robustness(double tau, SmoothType type) {
    double rL = childL->compute_smooth_robustness(tau, type);
    double rR = childR->compute_smooth_robustness(tau, type);
    return smooth_min2(rL, rR, tau, type);
}

// or <-> smooth max of two children
double or_transducer::compute_smooth_robustness(double tau, SmoothType type) {
    double rL = childL->compute_smooth_robustness(tau, type);
    double rR = childR->compute_smooth_robustness(tau, type);
    return smooth_max2(rL, rR, tau, type);
}

// implies: phi => psi  ≡  (not phi) or psi
double implies_transducer::compute_smooth_robustness(double tau, SmoothType type) {
    double rL = -childL->compute_smooth_robustness(tau, type);
    double rR =  childR->compute_smooth_robustness(tau, type);
    return smooth_max2(rL, rR, tau, type);
}

// ---------------------------------------------------------------------------
// Temporal operators – smooth aggregation over a time window
//
// Strategy: compute the child's exact robustness Signal (full trace), then
// apply smooth_max / smooth_min over the breakpoint values in the window.
// This gives a differentiable scalar w.r.t. the trajectory while reusing the
// existing Signal machinery for trace computation.
// ---------------------------------------------------------------------------

// ev_[a,b] phi  <->  smooth_max_{t in [a,b]} rho(phi, t)
double ev_transducer::compute_smooth_robustness(double tau, SmoothType type) {
    double a, b;
    if (!get_param(I->begin_str, a)) a = I->begin;
    if (!get_param(I->end_str,   b)) b = I->end;

    child->compute_robustness(); // populates child->z over [start_time+a, end_time+b]

    double t_lo = start_time + a;
    double t_hi = start_time + b;

    std::vector<double> vals = extract_signal_values(child->z, t_lo, t_hi);
    if (vals.empty()) return child->z.empty() ? 0.0 : child->z.front().value;

    return smooth_max(vals, tau, type);
}

// alw_[a,b] phi  <->  smooth_min_{t in [a,b]} rho(phi, t)
double alw_transducer::compute_smooth_robustness(double tau, SmoothType type) {
    double a, b;
    if (!get_param(I->begin_str, a)) a = I->begin;
    if (!get_param(I->end_str,   b)) b = I->end;

    child->compute_robustness(); // populates child->z

    double t_lo = start_time + a;
    double t_hi = start_time + b;

    std::vector<double> vals = extract_signal_values(child->z, t_lo, t_hi);
    if (vals.empty()) return child->z.empty() ? 0.0 : child->z.front().value;

    return smooth_min(vals, tau, type);
}

// phi U_[a,b] psi  <->
//   smooth_max_{t in [a,b]} { smooth_min( smooth_min_{tau in [0,t]} rho(phi,tau),
//                                          rho(psi, t) ) }
//
// Implementation:
//   1. Compute exact traces for both subformulas.
//   2. Collect candidate timesteps from both traces within [a, b].
//   3. For each candidate t, compute a running smooth_min of phi over [0,t]
//      and evaluate psi at t, then take their smooth_min.
//   4. Take smooth_max over all candidate values.
double until_transducer::compute_smooth_robustness(double tau, SmoothType type) {
    double a, b;
    if (!get_param(I->begin_str, a)) a = I->begin;
    if (!get_param(I->end_str,   b)) b = I->end;

    childL->compute_robustness(); // phi trace
    childR->compute_robustness(); // psi trace

    const Signal& phi_sig = childL->z;
    const Signal& psi_sig = childR->z;

    if (phi_sig.empty() || psi_sig.empty()) return 0.0;

    double t_lo = start_time + a;
    double t_hi = start_time + b;

    // Clamp window to available signal data
    t_lo = std::max(t_lo, std::max(phi_sig.front().time, psi_sig.front().time));
    t_hi = std::min(t_hi, std::min(phi_sig.endTime, psi_sig.endTime));
    if (t_lo > t_hi) return 0.0;

    // Collect candidate evaluation times: endpoints + breakpoints of both traces
    std::vector<double> candidates;
    candidates.push_back(t_lo);
    for (const auto& s : phi_sig)
        if (s.time > t_lo && s.time < t_hi) candidates.push_back(s.time);
    for (const auto& s : psi_sig)
        if (s.time > t_lo && s.time < t_hi) candidates.push_back(s.time);
    candidates.push_back(t_hi);
    std::sort(candidates.begin(), candidates.end());
    candidates.erase(std::unique(candidates.begin(), candidates.end()),
                     candidates.end());

    // For each candidate t, compute smooth_min(min_phi_[0,t], psi_t)
    std::vector<double> outer_vals;
    outer_vals.reserve(candidates.size());

    for (double t : candidates) {
        // smooth_min of phi over [start_time, t]
        std::vector<double> phi_vals = extract_signal_values(phi_sig, start_time, t);
        double min_phi;
        if (phi_vals.empty()) {
            min_phi = signal_value_at(phi_sig, start_time);
        } else {
            min_phi = smooth_min(phi_vals, tau, type);
        }

        double psi_t = signal_value_at(psi_sig, t);

        outer_vals.push_back(smooth_min2(min_phi, psi_t, tau, type));
    }

    return smooth_max(outer_vals, tau, type);
}

} // namespace STLRom
