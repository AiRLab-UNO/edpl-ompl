/*
 * FIRMCP + stl_bow_v2 integration benchmark (headless).
 *
 *   ./firmcp-stl-demo <yaml> <spec> [plan_seconds=15] [max_exec_steps=0]
 *
 * Two MCTS variants are exercised back-to-back:
 *   1) STL-guided rollout — FIRMCP whose rollout edge selection is biased by
 *      STL robustness (lower edge cost when target satisfies more of the spec).
 *   2) STLBOW rollout     — FIRMCP whose rollout selection mimics the STLBOW
 *      acquisition: STL_rob + UCB exploration bonus over edge visits.
 *
 * If max_exec_steps > 0 we run executeFeedbackWithPOMCP() up to that many
 * simulated steps per variant — that is the only way to actually exercise the
 * difference between the two rollouts. With max_exec_steps == 0 (default) the
 * driver only builds the roadmap (fast smoke test).
 *
 * Each run emits Results/firmcp_stl-<TS>/<variant>/{summary.txt, run-...}
 * plus a top-level benchmark.csv summarising both variants.
 */

#include <chrono>
#include <fstream>
#include <iostream>
#include <string>

#include <armadillo>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>
#include <ompl/util/Console.h>

#include "Planner/FIRMCP.h"
#include "Setup/STLFIRMCPSetup.h"

namespace
{
std::string isoTimestamp()
{
    auto now = boost::posix_time::second_clock::local_time();
    return boost::posix_time::to_iso_string(now);
}

struct VariantResult
{
    std::string label;
    double      planSeconds = 0.0;
    bool        solved      = false;
    double      execSeconds = 0.0;
    int         execSteps   = 0;
    bool        reachedGoal = false;
    double      robPhi   = -std::numeric_limits<double>::infinity();
    double      robReach = -std::numeric_limits<double>::infinity();
    double      robSafe  = -std::numeric_limits<double>::infinity();
};

VariantResult runVariant(const std::string&                    label,
                         const std::string&                    yamlPath,
                         const std::string&                    specPath,
                         const std::string&                    runRoot,
                         STLFIRMCPSetup::RolloutVariant        variant,
                         double                                planSeconds,
                         int                                   maxExecSteps)
{
    const std::string variantDir = runRoot + "/" + label;
    boost::filesystem::create_directories(variantDir);

    std::cout << "\n========================================\n";
    std::cout << " Variant : " << label << "\n";
    std::cout << " Output  : " << variantDir << "\n";
    std::cout << " Plan(s) : " << planSeconds << "  ExecSteps: " << maxExecSteps << "\n";
    std::cout << "========================================\n";

    VariantResult r;
    r.label = label;

    auto t0 = std::chrono::steady_clock::now();
    STLFIRMCPSetup setup(yamlPath, specPath, variantDir, variant);
    if (maxExecSteps > 0) setup.setMaxExecutionSteps(maxExecSteps);
    setup.setup();
    auto status = setup.solve(planSeconds);
    auto t1 = std::chrono::steady_clock::now();
    r.planSeconds = std::chrono::duration<double>(t1 - t0).count();
    r.solved      = static_cast<bool>(status);

    if (r.solved)
    {
        if (maxExecSteps > 0)
        {
            std::cout << "[" << label << "] running executeFeedbackWithPOMCP up to "
                      << maxExecSteps << " steps...\n";
            auto e0 = std::chrono::steady_clock::now();
            setup.runExecution();
            auto e1 = std::chrono::steady_clock::now();
            r.execSeconds = std::chrono::duration<double>(e1 - e0).count();
            r.execSteps   = setup.executionTimeStep();
            r.reachedGoal = setup.reachedGoal();
        }

        const auto& goals = setup.spec()->orderedGoals();
        if (!goals.empty())
        {
            // Score the planner's intent (start → ordered targets, straight
            // line). For a faithful score one would replay the executed
            // trajectory; this proxy is consistent across variants.
            std::vector<arma::vec2> path;
            arma::vec2 cur{setup.spec()->param("rx"),
                           setup.spec()->param("ry")};
            path.push_back(cur);
            for (const auto& g : goals)
                path.push_back(arma::vec2{g[0], g[1]});
            r.robPhi   = setup.stlEval()->score(path);
            r.robReach = setup.stlEval()->scoreReach(path);
            r.robSafe  = setup.stlEval()->scoreSafe(path);
        }
        // POMCP execution grows the graph with belief-tree nodes; saving the
        // post-exec roadmap can OOM the XML serialiser. Only dump the static
        // roadmap when we did NOT exercise the executor.
        if (maxExecSteps <= 0)
            setup.saveRoadmap();
    }
    else
    {
        OMPL_WARN("[%s] FIRMCP could not find a roadmap solution.", label.c_str());
    }

    std::ofstream summary(variantDir + "/summary.txt");
    summary << "variant: "          << label << "\n";
    summary << "yaml: "             << yamlPath << "\n";
    summary << "spec: "             << specPath << "\n";
    summary << "plan_seconds: "     << r.planSeconds << "\n";
    summary << "solved: "           << (r.solved ? "true" : "false") << "\n";
    summary << "exec_seconds: "     << r.execSeconds << "\n";
    summary << "exec_steps: "       << r.execSteps   << "\n";
    summary << "reached_goal: "     << (r.reachedGoal ? "true" : "false") << "\n";
    summary << "rob_phi: "          << r.robPhi   << "\n";
    summary << "rob_reach: "        << r.robReach << "\n";
    summary << "rob_safe: "         << r.robSafe  << "\n";
    summary << "num_landmarks: "    << setup.spec()->landmarks().size() << "\n";
    summary << "num_obstacles: "    << setup.spec()->obstacles().size() << "\n";

    std::cout << "[" << label << "] plan=" << r.planSeconds
              << "s  solved=" << r.solved
              << "  exec=" << r.execSeconds << "s/" << r.execSteps << "steps"
              << "  goal=" << r.reachedGoal
              << "  phi=" << r.robPhi
              << "  reach=" << r.robReach
              << "  safe="  << r.robSafe << "\n";
    return r;
}
}

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::cerr << "Usage: " << argv[0]
                  << " <yaml_config> <stl_spec> [plan_seconds=15] [max_exec_steps=0]\n"
                  << "  max_exec_steps=0 (default): roadmap-only smoke test\n"
                  << "  max_exec_steps>0          : run POMCP executor up to N steps/variant\n";
        return 1;
    }

    arma::arma_rng::set_seed_random();

    // Silence the OMPL planner chatter unless FIRMCP_VERBOSE=1.
    const bool verbose = []() {
        const char* v = std::getenv("FIRMCP_VERBOSE");
        return v && std::string(v) != "0";
    }();
    ompl::msg::setLogLevel(verbose ? ompl::msg::LOG_INFO : ompl::msg::LOG_NONE);
    FIRMCP::setVerbose(verbose);

    const std::string yamlPath = argv[1];
    const std::string specPath = argv[2];
    const double planSeconds   = (argc >= 4) ? std::stod(argv[3]) : 15.0;
    const int    maxExecSteps  = (argc >= 5) ? std::stoi(argv[4]) :  0;

    const std::string runRoot = std::string("Results/firmcp_stl-") + isoTimestamp();
    boost::filesystem::create_directories(runRoot);

    std::ofstream readme(runRoot + "/README.txt");
    readme << "FIRMCP + stl_bow_v2 benchmark run\n";
    readme << "yaml: " << yamlPath << "\n";
    readme << "spec: " << specPath << "\n";
    readme << "plan_seconds_per_variant: " << planSeconds << "\n";
    readme << "max_exec_steps_per_variant: " << maxExecSteps << "\n";
    readme.close();

    auto guided = runVariant("stl_guided", yamlPath, specPath, runRoot,
                             STLFIRMCPSetup::RolloutVariant::STL_GUIDED,
                             planSeconds, maxExecSteps);
    auto bow    = runVariant("stl_bow",    yamlPath, specPath, runRoot,
                             STLFIRMCPSetup::RolloutVariant::STL_BOW,
                             planSeconds, maxExecSteps);

    std::ofstream bench(runRoot + "/benchmark.csv");
    bench << "variant,plan_seconds,solved,exec_seconds,exec_steps,reached_goal,"
          << "rob_phi,rob_reach,rob_safe\n";
    auto row = [&](const VariantResult& r) {
        bench << r.label << "," << r.planSeconds << "," << r.solved << ","
              << r.execSeconds << "," << r.execSteps << ","
              << (r.reachedGoal ? 1 : 0) << ","
              << r.robPhi << "," << r.robReach << "," << r.robSafe << "\n";
    };
    row(guided);
    row(bow);

    std::cout << "\nResults: " << runRoot << "\n";
    return 0;
}
