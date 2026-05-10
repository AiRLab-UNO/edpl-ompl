/*
 * FIRMCP demo — headless (no Qt/GL).
 * Results are saved to a SQLite database in the run log directory.
 */
#include <iostream>
#include "Setup/TwoDPointRobotFIRMCPSetup.h"

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <setup_file.xml>" << std::endl;
        return 1;
    }

    arma::arma_rng::set_seed_random();

    TwoDPointRobotFIRMCPSetup mySetup;
    mySetup.setPathToSetupFile(argv[1]);
    mySetup.setup();

    Visualizer::setMode(Visualizer::VZRDrawingMode::PRMViewMode);

    if (mySetup.solve())
    {
        mySetup.Run();
        OMPL_INFORM("Plan executed. Results written to SQLite DB.");
    }
    else
    {
        OMPL_WARN("Unable to find solution in given time.");
        return 1;
    }

    OMPL_INFORM("Execution complete.");
    return 0;
}
