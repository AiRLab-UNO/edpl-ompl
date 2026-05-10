/*
 * Visualizer stub implementation.
 * No GL/Qt; robot path is exported to the SQLite DB created by FIRM.
 */
#include "Visualization/Visualizer.h"
#include "Spaces/SE2BeliefSpace.h"
#include <sqlite3.h>
#include <iostream>

boost::mutex                             Visualizer::drawMutex_;
ompl::base::State*                       Visualizer::trueState_     = nullptr;
ompl::base::State*                       Visualizer::currentBelief_ = nullptr;
std::vector<ompl::base::State*>          Visualizer::robotPath_;
firm::SpaceInformation::SpaceInformationPtr Visualizer::si_;
Visualizer::VZRDrawingMode               Visualizer::mode_ = Visualizer::PRMViewMode;
bool                                     Visualizer::saveVideo_ = false;

void Visualizer::printRobotPathToFile(const std::string& logPath)
{
    if (robotPath_.empty()) return;

    std::string dbFile = logPath + "results.db";
    sqlite3* db = nullptr;
    if (sqlite3_open(dbFile.c_str(), &db) != SQLITE_OK)
    {
        std::cerr << "[Visualizer] Cannot open DB: " << dbFile << std::endl;
        return;
    }

    const char* createSQL =
        "CREATE TABLE IF NOT EXISTS robot_path ("
        "  step INTEGER, x REAL, y REAL, theta REAL);";
    char* errmsg = nullptr;
    sqlite3_exec(db, createSQL, nullptr, nullptr, &errmsg);
    if (errmsg) { sqlite3_free(errmsg); errmsg = nullptr; }

    sqlite3_exec(db, "BEGIN TRANSACTION;", nullptr, nullptr, nullptr);
    for (int i = 0; i < (int)robotPath_.size(); ++i)
    {
        if (!robotPath_[i]) continue;
        arma::colvec x = robotPath_[i]
            ->as<SE2BeliefSpace::StateType>()->getArmaData().subvec(0,2);
        std::string ins =
            "INSERT INTO robot_path VALUES (" +
            std::to_string(i) + "," +
            std::to_string(x[0]) + "," +
            std::to_string(x[1]) + "," +
            std::to_string(x[2]) + ");";
        sqlite3_exec(db, ins.c_str(), nullptr, nullptr, nullptr);
    }
    sqlite3_exec(db, "COMMIT;", nullptr, nullptr, nullptr);
    sqlite3_close(db);
}
