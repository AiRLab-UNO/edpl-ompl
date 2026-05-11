/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Texas A&M University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Texas A&M University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/


#include "Utils/FIRMUtils.h"
#include <boost/math/constants/constants.hpp>
#include <boost/date_time.hpp>
#include <utility>
#include <random>
#include <fstream>
#include <yaml-cpp/yaml.h>


void FIRMUtils::normalizeAngleToPiRange(double &theta)
{

    while(theta > boost::math::constants::pi<double>())
    {
        theta -= 2*boost::math::constants::pi<double>();
    }

    while(theta < -boost::math::constants::pi<double>())
    {
        theta += 2*boost::math::constants::pi<double>();
    }

}

int FIRMUtils::signum(const double d)
{
        if(d>0)
            return 1;

        if(d<0)
            return -1;
}

int FIRMUtils::generateRandomIntegerInRange(const int floor, const int ceiling)
{
    //std::random_device rd; // obtain a random number from hardware

    //std::mt19937 eng(rd()); // seed the generator

    //std::uniform_int_distribution<> distr(floor, ceiling); // define the range

    //return distr(eng);

    int r = rand()%(ceiling - floor + 1) + floor;

    return r;
}

void FIRMUtils::writeFIRMGraphToYAML(const std::vector<std::pair<int,std::pair<arma::colvec,arma::mat> > > nodes, const std::vector<std::pair<std::pair<int,int>,FIRMWeight> > edgeWeights, const std::string &outputPath)
{
    YAML::Emitter out;
    out << YAML::BeginMap;

    out << YAML::Key << "nodes" << YAML::Value << YAML::BeginSeq;
    for (int i = 0; i < (int)nodes.size(); i++)
    {
        int nodeID        = nodes[i].first;
        arma::colvec xVec = nodes[i].second.first;
        arma::mat cov     = nodes[i].second.second;

        out << YAML::BeginMap;
        out << YAML::Key << "id"    << YAML::Value << nodeID;
        out << YAML::Key << "x"     << YAML::Value << xVec(0);
        out << YAML::Key << "y"     << YAML::Value << xVec(1);
        out << YAML::Key << "theta" << YAML::Value << xVec(2);
        out << YAML::Key << "cov" << YAML::Value << YAML::Flow
            << YAML::BeginSeq
            << cov(0,0) << cov(0,1) << cov(0,2)
            << cov(1,0) << cov(1,1) << cov(1,2)
            << cov(2,0) << cov(2,1) << cov(2,2)
            << YAML::EndSeq;
        out << YAML::EndMap;
    }
    out << YAML::EndSeq;

    out << YAML::Key << "edges" << YAML::Value << YAML::BeginSeq;
    for (int i = 0; i < (int)edgeWeights.size(); i++)
    {
        FIRMWeight w = edgeWeights[i].second;
        out << YAML::BeginMap;
        out << YAML::Key << "start" << YAML::Value << edgeWeights[i].first.first;
        out << YAML::Key << "end"   << YAML::Value << edgeWeights[i].first.second;
        out << YAML::Key << "success_prob" << YAML::Value << w.getSuccessProbability();
        out << YAML::Key << "cost"         << YAML::Value << w.getCost();
        out << YAML::EndMap;
    }
    out << YAML::EndSeq;
    out << YAML::EndMap;

    namespace pt = boost::posix_time;
    pt::ptime now = pt::second_clock::local_time();
    std::string timeStamp(to_iso_string(now));
    std::string roadmapFileName = outputPath + "FIRMRoadMap-" + timeStamp + ".yaml";

    std::ofstream fout(roadmapFileName);
    fout << out.c_str();
}

bool FIRMUtils::readFIRMGraphFromYAML(const std::string &pathToYAML, std::vector<std::pair<int, arma::colvec> > &FIRMNodePosList, std::vector<std::pair<int, arma::mat> > &FIRMNodeCovarianceList, std::vector<std::pair<std::pair<int,int>,FIRMWeight> > &edgeWeights)
{
    YAML::Node doc;
    try {
        doc = YAML::LoadFile(pathToYAML);
    } catch (const YAML::Exception&) {
        OMPL_INFORM("FIRMUtils: Could not load Graph from YAML. Need to construct graph.");
        return false;
    }

    for (const auto& n : doc["nodes"])
    {
        int id = n["id"].as<int>();
        arma::colvec xVec(3);
        xVec(0) = n["x"].as<double>();
        xVec(1) = n["y"].as<double>();
        xVec(2) = n["theta"].as<double>();

        auto covSeq = n["cov"];
        arma::mat cov(3,3);
        cov(0,0) = covSeq[0].as<double>(); cov(0,1) = covSeq[1].as<double>(); cov(0,2) = covSeq[2].as<double>();
        cov(1,0) = covSeq[3].as<double>(); cov(1,1) = covSeq[4].as<double>(); cov(1,2) = covSeq[5].as<double>();
        cov(2,0) = covSeq[6].as<double>(); cov(2,1) = covSeq[7].as<double>(); cov(2,2) = covSeq[8].as<double>();

        FIRMNodePosList.push_back(std::make_pair(id, xVec));
        FIRMNodeCovarianceList.push_back(std::make_pair(id, cov));
    }

    for (const auto& e : doc["edges"])
    {
        int start       = e["start"].as<int>();
        int end         = e["end"].as<int>();
        double succProb = e["success_prob"].as<double>();
        double cost     = e["cost"].as<double>();
        FIRMWeight w(cost, succProb);
        edgeWeights.push_back(std::make_pair(std::make_pair(start, end), w));
    }

    return true;
}

double FIRMUtils::degree2Radian(double deg)
{
    return boost::math::constants::pi<double>()*deg/180.0;
}

double FIRMUtils::radian2Degree(double rads)
{
    return rads*180.0/boost::math::constants::pi<double>();
}

