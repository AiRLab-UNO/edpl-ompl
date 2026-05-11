#include "STL/SpecLandmarks.h"

#include <fstream>
#include <iostream>
#include <map>
#include <regex>
#include <sstream>

namespace firmcp_stl
{

namespace
{
std::unordered_map<std::string, double> parseParamLine(const std::string& line)
{
    std::unordered_map<std::string, double> out;
    auto pos = line.find("param");
    if (pos == std::string::npos)
        return out;

    std::string body = line.substr(pos + 5);
    std::stringstream ss(body);
    std::string pair;
    while (std::getline(ss, pair, ','))
    {
        auto eq = pair.find('=');
        if (eq == std::string::npos)
            continue;
        std::string name  = pair.substr(0, eq);
        std::string value = pair.substr(eq + 1);
        auto trim = [](std::string& s) {
            const char* ws = " \t\r\n";
            s.erase(0, s.find_first_not_of(ws));
            auto last = s.find_last_not_of(ws);
            if (last != std::string::npos)
                s.erase(last + 1);
        };
        trim(name);
        trim(value);
        try
        {
            out[name] = std::stod(value);
        }
        catch (...)
        {
            // ignore non-numeric params
        }
    }
    return out;
}

template <typename Container>
void emitOrdered(const std::map<int, std::array<double, 2>>& src, Container& dst)
{
    dst.clear();
    dst.reserve(src.size());
    for (const auto& [_, p] : src)
        dst.push_back(p);
}
} // namespace

bool SpecLandmarks::load(const std::string& specFile)
{
    std::ifstream in(specFile);
    if (!in.is_open())
    {
        std::cerr << "[SpecLandmarks] Could not open spec: " << specFile << std::endl;
        return false;
    }

    std::stringstream buffer;
    buffer << in.rdbuf();
    specText_ = buffer.str();

    params_.clear();
    landmarks_.clear();
    obstacles_.clear();
    orderedGoals_.clear();

    {
        std::stringstream ss(specText_);
        std::string line;
        while (std::getline(ss, line))
        {
            for (auto& kv : parseParamLine(line))
                params_[kv.first] = kv.second;
        }
    }

    std::regex rxGoalX(R"(^gx(\d+)$)");
    std::regex rxGoalY(R"(^gy(\d+)$)");
    std::regex rxObsX(R"(^ox(\d+)_x$)");
    std::regex rxObsY(R"(^ox(\d+)_y$)");

    std::map<int, double> gx, gy, ox, oy;
    for (const auto& [k, v] : params_)
    {
        std::smatch m;
        if (std::regex_match(k, m, rxGoalX))
            gx[std::stoi(m[1].str())] = v;
        else if (std::regex_match(k, m, rxGoalY))
            gy[std::stoi(m[1].str())] = v;
        else if (std::regex_match(k, m, rxObsX))
            ox[std::stoi(m[1].str())] = v;
        else if (std::regex_match(k, m, rxObsY))
            oy[std::stoi(m[1].str())] = v;
    }

    std::map<int, std::array<double, 2>> goalMap;
    for (const auto& [idx, x] : gx)
    {
        auto it = gy.find(idx);
        if (it == gy.end())
            continue;
        goalMap[idx] = {x, it->second};
    }
    emitOrdered(goalMap, orderedGoals_);

    landmarks_.reserve(orderedGoals_.size());
    for (size_t i = 0; i < orderedGoals_.size(); ++i)
    {
        arma::colvec lm(3);
        lm(0) = static_cast<double>(i);
        lm(1) = orderedGoals_[i][0];
        lm(2) = orderedGoals_[i][1];
        landmarks_.push_back(lm);
    }

    for (const auto& [idx, x] : ox)
    {
        auto it = oy.find(idx);
        if (it == oy.end())
            continue;
        obstacles_.push_back({x, it->second});
    }

    return true;
}

} // namespace firmcp_stl
