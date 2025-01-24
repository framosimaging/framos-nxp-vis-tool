#pragma once

#include <string>
#include <unordered_map>

#define USE_DEFAULT (INT64_MIN)

struct SensorV4L2Control
{
    std::string name;
    unsigned int id;
    std::string cliName;
    std::string helpExplanation;
    int64_t value;
    int64_t defaultValue;
    SensorV4L2Control(std::string cliName, std::string helpExplanation, int64_t value)
                        : cliName(cliName), helpExplanation(helpExplanation), value(value) {
    }
};

std::unordered_map<std::string, SensorV4L2Control> controls = {
    {"Exposure", SensorV4L2Control("--exposure", "sensor exposure setting [us]", USE_DEFAULT)},
    {"Gain", SensorV4L2Control("--gain", "sensor gain setting", USE_DEFAULT)},
};