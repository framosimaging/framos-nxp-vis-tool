#ifndef V4L2_SUBDEVICE_CONTROLS_H
#define V4L2_SUBDEVICE_CONTROLS_H

#include <string>
#include <vector>
#include <linux/videodev2.h>
#include <nlohmann/json.hpp>
using json = nlohmann::json;
const std::vector<std::string> kV4L2Subdevice_control_names = {"Frame rate", "Data rate", "Shutter mode", "Exposure", "Gain"};

struct SubdeviceControls {
    uint16_t frame_rate = 30;
    bool profile = false;
};

// Structure to represent a single control
struct V4l2Control {
    std::string name;
    int value; // all our controls take integers as values, can be extended
    int control_id; // Control ID (initialized to -1) 

    V4l2Control(const std::string& name, int value, int control_id = -1)
        : name(name), value(value), control_id(control_id) {}
};

class V4l2Subdevice {
public:
    V4l2Subdevice(const std::string& device_path, const json v4l2_subdevice_config);
    ~V4l2Subdevice();

    bool openSubdevice();
    void closeSubdevice();
    bool findControls();
    bool setControls();
    bool run();

private:
    std::string device_path_;
    // std::vector<std::string> control_names_ = kV4L2Subdevice_control_names;
    int fd_ = -1; 
    std::unordered_map<std::string, V4l2Control> controls;
};

#endif // V4L2_SUBDEVICE_CONTROLS_H