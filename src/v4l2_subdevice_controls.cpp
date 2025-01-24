#include "v4l2_subdevice_controls.hpp"
#include <iostream>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <errno.h>

V4l2Subdevice::V4l2Subdevice(const std::string& device_path, json v4l2_subdevice_config) : device_path_(device_path) {
  for (auto it = v4l2_subdevice_config.begin(); it != v4l2_subdevice_config.end(); ++it) {
    const std::string& control_name = it.key();
    int control_value = it.value();
    controls.emplace(control_name, V4l2Control(control_name, control_value, -1));
  }
}

V4l2Subdevice::~V4l2Subdevice() {
  closeSubdevice();
}

bool V4l2Subdevice::openSubdevice() {
    fd_ = ::open(device_path_.c_str(), O_RDWR);
    return fd_ >= 0;
}

void V4l2Subdevice::closeSubdevice() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    std::cout << "Subdevice closed: " << std::endl;
}

bool V4l2Subdevice::findControls() {
  struct v4l2_queryctrl queryctrl;

  for (auto i = V4L2_CID_BASE; i < V4L2_CID_USER_IMX_BASE + 16; i++) {
    queryctrl.id = i;
    if (ioctl(fd_, VIDIOC_QUERYCTRL, &queryctrl) == 0) {
      std::string control_name = reinterpret_cast<char*>(queryctrl.name);
      if (controls.find(control_name) != controls.end()) {
        std::cout << "Found control: " << control_name 
                  << " with ID: " << queryctrl.id << std::endl;
	controls.at(control_name).control_id = queryctrl.id;
	// TODO remove:
	std::cout << "Name: " << controls.at(control_name).name 
		  << "Value: " << controls.at(control_name).value 
		  << "Control_id: " << controls.at(control_name).control_id << std::endl;
      }
    } else // query fails, not critical
      continue;
  }

  return true;
}

bool V4l2Subdevice::setControls() {

  // when setting controls make sure to set data_rate, frame rate
  // and exposure are set in that order
  for (const auto& [control_name, control] : controls) {
    std::cout << "Setting control " << control_name
            << ", Value: " << control.value
            << ", Control ID: " << control.control_id << std::endl;

    v4l2_control ctrl = {0};
    ctrl.id = control.control_id; 
    ctrl.value = control.value;

    if (ioctl(fd_, VIDIOC_S_CTRL, &ctrl) < 0) {
        std::cerr << "Error setting control: " << strerror(errno) << std::endl;
        return false;
    }
  }
    return true;
}



/*
 This function is an example how to use this code in your project. If one just want to set up controls without changing 
 them later in project, you can just use this function. If you want to dynamically change some controls, you can use set_control 
*/
bool V4l2Subdevice::run() {

  if (!openSubdevice()){
    std::cerr << "Failed to open subdevice " << std::endl;
    return false;
  }

  if (!findControls())
    return false;

  if (!setControls())
    return false;

  return true;
}
// update v4l2-control values passed by command line
/*
static int updateControls(int subd) {
    for (auto &control : controls) {
	v4l2_control controlToSet;
	std::memset(&controlToSet, 0, sizeof(controlToSet));
	controlToSet.id = control.second.id;
	controlToSet.value = control.second.defaultValue;
	if (control.second.value != USE_DEFAULT) {
		controlToSet.value = control.second.value;
	}
	bool res = xioctl(subd, VIDIOC_S_CTRL, &controlToSet);
	if (!res) {
		std::cout << "Setting control failed " << std::endl;
		return -1;
	}
    }
    return 0;
}


    return false;
}
bool V4l2Subdevice::setControl(const std::string& control_name, int value) {
    v4l2_control control = {0};

    if (!findControl(control_name, control)) {
        return false;
    }

    control.id = control.id; 
    control.value = value;

    if (ioctl(fd_, VIDIOC_S_CTRL, &control) < 0) {
        std::cerr << "Error setting control: " << strerror(errno) << std::endl;
        return false;
    }

    return true;
}

bool V4l2Subdevice::getControl(const std::string& control_name, v4l2_control& control) {
    if (!findControl(control_name, control)) {
        return false;
    }

    if (ioctl(fd_, VIDIOC_G_CTRL, &control) < 0) {
        std::cerr << "Error getting control: " << strerror(errno) << std::endl;
        return false;
    }

    return true;
}
*/