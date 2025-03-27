#define VIV_JSON_BUFFER_SIZE  (64*1024)
#define VIV_CTRL_NAME "viv_ext_ctrl"
#define VIV_CUSTOM_CID_BASE (V4L2_CID_USER_BASE | 0xf000)
//#define V4L2_CID_VIV_EXTCTRL (VIV_CUSTOM_CID_BASE + 1)
#define V4L2_CID_VIV_EXTCTRL 0x0098F901

#include "viv_controls.hpp"

#include <cstring>
#include <iostream>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <ioctl_cmds.h>

using json = nlohmann::json;

int viv_get_ctrl(int fd, const char* config_id) {
	struct v4l2_queryctrl queryctrl;
	memset(&queryctrl, 0, sizeof(queryctrl));

	queryctrl.id = V4L2_CID_VIV_EXTCTRL;  // External control ID

	if (ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl) < 0) {
		perror("VIDIOC_QUERYCTRL failed (control may not exist)");
		return 1;
	}

	printf("Control Name: %s\n", queryctrl.name);
	printf("Control Type: %d\n", queryctrl.type);
	printf("Min Value: %d\n", queryctrl.minimum);
	printf("Max Value: %d\n", queryctrl.maximum);
	printf("Step: %d\n", queryctrl.step);
	printf("Default Value: %d\n", queryctrl.default_value);

	struct v4l2_ext_controls viv_ctrls;
	struct v4l2_ext_control viv_ctrl;
	memset(&viv_ctrls, 0, sizeof(viv_ctrls));
	memset(&viv_ctrl, 0, sizeof(viv_ctrl));

	viv_ctrl.string = new char[VIV_JSON_BUFFER_SIZE];
	viv_ctrl.id = V4L2_CID_VIV_EXTCTRL;
	viv_ctrl.size = VIV_JSON_BUFFER_SIZE;
	viv_ctrls.controls = &viv_ctrl;
	viv_ctrls.count = 1;

	json j;
        j["id"] = config_id;

	std::string json_dump = j.dump();
        const char* json_str = json_dump.c_str();

	strncpy(viv_ctrl.string, json_str, VIV_JSON_BUFFER_SIZE - 1);

	int ret = ioctl(fd, VIDIOC_S_EXT_CTRLS, &viv_ctrls);
	if (ret != 0) {
		std::cerr << "failed to set viv control error: " << ret << std::endl;
		return 1;
	}

	ret = ioctl(fd, VIDIOC_G_EXT_CTRLS, &viv_ctrls);
	if (ret != 0) {
		std::cout << "FAILED to get control ret: " << ret << std::endl;
		return 1;
	}

	std::string json_data(viv_ctrl.string);
        json parsed = json::parse(json_data);
	std::cout << "Parsed JSON: " << parsed.dump(4) << std::endl;
	
	if (parsed.contains("result")) {
		int result = parsed["result"];
		std::cout << "Result: " << result << std::endl;
		return result;
	} else {
	    std::cerr << "Result field not found in viv controls response "<< std::endl;
	    return 1;
	}
}

int viv_set_ctrl(int fd, const char* config_id, const std::string& key, const nlohmann::json& value) {
	struct v4l2_ext_controls viv_ctrls;
	struct v4l2_ext_control viv_ctrl;
	memset(&viv_ctrls, 0, sizeof(viv_ctrls));
	memset(&viv_ctrl, 0, sizeof(viv_ctrl));

	viv_ctrl.string = new char[VIV_JSON_BUFFER_SIZE];
	viv_ctrl.id = V4L2_CID_VIV_EXTCTRL;
	viv_ctrl.size = VIV_JSON_BUFFER_SIZE;

	json j;
        j["id"] = config_id;
	j[key] = value;

	std::string json_dump = j.dump();
        const char* json_str = json_dump.c_str();

	strncpy(viv_ctrl.string, json_str, VIV_JSON_BUFFER_SIZE - 1);

	viv_ctrls.controls = &viv_ctrl;
	viv_ctrls.count = 1;
	int ret = ioctl(fd, VIDIOC_S_EXT_CTRLS, &viv_ctrls);
	if (ret != 0) {
		std::cerr << "Failed to set viv control, ret: " << ret << std::endl;
		return ret;
	}
	std::cout << "Finished setting viv controls" << viv_ctrl.string << std::endl;

	return 0;
}