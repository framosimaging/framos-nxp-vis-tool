#include "CLI11.hpp"
#include "buffers.hpp"
#include "image_processor.hpp"
#include "v4l2_subdevice_controls.hpp"
#include "viv_controls.hpp"
#include "ioctl_cmds.h"

#include <nlohmann/json.hpp>
#include <ctime>
#include <cstring>
#include <cstdint>
#include <iostream>
#include <unistd.h>
#include <string>
#include <vector>
#include <chrono>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <dirent.h>
#include <fnmatch.h>
#include <fcntl.h>
#include <fstream>

#include <linux/videodev2.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

#define CL_HPP_TARGET_OPENCL_VERSION 300

using json = nlohmann::json;

bool xioctl(int fd, unsigned long request, void* arg);

enum BitWidth {
  EightBits = 8,
  TenBits = 10,
  TwelveBits = 12
};

const std::map<std::string, uint32_t> abbrev_to_pixel_fmt = {
    {"GRAY", V4L2_PIX_FMT_GREY}, // not supported
    {"Y10", V4L2_PIX_FMT_Y10},   // not supported
    {"Y12", V4L2_PIX_FMT_Y12},   // not supported
    {"YUYV", V4L2_PIX_FMT_YUYV},
    {"NV12", V4L2_PIX_FMT_NV12}, 
    {"NV16", V4L2_PIX_FMT_NV16},
    {"RG10", V4L2_PIX_FMT_SRGGB10},
    {"RG12", V4L2_PIX_FMT_SRGGB12},
};

//TODO: use this
const std::map<std::string, uint8_t> abbrev_to_memory_fmt = {
  {"mmap", V4L2_MEMORY_MMAP},
  {"dma", V4L2_MEMORY_DMABUF},
};

const std::map<BitWidth, uint32_t> bits_to_pixel_fmt = {
    {EightBits, V4L2_PIX_FMT_GREY},
    {TenBits, V4L2_PIX_FMT_Y10},
    //{TwelveBits, V4L2_PIX_FMT_SRGGB12}
    {TwelveBits, V4L2_PIX_FMT_NV12}
};

const std::map<BitWidth, uint8_t> conversion_factor = {
    {EightBits, 1},
    {TenBits, 4},
    {TwelveBits, 16}
};

struct CameraConfiguration {
    std::string camera_id = "/dev/video2";
    std::string subdevice_id = "/dev/v4l-subdev1";
    std::vector<uint32_t> resize = {0, 0};
    std::vector<uint32_t>  resolution = {1920, 1080};
    std::string pixel_format = "NV12";
    uint16_t frame_rate = 30;
    BitWidth bit_width = TwelveBits;
    bool profile = false;
    bool use_gpu = true;
};

static int ParseArguments(int argc, char **argv, json *json_config, CameraConfiguration *cam_conf, int* dma_mem) {
    CLI::App app{"Display Image Example CLI"};

    std::string config_file="config.json";
    json config;
    app.add_option("-b,--bit-width", cam_conf->bit_width, "Bit width 8, 10, 12, default -b 12")
    	->check(CLI::IsMember({8, 10, 12}));
    app.add_option("-m,--dma-mem", *dma_mem, "add this flag -m for choosing memory type 0 - mmap, 1 - cma, 2 dma-gpu")
    	->check(CLI::IsMember({0, 1, 2}));
    app.add_option("-c,--config", config_file, "Path to JSON configuration file, default is config.json");

    try {
    CLI11_PARSE(app, argc, argv);
    } catch (const CLI::ParseError &e) {
        int exitCode = app.exit(e);
        if (exitCode < 0){
            std::cerr << "Error encountered during CLI parsing. Exiting...\n";
        }
        return exitCode; 
    }

    try {
        std::ifstream file(config_file);
        if (!file.is_open()) {
            throw std::runtime_error("Could not open configuration file: " + config_file);
        }
        file >> config;
	cam_conf->camera_id = config.value("camera_id", cam_conf->camera_id);
        cam_conf->subdevice_id = config.value("subdevice_id", cam_conf->subdevice_id);
        cam_conf->resize = config.value("resize", cam_conf->resize);
        cam_conf->resolution = config.value("resolution", cam_conf->resolution);
        cam_conf->pixel_format = config.value("pixel_format", "NV12");
        cam_conf->profile = config.value("verbose", cam_conf->profile);
	cam_conf->use_gpu = config.value("use_gpu", cam_conf->use_gpu);
    } catch (const std::exception& e) {
        std::cerr << "Error reading configuration: " << e.what() << std::endl;
        return 1;
    }

    // Display the configuration
    std::cout << "Configuration loaded:" << std::endl
	<< "\t Camera ID: " << cam_conf->camera_id << std::endl
	<< "\t Subdevice ID: " << cam_conf->subdevice_id << std::endl
	<< "\t Resolution: " << cam_conf->resolution[0] << "x" << cam_conf->resolution[1] << std::endl
	<< "\t Resize: " << cam_conf->resize[0] << "x" << cam_conf->resize[1] << std::endl
	<< "\t Pixel Format: " << cam_conf->pixel_format << std::endl
	<< "\t Frame rate: " << cam_conf->frame_rate << std::endl
	//<< "Bit Width: " << (settings.bit_width == EightBits ? "8" : settings.bit_width == TenBits ? "10" : "12") << " bits\n"
	<< "\t Profile: " << (cam_conf->profile ? "Enabled" : "Disabled") << std::endl ;

    *json_config = config;
    return 1;
}

// other fields are set in driver
static int SetFormat(int fd, CameraConfiguration cam_conf, v4l2_format *format) {
    format->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format->fmt.pix.pixelformat = abbrev_to_pixel_fmt.at(cam_conf.pixel_format);
    std::cout << "camera configuration pixel format: " << cam_conf.pixel_format << " V4l2 PIX FMT: " << format->fmt.pix.pixelformat << std::endl;

    format->fmt.pix.width = cam_conf.resolution.at(0);
    format->fmt.pix.height = cam_conf.resolution.at(1);
    std::cout << " VIDIOC_S_FMT " << format->fmt.pix.width << " x " << format->fmt.pix.height << std::endl;

    if (xioctl(fd, VIDIOC_S_FMT, format) < 0) {
        std::cerr << "Format set error " << std::endl;
        return -1;
    }
    return 0;
}

int main(int argc, char **argv) {
    CameraConfiguration cam_conf;
    int dma_fd;
    int dma_mem = 0;
    json json_config;

    int ret = ParseArguments(argc, argv, &json_config, &cam_conf, &dma_mem);
    if (ret < 1) {
        std::cerr << "Failed to parse arguments " << ret << std::endl;
        return ret;
    }
    std::cout << "profiling = " << cam_conf.profile << std::endl;
    std::cout << "dma mem = " << dma_mem << std::endl;

    // Open video device
    int fd = ::open(cam_conf.camera_id.c_str(), O_RDWR);
    if (fd == -1) {
        std::cerr << "Open failed" << std::endl;
        return 1;
    }

    struct v4l2_format format;
    ret = SetFormat(fd, cam_conf, &format);
    if (ret < 0) {
        std::cerr << "Set format failed" << std::endl;
        return ret;
    }

    /* 
    //Example how to set vivante controls from ISP,

    // reading auto exposure configuration
    std::cout << "Calling viv get control" << std::endl;
    ret = viv_get_ctrl(fd, IF_AE_G_EN);
    if (ret > 0) {
	std::cerr << "Getting viv controls failed" << std::endl;
	return 1;
    }

    // setting auto exposure configuration to false
    std::cout << "Calling viv set control" << std::endl;
    ret = viv_set_ctrl(fd, IF_AE_S_EN, "enable", false);
    if (ret > 0) {
	std::cerr << "Setting viv controls failed" << std::endl;
	return 1;
    }

    // check if the value was set properly
    std::cout << "Calling viv get control" << std::endl;
    ret = viv_get_ctrl(fd, IF_AE_G_EN);
    if (ret > 0) {
	std::cerr << "Getting viv controls failed" << std::endl;
	return 1;
    }
    */

    // Open subdevice and set v4l2 controls, if you plan to use default settings for
    // frame rate, data rate, shutter, etc. you can remove this part
    // For exposure and gain it is recommended to use viv controls.
    json v4l2_controls = json_config["v4l2_subdevice_config"];
    V4l2Subdevice v4l2_subdevice(cam_conf.subdevice_id, v4l2_controls);
    v4l2_subdevice.run();

    v4l2_pix_format pix_fmt = format.fmt.pix;
    size_t buffer_size = pix_fmt.sizeimage;

    std::unique_ptr<V4l2Buffers> buffers; 
    if (dma_mem == 0) {
      buffers = std::make_unique<MMAPBuffers>(fd, dma_mem, pix_fmt.sizeimage);
    } else if (dma_mem == 1) {
      buffers = std::make_unique<DMABuffers>(fd, dma_mem, pix_fmt.sizeimage);
    }

    std::cout << "Requesting Buffers " << std::endl;
    ret = buffers->RequestBuffers();
    if (!ret) {
        std::cerr << "Requesting Buffers failed" << std::endl;
        return 1;
    }
    ret = buffers->AllocateBuffers();
    if (!ret) {
        std::cerr << "Initializing buffers failed" << std::endl;
        return 1;
    }
    ret = buffers->QueueAllBuffers();
    if (!ret) {
        std::cerr << "Initializing buffers failed" << std::endl;
        return 1;
    }
    ret = buffers->StartStream();
    if (!ret) {
        std::cerr << "Start Streaming failed" << std::endl;
        return 1;
    }

    int width = cam_conf.resolution[0];
    int height = cam_conf.resolution[1];
    std::unique_ptr<ImageProcessor> image_processor;
    bool raw_stream = false;

    if ((pix_fmt.pixelformat == V4L2_PIX_FMT_YUYV) || (pix_fmt.pixelformat == V4L2_PIX_FMT_NV12) || (pix_fmt.pixelformat == V4L2_PIX_FMT_NV16) ){
      raw_stream = false;
      image_processor = std::make_unique<G2DImageProcessor>(width, height, pix_fmt.pixelformat);
    }
    else {
      raw_stream = true;
      if (!cam_conf.use_gpu)
          image_processor = std::make_unique<CPUImageProcessor>(width, height, pix_fmt.pixelformat);
      else
          image_processor = std::make_unique<GPUImageProcessor>(width, height, pix_fmt.pixelformat);
    }

    auto ts_loop = std::chrono::system_clock::now();
    int fps = 0;
    uint8_t *image_data;
    while (true) {
	ret = buffers->DequeueBuffers(&image_data);

	if (!raw_stream) {
	    image_processor->ProcessImage(&buffers->buf_addr);
	} else {
	    image_processor->ProcessImage(image_data);
	}

	ret |= buffers->QueueBuffers();
	int key = cv::pollKey();
	if (key == 27 || key == 'q' || key == 'Q') { // ESC key
            std::cout << "Exiting..." << std::endl;
            break;
        } else if (key == 's' || key == 'S') {
	  ret = image_processor->SaveImage();
	  if (!ret) {
		return 1;
	  }
	}

	if (cam_conf.profile) {
		fps++;
		if (std::chrono::system_clock::now() - ts_loop > std::chrono::seconds(1)) {
			ts_loop = std::chrono::system_clock::now();
			std::cout << "FPS: " << fps << std::endl;
			fps = 0;
		}
		image_processor->PrintProfiling();
	};
    }

    cv::destroyAllWindows();
    std::cout << "Stop streaming..." << std::endl;
    buffers->StopStream();
    std::cout << "Release buffers..." << std::endl;
    buffers->ReleaseBuffers();
    ::close(fd);

    return 0;
}


bool xioctl(int fd, unsigned long request, void* arg) {
    uint32_t retry_attempts = 0;
    static constexpr uint32_t max_retry = 2;
    while (::ioctl(fd, request, arg) == -1 && retry_attempts < max_retry) {
        ++retry_attempts;
    }
    return retry_attempts != max_retry;
}
