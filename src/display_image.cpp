#define IMX_G2D 1
//#define USE_OPENCV 1

#include "CLI11.hpp"
#include "v4l2_subdevice_controls.hpp"
#include "buffers.hpp"

#include <nlohmann/json.hpp>
#include <ctime>
#include <cstring>
#include <cstdint>
#include <iostream>
#include <unistd.h>
#include <string>
//#include <algorithm>
#include <vector>
#include <chrono>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <dirent.h>
#include <fnmatch.h>
#include <fcntl.h>

#include <linux/videodev2.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
//#include <opencv2/core/ocl.hpp>

#include <imx/linux/dma-buf.h>
#include <g2d.h>
#include <g2dExt.h>

using json = nlohmann::json;


bool xioctl(int fd, unsigned long request, void* arg);
void DrawFps(cv::Mat &mat, uint32_t displayFps);


enum BitWidth {
  EightBits = 8,
  TenBits = 10,
  TwelveBits = 12
};

const std::map<std::string, uint32_t> abbrev_to_pixel_fmt = {
    {"GRAY", V4L2_PIX_FMT_GREY},
    {"Y10", V4L2_PIX_FMT_Y10},
    {"Y12", V4L2_PIX_FMT_Y12},
    {"YUYV", V4L2_PIX_FMT_YUYV},
    {"NV12", V4L2_PIX_FMT_NV12},
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

struct ProfileApp {
    long long deque;
    long long show;
    long long draw_fps;
    long long queue;
    long long visualize;
    long long copy_buffer;
    long long copy_buffer2;
    long long resize;
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
};

static int ParseArguments(int argc, char **argv, nlohmann::json *json_config, CameraConfiguration *cam_conf, int* dma_mem) {
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
    } catch (const std::exception& e) {
        std::cerr << "Error reading configuration: " << e.what() << std::endl;
        return 1;
    }

    // Display the configuration
    std::cout << "Configuration loaded:"
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

static void print_profile_time(struct ProfileApp *profile_time, int visualize_num) {
    // only resize, convert, deque and show are relevent
    //std::cout << "Deque time: " << profile_time->deque / visualize_num << " ms" << std::endl;
    std::cout << "Copy buffer time: " << profile_time->copy_buffer / visualize_num << " ms" << std::endl;
    
    //std::cout << "Resize image time: " << profile_time->resize / visualize_num << " ms" << std::endl;
    //std::cout << "Convert image time: " << profile_time->visualize / visualize_num << " ms" << std::endl;
    //std::cout << "Draw fps time: " << profile_time->draw_fps / visualize_num << " ms" << std::endl;
    std::cout << "Show image time: " << profile_time->show / visualize_num << " ms" << std::endl;
    //std::cout << "Queue buffer time: " << profile_time->queue / visualize_num << " ms " << std::endl;

    profile_time->deque = 0;
    profile_time->copy_buffer = 0;
    profile_time->copy_buffer2 = 0;
    profile_time->resize = 0;
    profile_time->visualize = 0;
    profile_time->draw_fps = 0;
    profile_time->show = 0;
    profile_time->queue = 0;
}

int main(int argc, char **argv) {
    CameraConfiguration cam_conf;
    int dma_fd;
    int dma_mem = 0;
    nlohmann::json json_config;

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

    // Open subdevice and set v4l2 controls, if you plan to use default settings for
    // frame rate, data rate, shutter, etc. you can remove this part
    // For exposure and gain it is recommended to use viv controls.
    json v4l2_controls = json_config["v4l2_subdevice_config"];
    V4l2Subdevice v4l2_subdevice(cam_conf.subdevice_id, v4l2_controls);
    v4l2_subdevice.run();

    v4l2_pix_format pix_fmt = format.fmt.pix;
    size_t buffer_size = pix_fmt.sizeimage;

    //void *user_buf = malloc(pix_fmt.sizeimage);
    //void* temp_buffer = malloc(1920 * 1080 * 4);
    /*if (!user_buf) {
	std::cerr << "Failed to allocate user buffer" << std::endl;
        return 1;
    }
    */
    void *user_buf = malloc(pix_fmt.sizeimage);
    std::unique_ptr<V4l2Buffers> buffers; 
    if (dma_mem == 0) {
      buffers = std::make_unique<MMAPBuffers>(fd, dma_mem, pix_fmt.sizeimage);
    }
    else if (dma_mem == 1) {
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
    
    cv::Mat image,frame;
     //= cv::Mat(1080 + 1080 / 2, 1920, CV_8UC1, &output[0]);
    std::chrono::time_point<std::chrono::high_resolution_clock> tic, toc;
    struct ProfileApp profile_time = {};
    uint32_t visualize_num = 100;
    uint32_t count_frames = 0;
    uint32_t one_sec = 1000;
    uint32_t fps = 0;
    auto ts_loop = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    uint8_t *image_data;
    struct g2d_surfaceEx src, dst;
    struct g2d_buf *d_buf;

    void *g2d_handle = NULL;
    if (g2d_open(&g2d_handle)) {
	printf("g2d_open fail.\n");
	return -ENOTTY;
    }
    int w = 1920;
    int h = 1080;
    d_buf = g2d_alloc(w * h * 4, 2);

    while (true) {
	fps++;
        if ((std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() - ts_loop) > one_sec) {
            ts_loop = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	    std::cout << "fps" << fps << std::endl;
            fps = 0;
        }
	tic = std::chrono::high_resolution_clock::now();

	ret = buffers->DequeueBuffers(&image_data);
	memset(&src, 0, sizeof(src));
	memset(&dst, 0, sizeof(dst));

	src.base.left = 0;
	src.base.top = 0;
	src.base.right = w;
	src.base.bottom = h;
	src.base.width = 1920;
	src.base.height = 1080;
	src.base.stride = 1920;
	src.base.format = G2D_NV12;
	src.base.planes[0] = buffers->buf_addr.phys;
	src.base.planes[1] = buffers->buf_addr.phys + 1920 * 1080;
	src.tiling = G2D_AMPHION_TILED;

	dst.base.left = 0;
	dst.base.top = 0;
	dst.base.right = w;
	dst.base.bottom = h;
	dst.base.width = w;
	dst.base.height = h;
	dst.base.stride = w;
	dst.base.planes[0] = d_buf->buf_paddr;
	dst.base.format = G2D_BGRX8888;
	dst.tiling = G2D_LINEAR;

	g2d_blitEx(g2d_handle, &src, &dst);
	g2d_finish(g2d_handle);
	ret |= buffers->QueueBuffers();
        toc = std::chrono::high_resolution_clock::now();
	if (!ret) {
		std::cerr << "Capturing Failed" << std::endl;
		return 1;
	}
	image = cv::Mat(h, w, CV_8UC4, d_buf->buf_vaddr);

	//frame = cv::Mat(1080 + 1080 / 2, 1920, CV_8UC1, image_data);
	
	//cv::cvtColor(frame, image, cv::COLOR_YUV2BGR_NV12);

	profile_time.copy_buffer += std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count();

	tic = std::chrono::high_resolution_clock::now();
	cv::imshow("VIDEO", image);
	int key = cv::pollKey();

	toc = std::chrono::high_resolution_clock::now();
	profile_time.show +=  std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count();
	
	if (key == 27 || key == 'q' || key == 'Q') { // ESC key
            std::cout << "Exiting..." << std::endl;
            break;
        } else if (key == 's' || key == 'S') {
	  // Save the image to a file
	  std::cout << "Saving image " << std::endl;
	  std::string filename = "saved_image.jpg"; // Change the filename and format as needed
	  if (cv::imwrite(filename, image)) {
		std::cout << "Image saved to " << filename << std::endl;
	  } else {
		std::cerr << "Failed to save image!" << std::endl;
	  }
	}

        count_frames += 1;
	if (cam_conf.profile && count_frames == visualize_num) {
		print_profile_time(&profile_time, visualize_num);
		count_frames = 0;
	};
    }

    cv::destroyAllWindows();
    std::cout << "Stop streaming..." << std::endl;
    buffers->StopStream();
    std::cout << "Release buffers..." << std::endl;
    buffers->ReleaseBuffers();
    ::close(fd);
    free(user_buf);
    //free(temp_buffer);

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
/*
#ifdef USE_OPENCV
void DrawFps(cv::Mat &mat, uint32_t displayFps)
{
    std::string fpsText =  std::to_string(displayFps) + " fps";
    int fpsFont = cv::FONT_HERSHEY_SIMPLEX;
    double fpsScale = 0.7;
    int fpsThickness = 1;
    cv::Point fpsPoint(10, 30);
    uint32_t grayBackground;
    uint32_t grayText;


    if (mat.depth() == CV_8U) {
        grayBackground = 0x20;
        grayText = 0xd0;
    } else if (mat.depth() == CV_16U) {
        grayBackground = 0x2000;
        grayText = 0xd000;
    } else {
        throw std::invalid_argument("Unsupported bit depth");
    }
    int fpsBaseline;
    cv::Size fpsSize = cv::getTextSize(fpsText, fpsFont, fpsScale, fpsThickness, &fpsBaseline);

    cv::rectangle(
        mat,
        fpsPoint + cv::Point(0, fpsBaseline),
        fpsPoint + cv::Point(fpsSize.width, -fpsSize.height - 5),
        cv::Scalar(grayBackground, grayBackground, grayBackground),
        cv::FILLED
    );

    cv::putText(
        mat, 
        fpsText,
        fpsPoint,
        fpsFont,
        fpsScale,
        cv::Scalar(grayText, grayText, grayText),
        fpsThickness
    );
}
#endif
*/