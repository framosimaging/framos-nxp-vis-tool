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
#include <opencv2/core/ocl.hpp>

#include <imx/linux/dma-buf.h>
#include <g2d.h>
#include <g2dExt.h>

// TODOCL
#include <CL/opencl.hpp>
#include <fstream>

// Function to load OpenCL kernel
cl::Program load_kernel(cl::Context context, std::string kernel_file) {
    std::ifstream file(kernel_file);
    if (!file) {
        std::cerr << "Error: Cannot open kernel file: " << kernel_file << std::endl;
        exit(1);
    }
    std::string kernel_code((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    cl::Program::Sources source;
    source.push_back({kernel_code.c_str(), kernel_code.length()});
    return cl::Program(context, source);
}

using json = nlohmann::json;

bool xioctl(int fd, unsigned long request, void* arg);
void DrawFps(cv::Mat &mat, uint32_t displayFps);

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
    {"RG10", V4L2_PIX_FMT_SRGGB10}, // not supported
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

struct ProfileApp {
    long long deque;
    long long show;
    long long draw_fps;
    long long queue;
    long long visualize;
    long long copy_buffer;
    long long debayer;
    long long resize;
    long long endian_conv;
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

void convertBigToLittleEndian(cv::Mat& image) {
    if (image.type() != CV_16U) {
        std::cerr << "Error: Image is not 16-bit (CV_16U)!" << std::endl;
        return;
    }

    int totalPixels = image.rows * image.cols;
    uint16_t* data = reinterpret_cast<uint16_t*>(image.data);

    for (int i = 0; i < totalPixels; i++) {
        data[i] = (data[i] >> 8) | (data[i] << 8);  // Swap bytes for each pixel
    }
}

const char *oclKernelSource = R"(
	__kernel void process_raw12(
	    __global ushort *output, 
	    const int width, 
	    const int height) 
	{
	    int i = get_global_id(0);
	
	    if (i < width * height) {
		// Convert Big-Endian to Little-Endian
		//ushort pixel = (output[i] >> 8) | (output[i] << 8);
	        ushort pixel = (output[i] >> 8) | (output[i] << 8);
		output[i] = pixel;
	    }
	}
	)";

	
void processRaw12WithOpenCL(cv::UMat &u_raw12, int width, int height) {
	if (!cv::ocl::haveOpenCL()) {
	  std::cerr << "OpenCL is not available!" << std::endl;
	  return;
	}
   
	// Convert raw12 to OpenCL UMat
	cv::ocl::Context context;

	if (!context.create(cv::ocl::Device::TYPE_GPU)) {
		std::cerr << "Failed to create OpenCL context!" << std::endl;
		return;
	}
	//cv::ocl::Device(context.device(0)).set(cv::ocl::Device::TYPE_GPU);
	cv::ocl::Device device = context.device(0);

	cv::String errMsg;
	cv::ocl::Kernel kernel("process_raw12", cv::ocl::ProgramSource(oclKernelSource), errMsg);

	
        //cv::ocl::ProgramSource source(oclKernelSource);
        //cv::ocl::Kernel kernel("process_raw12", source, errMsg);

	if (kernel.empty()) {
	     std::cerr << "Failed to compile OpenCL kernel: " << errMsg << std::endl;
	return;
	}
	//u_raw12.getUMat(cv::ACCESS_RW, cv::USAGE_ALLOCATE_DEVICE_MEMORY);

	kernel.args(cv::ocl::KernelArg::PtrReadWrite(u_raw12), width, height);
	//kernel.args(cv::ocl::KernelArg::ReadWriteNoSize(u_raw12), (int)width, (int)height);
	
	size_t globalSize = width * height;
	bool success = kernel.run(1, &globalSize, nullptr, true);
	if (!success) {
		std::cerr << "OpenCL kernel execution failed!" << std::endl;
	    }
}

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

static void print_profile_time(struct ProfileApp *profile_time, int visualize_num) {
    // only resize, convert, deque and show are relevent
    //std::cout << "Deque time: " << profile_time->deque / visualize_num << " ms" << std::endl;
    std::cout << "Copy buffer time: " << profile_time->copy_buffer / visualize_num << " ms" << std::endl;
    std::cout << "Resize image time: " << profile_time->resize / visualize_num << " ms" << std::endl;
    std::cout << "Convert image time: " << profile_time->visualize / visualize_num << " ms" << std::endl;
    std::cout << "Debayer time: " << profile_time->debayer / visualize_num << " ms" << std::endl;
    //std::cout << "Draw fps time: " << profile_time->draw_fps / visualize_num << " ms" << std::endl;
    std::cout << "Show image time: " << profile_time->show / visualize_num << " ms" << std::endl;
    std::cout << "Endian conversion: " << profile_time->endian_conv / visualize_num << " ms" << std::endl;
    //std::cout << "Queue buffer time: " << profile_time->queue / visualize_num << " ms " << std::endl;

    profile_time->deque = 0;
    profile_time->copy_buffer = 0;
    profile_time->debayer = 0;
    profile_time->resize = 0;
    profile_time->visualize = 0;
    profile_time->draw_fps = 0;
    profile_time->show = 0;
    profile_time->queue = 0;
    profile_time->endian_conv = 0;
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
    
    /*
    cl::Platform platform = cl::Platform::getDefault();
    std::cout << "Using OpenCL Platform: " << platform.getInfo<CL_PLATFORM_NAME>() << std::endl;
    std::cout << "Platform Vendor: " << platform.getInfo<CL_PLATFORM_VENDOR>() << std::endl;
    std::cout << "Platform Version: " << platform.getInfo<CL_PLATFORM_VERSION>() << std::endl;

    cl::Device device = cl::Device::getDefault();
    std::cout << "Using OpenCL device : " << device.getInfo<CL_DEVICE_NAME>() << std::endl;
    cl::Context context(device);

    const char* kernel_code = R"(
        __kernel void add_vectors(__global const int* a, 
                          	__global const int* b, 
                        	__global int* result, 
			        const int size) {
	    int id = get_global_id(0);
	    if (id < size) {
	        result[id] = a[id] + b[id]; // Now correctly using integers
	    }
        }
     )";

    //cl::Program program = load_kernel(context, "ocl_kernel.cl");
    cl::Program::Sources source;
    source.push_back({kernel_code, strlen(kernel_code)});
    cl::Program program(context, source);
    program.build({device});

    const int size = 10;
    std::vector<int> host_a(size, 10);  // Vector A = [10, 10, 10, ...]
    std::vector<int> host_b(size, 5);   // Vector B = [5, 5, 5, ...]
    std::vector<int> host_result(size, 0);

    cl::Buffer buffer_a(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, sizeof(int) * size, host_a.data());
    cl::Buffer buffer_b(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, sizeof(int) * size, host_b.data());
    cl::Buffer buffer_result(context, CL_MEM_WRITE_ONLY, sizeof(int) * size);
    cl::Kernel kernel(program, "add_vectors");

    kernel.setArg(0, buffer_a);
    kernel.setArg(1, buffer_b);
    kernel.setArg(2, buffer_result);
    kernel.setArg(3, size);

    cl::CommandQueue queue(context, device);
    queue.enqueueNDRangeKernel(kernel, cl::NullRange, cl::NDRange(size));
    queue.finish();
    queue.enqueueReadBuffer(buffer_result, CL_TRUE, 0, sizeof(int) * size, host_result.data());

    std::cout << "Integer Vector Addition Results:" << std::endl;
    for (int i = 0; i < size; i++) {
        std::cout << host_a[i] << " + " << host_b[i] << " = " << host_result[i] << std::endl;
    }
    */
    cv::Mat image,frame;
     //= cv::Mat(1080 + 1080 / 2, 1920, CV_8UC1, &output[0]);
    std::chrono::time_point<std::chrono::high_resolution_clock> tic, toc;
    struct ProfileApp profile_time = {};
    uint32_t visualize_num = 30;
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
    //cv::UMat processed8U;
    //processed8U.create(h, w, CV_16UC1); 

    enum g2d_format g2_format;
    if (pix_fmt.pixelformat == V4L2_PIX_FMT_YUYV)
      g2_format = G2D_YUYV;
    else if (pix_fmt.pixelformat == V4L2_PIX_FMT_NV12)
      g2_format = G2D_NV12;
    else if (pix_fmt.pixelformat == V4L2_PIX_FMT_NV16)
      g2_format = G2D_NV16;
    else 
      g2_format = G2D_RGBX8888;

    // Step 1-2: Process RAW12 with OpenCL
    cv::ocl::setUseOpenCL(true);
    if (!cv::ocl::useOpenCL()) {
	std::cerr << "OpenCL is NOT enabled in OpenCV!" << std::endl;
    }
    cv::UMat colorImage;
    //colorImage.create(h, w, CV_16UC3);
    //cv::namedWindow("Fast OpenGL Display", cv::WINDOW_OPENGL);
    //cv::ogl::Texture2D oglTex;

    while (true) {
	fps++;
        if ((std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() - ts_loop) > one_sec) {
            ts_loop = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	    std::cout << "fps" << fps << std::endl;
            fps = 0;
        }
	tic = std::chrono::high_resolution_clock::now();

	ret = buffers->DequeueBuffers(&image_data);

	if (g2_format != G2D_RGBX8888) {

		memset(&src, 0, sizeof(src));
		memset(&dst, 0, sizeof(dst));

		src.base.left = 0;
		src.base.top = 0;
		src.base.right = w;
		src.base.bottom = h;
		src.base.width = 1920;
		src.base.height = 1080;
		src.base.stride = 1920;
		src.base.format = g2_format;
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

		if (!ret) {
			std::cerr << "Capturing Failed" << std::endl;
			return 1;
		}
		image = cv::Mat(h, w, CV_8UC4, d_buf->buf_vaddr);

		toc = std::chrono::high_resolution_clock::now();
		profile_time.copy_buffer += std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count();
		tic = std::chrono::high_resolution_clock::now();
		cv::imshow("VIDEO", image);

	} else {
	    if (false) { // without gpu 
		tic = std::chrono::high_resolution_clock::now();
		image = cv::Mat(h, w, CV_16U, image_data);
		toc = std::chrono::high_resolution_clock::now();
		profile_time.copy_buffer += std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count();

		tic = std::chrono::high_resolution_clock::now();
		convertBigToLittleEndian(image);
		toc = std::chrono::high_resolution_clock::now();
		profile_time.endian_conv += std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count();

		cv::Mat frame8;
		tic = std::chrono::high_resolution_clock::now();
		image.convertTo(frame8, CV_8UC1, 1.0 / 16.0);
		ret |= buffers->QueueBuffers();
		toc = std::chrono::high_resolution_clock::now();
		profile_time.resize += std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count();
		
		//cv::cvtColor(u_raw12, colorImage, cv::COLOR_BayerRG2BGR_EA);  // Uses OpenCL
		tic = std::chrono::high_resolution_clock::now();
		cv::cvtColor(frame8, frame8, cv::COLOR_BayerRG2BGR);
		toc = std::chrono::high_resolution_clock::now();
		profile_time.debayer += std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count();
		    
		tic = std::chrono::high_resolution_clock::now();
    		cv::imshow("VIDEO", frame8); 
	    } else {
		tic = std::chrono::high_resolution_clock::now();
		image = cv::Mat(h, w, CV_16U, image_data);
		cv::UMat u_raw12;
		image.copyTo(u_raw12);
		toc = std::chrono::high_resolution_clock::now();
		profile_time.copy_buffer += std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count();

		tic = std::chrono::high_resolution_clock::now();
		//cv::UMat processed8U(h, w, CV_8UC1, cv::USAGE_ALLOCATE_DEVICE_MEMORY);
		processRaw12WithOpenCL(u_raw12, w, h);
		//convertBigToLittleEndian(image);
		toc = std::chrono::high_resolution_clock::now();
		profile_time.endian_conv += std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count();

		//cv::cvtColor(frame8, frame8, cv::COLOR_BayerRG2BGR);

		//tic = std::chrono::high_resolution_clock::now();
		//cv::cvtColor(u_raw12, u_raw12, cv::COLOR_BayerRG2BGR);
		//toc = std::chrono::high_resolution_clock::now();
		//profile_time.debayer += std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count();

		// test bit conversion

		//image =cv::Mat(h, w, CV_16U, image_data);
		//cv::Mat frame8;
		tic = std::chrono::high_resolution_clock::now();

		//cv::Mat mat = u_raw12.getMat(cv::ACCESS_READ);
		//cv::Mat frame8;
		//mat.convertTo(frame8, CV_8UC1, 1.0 / 16.0);
		//image.convertTo(frame8, CV_8UC1, 1.0 / 16.0);
		cv::UMat u_raw8;
		u_raw12.convertTo(u_raw8, CV_8UC1, 1.0 / 16.0);
		ret |= buffers->QueueBuffers();

		toc = std::chrono::high_resolution_clock::now();
		profile_time.resize += std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count();

		tic = std::chrono::high_resolution_clock::now();
		//cv::Mat mat = u_raw12.getMat(cv::ACCESS_READ);

		cv::cvtColor(u_raw8, u_raw8, cv::COLOR_BayerRG2BGR);
		//cv::cvtColor(mat, colorImage, cv::COLOR_BayerRG2BGR);  // Uses OpenCL
		//cv::Mat mat = u_raw8.getMat(cv::ACCESS_READ);
		cv::Mat mat;
		u_raw8.copyTo(mat);
		toc = std::chrono::high_resolution_clock::now();
		profile_time.debayer += std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count();

		tic = std::chrono::high_resolution_clock::now();
		//cv::UMat colorImage8U;
		//colorImage.convertTo(colorImage8U, CV_8UC3, 1.0 / 16.0);
		//image.convertTo(frame8, CV_8UC1, 1.0 / 16.0);
		cv::imshow("VIDEO", u_raw8); 
		
	    } // end else true

	}
	//frame = cv::Mat(1080 + 1080 / 2, 1920, CV_8UC1, image_data);
	
	//cv::cvtColor(frame, image, cv::COLOR_YUV2BGR_NV12);

	// THIS SHOULD NOT BE COMMENTED
	// profile_time.copy_buffer += std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count();

	//tic = std::chrono::high_resolution_clock::now();
	//cv::imshow("VIDEO", image);
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