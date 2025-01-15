#define IMX_G2D 1
#define USE_OPENCV 1
#include "CLI11/CLI11.hpp"
#include "controls.hpp"

#include <atomic> 
#include <ctime>
#include <iostream>
#include <unistd.h>
#include <string>
#include <algorithm>
#include <vector>
#include <chrono>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <dirent.h>
#include <fnmatch.h>
#include <linux/videodev2.h>
#include <cstring>

#include <cstdint>
#include <cstring>
#include <linux/dma-heap.h>

#ifdef IMX_G2D
	#include <imx/linux/dma-buf.h>
	#include <g2d.h>
	#include <g2dExt.h>
#endif
#ifdef USE_OPENCV
	#include <opencv2/highgui.hpp>
	#include <opencv2/imgproc/imgproc.hpp>
	#include <opencv2/imgcodecs/imgcodecs.hpp>
	#include <opencv2/core/ocl.hpp>
#endif
#define BUF_COUNT 3

std::atomic<bool> streamActive(true);

bool xioctl(int fd, unsigned long request, void* arg);
#ifdef USE_OPENCV
	void DrawFps(cv::Mat &mat, uint32_t displayFps);
#endif

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
    std::string pixel_format;
    BitWidth bit_width = TwelveBits;
    bool profile = false;
};

struct Buffer {
    uint8_t *rawData = nullptr;
    uint32_t rawLength = 0;
    uint32_t index = 0;
};

struct DMABuffer {
    int dma_fd;
    size_t size;
};

static int ParseArguments(int argc, char **argv, CameraConfiguration *cam_conf, bool* profile, int* dma_mem) {
    CLI::App app{"Display Image Example CLI"};

    // app.add_option("-d,--device", camera, "Device to stream from")->required();
    cam_conf->pixel_format = "NV12";
    app.add_option("--resize", cam_conf->resize, "Resize image to X Y dimension. default is no resizing");
    app.add_option("-p,--pixel-format", cam_conf->pixel_format, "add this flag -p NV12 to choose pixel format")
	->check(CLI::IsMember({"NV12", "NV16", "RG10", "RG12", "RG8", "YUYV", "GRAY"}));
    app.add_option("-r,--resolution", cam_conf->resolution, "Image resolution, default is -r 1920 1080");
    app.add_option("-b,--bit-width", cam_conf->bit_width, "Bit width 8, 10, 12, default -b 12")
    	->check(CLI::IsMember({8, 10, 12}));
    app.add_option("-v,--verbose", *profile, "add this flag -v 1 to output profiling information");
    app.add_option("-m,--dma-mem", *dma_mem, "add this flag -m for choosing memory type 0 - mmap, 1 - cma, 2 dma-gpu")
    	->check(CLI::IsMember({0, 1, 2}));
    for (auto &control : controls) {
        app.add_option(control.second.cliName, control.second.value, control.second.helpExplanation);
    }

    try {
    CLI11_PARSE(app, argc, argv);
    } catch (const CLI::ParseError &e) {
        int exitCode = app.exit(e);
        if (exitCode < 0){
            std::cerr << "Error encountered during CLI parsing. Exiting...\n";
        }
        return exitCode; 
    }
    return 1;
}

// other fields are set in driver
static int SetFormat(int fd, CameraConfiguration cam_conf, v4l2_format *format) {
    format->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format->fmt.pix.pixelformat = abbrev_to_pixel_fmt.at(cam_conf.pixel_format);
    std::cout << "pixel format: " << cam_conf.pixel_format << " V4l2 PIX FMT " << format->fmt.pix.pixelformat << std::endl;

    format->fmt.pix.width = cam_conf.resolution.at(0);
    format->fmt.pix.height = cam_conf.resolution.at(1);

    std::cout << " VIDIOC_S_FMT " << format->fmt.pix.width << " x " << format->fmt.pix.height << std::endl;
    if (xioctl(fd, VIDIOC_S_FMT, format) < 0) {
        std::cerr << "Format set error " << std::endl;
        return -1;
    }
    return 0;
}

static int RequestBuffers(int fd, struct v4l2_requestbuffers *req, int dma_mem) {
    req->count = BUF_COUNT;
    req->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (dma_mem == 1)
    	req->memory = V4L2_MEMORY_DMABUF;
    else
        req->memory = V4L2_MEMORY_MMAP;
 
    bool success = xioctl(fd, VIDIOC_REQBUFS, req);
    if (!success) {
        std::cerr << "Requesting Buffer failed" << std::endl;
        return 1;
    }
    if (req->count < 1) {
        std::cerr << "Insufficient buffer memory" << std::endl;
        return 1;
    }
    return 0;
}

static int QueueBuffers(int fd, int buf_size, int dma_mem, int dma_fd) {
    bool success;
    if (dma_mem == 1) {
		struct v4l2_buffer buf {};
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_DMABUF;
		buf.index = 0;
		buf.m.fd = dma_fd;
		success = xioctl(fd, VIDIOC_QBUF, &buf);
		if (!success) {
			std::cout << "VIDIOC_QBUF failed" << strerror(errno) << std::endl;
			return 1;
		}

    } else {
	for (uint32_t i = 0; i < buf_size; i++){
		struct v4l2_buffer buf {};
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;
		success = xioctl(fd, VIDIOC_QBUF, &buf);
		if (!success) {
			std::cout << "VIDIOC_QBUF failed" << strerror(errno) << std::endl;
			return 1;
		}
	}
    }
    return 0;
}

static int DequeueBuffers(int fd, v4l2_buffer *buf, int dma_mem) {
    bool success;
    buf->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (dma_mem == 1) {
    	buf->memory = V4L2_MEMORY_DMABUF;
    }
    else 
    	buf->memory = V4L2_MEMORY_MMAP;

    success = xioctl(fd, VIDIOC_DQBUF, buf);
    if (!success) {
        std::cout << "VIDIOC_DQBUF failed" << std::endl;
        return 1;
    }
    return 0;
}
// Function to allocate DMA-HEAP buffers
int allocate_dma_heap_buffer(size_t size) {
    // Open the DMA-HEAP system allocator
    int heap_fd = open("/dev/dma_heap/linux,cma", O_RDWR);
    if (heap_fd < 0) {
        perror("Failed to open DMA heap");
        return -1;
    }

    struct dma_heap_allocation_data alloc_data = {0};
    alloc_data.len = size;
    alloc_data.fd_flags = O_CLOEXEC | O_RDWR;

    // Allocate the buffer
    if (ioctl(heap_fd, DMA_HEAP_IOCTL_ALLOC, &alloc_data) < 0) {
        perror("Failed to allocate DMA buffer");
        close(heap_fd);
        return -1;
    }

    close(heap_fd);
    return alloc_data.fd;  // Return DMA-BUF FD
}

void *dma_buf_map(int dma_fd, size_t size) {
    void *mapped_addr = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, dma_fd, 0);
    if (mapped_addr == MAP_FAILED) {
        perror("Failed to mmap DMA buffer");
        return NULL;
    }
    return mapped_addr;
}

void copy_dma_to_user(int dma_fd, size_t size, void** user_buf) {
    // Map DMA buffer
    void *dma_ptr = dma_buf_map(dma_fd, size);
    if (!dma_ptr) {
        return;
    }

    // Copy data from DMA buffer to user buffer
    memcpy(*user_buf, dma_ptr, size);

    // Example: Save to a file (optional)
    /*
    FILE *file = fopen("frame.raw", "wb");
    if (file) {
        fwrite(user_buf, 1, size, file);
        fclose(file);
        printf("Frame data saved to frame.raw\n");
    }
    */

    // Cleanup
    munmap(dma_ptr, size);

}

static void print_profile_time(struct ProfileApp *profile_time, int visualize_num) {
    // only resize, convert, deque and show are relevent
    std::cout << "Deque time: " << profile_time->deque / visualize_num << " ms" << std::endl;
    std::cout << "Copy buffer time: " << profile_time->copy_buffer / visualize_num << " ms" << std::endl;
    std::cout << "Copy buffer time: " << profile_time->copy_buffer2 / visualize_num << " ms" << std::endl;
    std::cout << "Resize image time: " << profile_time->resize / visualize_num << " ms" << std::endl;
    std::cout << "Convert image time: " << profile_time->visualize / visualize_num << " ms" << std::endl;
    std::cout << "Draw fps time: " << profile_time->draw_fps / visualize_num << " ms" << std::endl;
    std::cout << "Show image time: " << profile_time->show / visualize_num << " ms" << std::endl;
    std::cout << "Queue buffer time: " << profile_time->queue / visualize_num << " ms " << std::endl;

    profile_time->deque = 0;
    profile_time->copy_buffer = 0;
    profile_time->copy_buffer2 = 0;
    profile_time->resize = 0;
    profile_time->visualize = 0;
    profile_time->draw_fps = 0;
    profile_time->show = 0;
    profile_time->queue = 0;
}

// find ids and values of v4lo2 controls of the subdevice
static void queryControls(int subd) {
    struct v4l2_queryctrl queryctrl;
    for (queryctrl.id = V4L2_CID_BASE; queryctrl.id < V4L2_CID_USER_IMX_BASE + 16; queryctrl.id++) {
    	if (xioctl(subd, VIDIOC_QUERYCTRL, &queryctrl) != 0) {
		if (controls.find(std::string((char*)queryctrl.name)) != controls.end()) {
            		controls.at(std::string((char*)queryctrl.name)).name = std::string((char*)queryctrl.name);
            		controls.at(std::string((char*)queryctrl.name)).id = queryctrl.id;
            		controls.at(std::string((char*)queryctrl.name)).defaultValue = queryctrl.default_value;
		}
	    std::cout << "Control: " << queryctrl.name << std::endl;
            std::cout << "  Minimum: " << queryctrl.minimum << std::endl;
            std::cout << "  Maximum: " << queryctrl.maximum << std::endl;
            std::cout << "  Default: " << queryctrl.default_value << std::endl;

    	} 
   }
}

// update v4l2-control values passed by command line
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

int main(int argc, char **argv) {
    std::vector<Buffer> buffers;
    int saveIm = 0;
    bool profile = false;
    int dma_mem = 0;
    CameraConfiguration cam_conf;
    int dma_fd;

    int ret = ParseArguments(argc, argv, &cam_conf, &profile, &dma_mem);
    if (ret < 1) {
        std::cerr << "Return ret " << ret << std::endl;
        return ret;
    }
    std::cout << "Using buffer type " << dma_mem << std::endl;
    std::cout << "PIXXEL FORMAT  " << cam_conf.pixel_format << std::endl;

#ifdef IMX_G2D
    if (dma_mem == 2)
	std::cout << "Using gpu dma buffer " << std::endl;

    struct dma_buf_phys buf_addrs[BUF_COUNT];
    memset(&buf_addrs, 0, sizeof(buf_addrs));
#endif

    // Open video device
    int fd = ::open(cam_conf.camera_id.c_str(), O_RDWR);
    if (fd == -1) {
        std::cerr << "Open failed" << std::endl;
        return 1;
    }

    // Open subdevice for driver controls 

    int subd = open(cam_conf.subdevice_id.c_str(), O_RDWR);
    if (subd < 0) {
         std::cout <<  "Failed to open subdev device" << std::endl;
        return -1;
    }

    // update controls given by command line

    queryControls(subd);
    ret = updateControls(subd);
    if (ret < 0) {
	std::cerr << "Updating controls failed" << std::endl;
        return ret;
    }

    struct v4l2_format format;
    ret = SetFormat(fd, cam_conf, &format);
    if (ret < 0) {
        std::cerr << "Set format failed" << std::endl;
        return ret;
    }

    v4l2_pix_format pix_fmt = format.fmt.pix;
    size_t buffer_size = pix_fmt.sizeimage;

    void *user_buf = malloc(pix_fmt.sizeimage);
    if (!user_buf) {
	std::cerr << "Failed to allocate user buffer" << std::endl;
        return 1;
    }
    void* temp_buffer =   malloc(1920 * 1080 * 4);


    struct v4l2_requestbuffers req {};
    std::cout << "Requesting Buffers " << std::endl;

    ret = RequestBuffers(fd, &req, dma_mem);
    if (ret < 0) {
        std::cerr << "Requesting Buffers failed" << std::endl;
        return ret;
    }


    // Init MMAP
    if (dma_mem == 2) {
	for (uint32_t i = 0; i < req.count; i++) {
		struct v4l2_buffer buf {};
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP; // V4L2_MEMORY_DMABUF 
		buf.index = i;
		bool success = xioctl(fd, VIDIOC_QUERYBUF, &buf);
		if(!success) {
		std::cout << "VIDIOC_QUERYBUF failed" << std::endl;
		return 1;
		}
		void *buffer_start = ::mmap (NULL, 
		pix_fmt.sizeimage, 
		PROT_READ | PROT_WRITE, 
		MAP_SHARED, 
		fd, 
		buf.m.offset);
		if (buffer_start == nullptr) {
		std::cout << "mmap failed" << std::endl;
		return 1;
		}
		std::memset(buffer_start, 0, pix_fmt.sizeimage);
		Buffer buffer;
		buffer.index = buf.index;
		buffer.rawLength = pix_fmt.sizeimage;
		buffer.rawData = (uint8_t *)buffer_start;
		buffers.push_back(buffer);
	
	#ifdef IMX_G2D
		struct v4l2_exportbuffer expbuf;
		memset(&expbuf, 0, sizeof(expbuf));
		expbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		expbuf.index = i;
		if (ioctl(fd, VIDIOC_EXPBUF, &expbuf) < 0) {
			std::cerr << "VIDIOC_EXPBUF" << strerror(errno) <<  std::endl;
			return -1;
		}
		int v4l2_dma_fd = expbuf.fd;
		std::cout << " EXPBUFFER PLANE \n" << expbuf.plane << std::endl;
		std::cout << " EXPBUFFER FLAGS \n" << expbuf.flags << std::endl;
		std::cout << " EXPBUFFER FD \n" << expbuf.fd << std::endl;
	        if (ioctl(expbuf.fd, DMA_BUF_IOCTL_PHYS, &buf_addrs[i]) < 0) {
			std::cerr << " EXPBUFFER DMA_BUF_IOCTL_PHYS error \n" << expbuf.fd << std::endl;
            		return -1;
		}

	#endif
        }
	ret = QueueBuffers(fd, buffers.size(), dma_mem, 0);
    	if (ret < 0) {
        	std::cerr << "Queueing Buffers failed" << std::endl;
        	return ret;
    	}
    }
    else if (dma_mem == 1) { 
		std::cout << "Allocating and Queing buffers " << std::endl;
		std::vector<DMABuffer> buffers(BUF_COUNT);
		for (int i = 0; i < BUF_COUNT; i++) {
			buffers[i].dma_fd = allocate_dma_heap_buffer(buffer_size);
			if (dma_fd < 0) {
				std::cerr << "Failed to allocate DMA buffer " << i << "\n";
				close(fd);
				return EXIT_FAILURE;
			}

			std::cout << "Queing buffers " << std::endl;
			struct v4l2_buffer buf {0};
			buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			buf.index = i;
			buf.memory = V4L2_MEMORY_DMABUF;
			buf.m.fd = buffers[i].dma_fd;

			if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
				perror("Failed to queue DMA buffer");
				close(dma_fd);
				close(fd);
				return EXIT_FAILURE;
			}
		}
		
		/*
		std::cout << "Queing buffers " << std::endl;
		ret = QueueBuffers(fd, 1, dma_mem, dma_fd);
    		if (ret < 0) {
        		std::cerr << "Queueing Buffers failed" << strerror(errno) <<  std::endl;
        		return ret;
		*/
    } else if (dma_mem == 0) {
	for (uint32_t i = 0; i < req.count; i++) {
		struct v4l2_buffer buf {};
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP; // V4L2_MEMORY_DMABUF 
		buf.index = i;
		bool success = xioctl(fd, VIDIOC_QUERYBUF, &buf);
		if(!success) {
		std::cout << "VIDIOC_QUERYBUF failed" << std::endl;
		return 1;
		}
		void *buffer_start = ::mmap (NULL, 
		pix_fmt.sizeimage, 
		PROT_READ | PROT_WRITE, 
		MAP_SHARED, 
		fd, 
		buf.m.offset);
		if (buffer_start == nullptr) {
		std::cout << "mmap failed" << std::endl;
		return 1;
		}
		std::memset(buffer_start, 0, pix_fmt.sizeimage);
		Buffer buffer;
		buffer.index = buf.index;
		buffer.rawLength = pix_fmt.sizeimage;
		buffer.rawData = (uint8_t *)buffer_start;
		buffers.push_back(buffer);
	}
	ret = QueueBuffers(fd, buffers.size(), dma_mem, 0);
    	if (ret < 0) {
        	std::cerr << "Queueing Buffers failed" << std::endl;
        	return ret;
    	}
	}





    std::cout << "Start streaming" << std::endl;
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bool success = xioctl(fd, VIDIOC_STREAMON, &type);
    if (!success) {
        std::cerr << "VIDIOC_STREAMON failed" << std::endl;
        return 1;
    }

    std::cout << "profiling = " << profile << std::endl;

    struct ProfileApp profile_time = {};
    std::chrono::time_point<std::chrono::high_resolution_clock> tic, toc;
    const uint32_t visualize_num = 100, one_sec = 1000;
    auto ts_loop = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    #ifdef USE_OPENCV
    cv::Mat image, image8U, res_image;
    #endif
    uint32_t fps = 0, count = 0, displayFps = 0;
    uint8_t *output = new uint8_t[pix_fmt.sizeimage];

    void *g2d_handle = NULL;
    if (g2d_open(&g2d_handle)) {
	printf("g2d_open fail.\n");
	return -ENOTTY;
    }
    struct g2d_surfaceEx src, dst;
    struct g2d_buf *d_buf;
    int w = 1280;
    int h = 720;
    d_buf = g2d_alloc(w * h * 4, 2);

    while (streamActive) {
        struct v4l2_buffer buf {};
        count += 1;

	// Ideally the deque operation should be the slowest one, then the app operates at max speed
        tic = std::chrono::high_resolution_clock::now();
	//std::cout << "Dequeying buffers " << std::endl;
        ret = DequeueBuffers(fd, &buf, dma_mem);
        if (ret < 0) {
            std::cout << "Dequeueing Buffers failed" << std::endl;
            return ret;
        }
        toc = std::chrono::high_resolution_clock::now();
        profile_time.deque += std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count();

        // Increase FPS counter
        fps++;
        if ((std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() - ts_loop) > one_sec) {
            ts_loop = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	    std::cout << "fps" << fps << std::endl;
            displayFps = fps;
            fps = 0;
        }

	// copy buffer is fast operation
	/*
        tic = std::chrono::high_resolution_clock::now();
  	::memcpy(&output[0], &buffers[buf.index].rawData[0], pix_fmt.sizeimage);

        if (cam_conf.bit_width == EightBits) {
            image = cv::Mat(pix_fmt.height, pix_fmt.width, CV_8UC1, &buffers[buf.index].rawData[0]);
        }
        else {
            image = cv::Mat(pix_fmt.height, pix_fmt.width, CV_16UC1, &buffers[buf.index].rawData[0]);
        }

        toc = std::chrono::high_resolution_clock::now();
        profile_time.copy_buffer += std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count();

	*/
	// Copy data from ISP to userspace and measure transfer time
	tic = std::chrono::high_resolution_clock::now();
	
	// ::memcpy(&output[0], &buffers[buf.index].rawData[0], pix_fmt.sizeimage);
	if (dma_mem == 2) {
		#ifdef IMX_G2D
		/*
		void *g2d_handle = NULL;
		if (g2d_open(&g2d_handle)) {
    			printf("g2d_open fail.\n");
    			return -ENOTTY;
		}
		*/
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
		src.base.planes[0] = buf_addrs[buf.index].phys;
		src.base.planes[1] = buf_addrs[buf.index].phys + 1920 * 1080;
		src.tiling = G2D_AMPHION_TILED;

		dst.base.left = 0;
		dst.base.top = 0;
		dst.base.right = w;
		dst.base.bottom = h;
		dst.base.width = w;
		dst.base.height = h;
		dst.base.stride = w;
		dst.base.planes[0] = d_buf->buf_paddr; //d_buf->buf_paddr;
		//st.base.planes[1] = buf_addrs[buf.index].phys+ 1920 * 1080;
		dst.base.format = G2D_BGRX8888;
		dst.tiling = G2D_LINEAR;

		g2d_blitEx(g2d_handle, &src, &dst);
		g2d_finish(g2d_handle);

		
		
		//memcpy(temp_buffer, d_buf->buf_vaddr, w * h * 4);

		//res_image = cv::Mat(h, w, CV_8UC4, temp_buffer);
		res_image = cv::Mat(h, w, CV_8UC4, d_buf->buf_vaddr);
		

		#endif
	} else if (dma_mem == 1) {
		copy_dma_to_user(buf.m.fd, buffer_size, &user_buf);
		#ifdef USE_OPENCV
			image = cv::Mat(pix_fmt.height + pix_fmt.height / 2, pix_fmt.width, CV_8UC1, user_buf);
			cv::cvtColor(image, res_image, cv::COLOR_YUV2BGR_NV12);
		#endif

    	}
	else if (dma_mem == 0) {
		//::memcpy(&output[0], &buffers[buf.index].rawData[0], pix_fmt.sizeimage);
		image = cv::Mat(pix_fmt.height + pix_fmt.height / 2, pix_fmt.width, CV_8UC1, &buffers[buf.index].rawData[0]);
		cv::cvtColor(image, res_image, cv::COLOR_YUV2BGR_NV12);
		/* for mono image
		// this is bottleneck operation, it is faster to resize first and then convert to CV_8UC1
	        if (cam_conf.bit_width == EightBits) {
        		image = cv::Mat(pix_fmt.height, pix_fmt.width, CV_8UC1, &buffers[buf.index].rawData[0]);
        	} else {
            		image = cv::Mat(pix_fmt.height, pix_fmt.width, CV_16UC1, &buffers[buf.index].rawData[0]);
		}
		*/
        }
	toc = std::chrono::high_resolution_clock::now();
	profile_time.copy_buffer += std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count();
        /*
	tic = std::chrono::high_resolution_clock::now();
	// Usage example
	// simd_memcpy(output2, &buffers[buf.index].rawData[0], pix_fmt.sizeimage);
	
        if (cam_conf.resize.at(0) != 0 && cam_conf.resize.at(1) != 0) {
                cv::resize(image, res_image, cv::Size(cam_conf.resize.at(0), cam_conf.resize.at(1)), 0, 0, cv::INTER_NEAREST);
        }
        else {
                res_image = image;
        }

        toc = std::chrono::high_resolution_clock::now();
        profile_time.resize += std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count();
	*/

	// opencv does not support visualizing CV_16UC1 images, so one must convert it here to add proper shift for RAW10 and RAW12
	// otherwise, opencv will make shift for RAW16
	/*
        tic = std::chrono::high_resolution_clock::now();

        if (cam_conf.bit_width != EightBits){
                res_image.convertTo(image8U, CV_8UC1, 1. / conversion_factor.at(cam_conf.bit_width));
        } else {
		image8U = res_image;
	}

        toc = std::chrono::high_resolution_clock::now();
        profile_time.visualize += std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count();
	*/
	/*
        if (image8U.empty()) {
                std::cerr << "The image is empty." << std::endl;
        }
	*/
	// Print FPS on image

        tic = std::chrono::high_resolution_clock::now();
	#ifdef USE_OPENCV
        DrawFps(res_image, displayFps);
	#endif
        toc = std::chrono::high_resolution_clock::now();
        profile_time.draw_fps +=  std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count();

	// show image using opencv/gtk
        tic = std::chrono::high_resolution_clock::now();
        #ifdef USE_OPENCV
	cv::imshow(cam_conf.camera_id, res_image);
	//cv::imshow(cam_conf.camera_id, frame_gpu);
        int key = cv::pollKey();
	#endif
        toc = std::chrono::high_resolution_clock::now();
        profile_time.show +=  std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count();

        // Queue buffer - fast operation
        tic = std::chrono::high_resolution_clock::now();
        success = xioctl(fd, VIDIOC_QBUF, &buf);
        if (!success) {
            std::cout << "VIDIOC_QBUF failed" << std::endl;
        }
        toc = std::chrono::high_resolution_clock::now();
        profile_time.queue += std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count();

	// print profiling info
	if (profile && count == visualize_num)
		print_profile_time(&profile_time, visualize_num);

	if (count == visualize_num) {
		count = 0;
	}
	// you can adapt this if you want to save images //cmat
        if (saveIm) {
            std::string filename = "image.png";
	    //std::vector<int> params = {cv::IMWRITE_PNG_COMPRESSION, 0};
	    //cv::imwrite(filename, image, params);
            saveIm = 0;
        }
    }
    g2d_close(g2d_handle);
    
    // Close buffers
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    success = xioctl(fd, VIDIOC_STREAMOFF, &type);
    if (!success) {
        std::cout << "VIDIOC_STREAMOFF failed" << std::endl;
    }
    if (dma_mem == 0) {
    for (uint32_t i = 0; i < BUF_COUNT; i++) {
        ::munmap(buffers[i].rawData, buffers[i].rawLength);
    }
    buffers.clear();
    }
    ::close(subd);
    ::close(fd);
    free(user_buf);
    free(temp_buffer);

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