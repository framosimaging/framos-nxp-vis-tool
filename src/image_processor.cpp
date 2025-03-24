#include "image_processor.hpp"
#include <linux/videodev2.h>
#include <ctime>

bool ImageProcessor::SaveImage() {
    std::cout << "Saving image " << std::endl;
    char timestamp[20];

    // Get current time
    std::time_t now = std::time(nullptr);
    std::strftime(timestamp, sizeof(timestamp), "%Y-%m-%d_%H-%M-%S", std::localtime(&now));

    // Create filename with timestamp
    std::string filename = "image_" + std::string(timestamp) + ".jpg";

    if (cv::imwrite(filename, processed_image)) {
	std::cout << "Image saved to " << filename << std::endl;
	return true;
    } else {
	std::cerr << "Failed to save image!" << std::endl;
	return false;
    }
}

void ImageProcessor::PrintProfiling() {
	if (count_frames_ == visualize_num_) { 
		std::cout << "Copy buffer time: " << profile_info.copy_buffer / visualize_num_ << " ms" << std::endl;
		std::cout << "Resize image time: " << profile_info.resize / visualize_num_ << " ms" << std::endl;
		std::cout << "Convert image time: " << profile_info.visualize / visualize_num_ << " ms" << std::endl;
		std::cout << "Debayer time: " << profile_info.debayer / visualize_num_ << " ms" << std::endl;
		std::cout << "Show image time: " << profile_info.show / visualize_num_ << " ms" << std::endl;
		std::cout << "Endian conversion: " << profile_info.endian_conv / visualize_num_ << " ms" << std::endl;
		count_frames_ = 0;
		
		profile_info.copy_buffer = 0;
		profile_info.debayer = 0;
		profile_info.resize = 0;
		profile_info.visualize = 0;
		//profile_info->draw_fps = 0;
		profile_info.show = 0;
		profile_info.endian_conv = 0;

	}
	else {
		count_frames_ += 1;
	}

}

CPUImageProcessor::CPUImageProcessor(int width, int height) {
    width_ = width;
    height_ = height;
    std::cout << "CPU Image Processor created." << std::endl;
}

CPUImageProcessor::~CPUImageProcessor() {
	std::cout << "CPU Image Processor finished." << std::endl;
}

void CPUImageProcessor::convertBigToLittleEndian(cv::Mat& image) {
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

// Implementation of ProcessImage for CPU
void CPUImageProcessor::ProcessImage(uint8_t *image_data) {
	auto tic = std::chrono::high_resolution_clock::now();
	cv::Mat image = cv::Mat(height_, width_, CV_16U, image_data);
	auto toc = std::chrono::high_resolution_clock::now();
	profile_info.copy_buffer += std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count();

	tic = std::chrono::high_resolution_clock::now();
	convertBigToLittleEndian(image);
	toc = std::chrono::high_resolution_clock::now();
	profile_info.endian_conv += std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count();
	
	tic = std::chrono::high_resolution_clock::now();
	image.convertTo(processed_image, CV_8UC1, 1.0 / 16.0);
	toc = std::chrono::high_resolution_clock::now();
	profile_info.resize += std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count();

	tic = std::chrono::high_resolution_clock::now();
	cv::cvtColor(processed_image, processed_image, cv::COLOR_BayerRG2BGR);
	toc = std::chrono::high_resolution_clock::now();
	profile_info.debayer += std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count();

	tic = std::chrono::high_resolution_clock::now();
	cv::imshow("VIDEO", processed_image);
	toc = std::chrono::high_resolution_clock::now();
	profile_info.visualize += std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count();
}

// Constructor for G2DImageProcessor
G2DImageProcessor::G2DImageProcessor(int width, int height, uint32_t pixel_format) {
    width_ = width;
    height_ = height;
    if (pixel_format == V4L2_PIX_FMT_YUYV) {
	g2_format = G2D_YUYV;
    }
    else if (pixel_format == V4L2_PIX_FMT_NV12) {
	g2_format = G2D_NV12;
    }
    else if (pixel_format == V4L2_PIX_FMT_NV16) {
	g2_format = G2D_NV16;
    } else {
	std::cout << "Pixel format not recognized..." << std::endl;
    }

    if (g2d_open(&g2d_handle)) {
	printf("g2d_open fail.\n");
	//return -ENOTTY;
    }
    d_buf = g2d_alloc(width_ * height_ * 4, 2);
    
    memset(&src, 0, sizeof(src));

    src.base.left = 0;
    src.base.top = 0;
    src.base.right = width_;
    src.base.bottom = height_;
    src.base.width = width_;
    src.base.height = height_;
    src.base.stride = width_;
    src.base.format = g2_format;
    src.tiling = G2D_AMPHION_TILED;

    memset(&dst, 0, sizeof(dst));

    dst.base.left = 0;
    dst.base.top = 0;
    dst.base.right = width_;
    dst.base.bottom = height_;
    dst.base.width = width_;
    dst.base.height = height_;
    dst.base.stride = width_;
    dst.base.format = G2D_BGRX8888;
    dst.tiling = G2D_LINEAR;
    dst.base.planes[0] = d_buf->buf_paddr;

    std::cout << "G2D Image Processor created." << std::endl;
}

G2DImageProcessor::~G2DImageProcessor() {
	std::cout << "G2D Image Processor finished." << std::endl;
	g2d_free(d_buf);
}

// Implementation of ProcessImage for G2D
void G2DImageProcessor::ProcessImage(struct dma_buf_phys *buf_addr) {
    auto tic = std::chrono::high_resolution_clock::now();

    src.base.planes[0] = buf_addr->phys;
    src.base.planes[1] = buf_addr->phys + width_ * height_;

    g2d_blitEx(g2d_handle, &src, &dst);
    g2d_finish(g2d_handle);

    processed_image = cv::Mat(height_, width_, CV_8UC4, d_buf->buf_vaddr);

    auto toc = std::chrono::high_resolution_clock::now();
    profile_info.copy_buffer += std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count();

    tic = std::chrono::high_resolution_clock::now();
    cv::imshow("VIDEO", processed_image);
    toc = std::chrono::high_resolution_clock::now();
    profile_info.visualize += std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count();
}


const char *demosaicKernel = R"(
	__kernel void demosaic(__global const ushort* src, __global uchar4* dst, int width) {
	    int i = get_global_id(1) * 2;
	    int j = get_global_id(0) * 2;
	    
	    int idxR  = i * width + j;
	    int idxG1 = i * width + j + 1;
	    int idxG2 = (i + 1) * width + j;
	    int idxB  = (i + 1) * width + j + 1;
    
	    if (i + 1 >= get_global_size(1) * 2 || j + 1 >= get_global_size(0) * 2)
		return;
    
	    // Read 16-bit input values
	    ushort r = src[idxR];
	    ushort g1 = src[idxG1];
	    ushort g2 = src[idxG2];
	    ushort b = src[idxB];
    
	    // Convert 16-bit values to 8-bit (preserving high bits)
	    uchar R  = (uchar) ((r >> 12) | (r << 4));
	    uchar G1 = (uchar) ((g1 >> 12) | (g1 << 4));
	    uchar G2 = (uchar) ((g2 >> 12) | (g2 << 4));
	    uchar B  = (uchar) ((b >> 12) | (b << 4));
    
	    // Store output as uchar3 (B, G, R format)
	    uchar4 pixel1 = (uchar4)(B, G1, R, 255);
	    uchar4 pixel2 = (uchar4)(B, G2, R, 255);
    
	    // Write directly to BGR output buffer
	    dst[idxR]  = pixel1;  // Top-left (R pixel)
	    dst[idxG1] = pixel1;  // Top-right (G1 pixel)
	    dst[idxG2] = pixel2;  // Bottom-left (G2 pixel)
	    dst[idxB]  = pixel2;  // Bottom-right (B pixel)
	}
    )";

// Constructor for GPUImageProcessor
GPUImageProcessor::GPUImageProcessor(int width, int height) {
    width_ = width;
    height_ = height;
    std::cout << "GPU Image Processor created." << std::endl;

    device = cl::Device::getDefault();
    context = cl::Context(device);
    queue = cl::CommandQueue(context, device, CL_QUEUE_PROFILING_ENABLE);
    std::cout << "Using OpenCL device : " << device.getInfo<CL_DEVICE_NAME>() << std::endl;

    source = cl::Program::Sources();
    source.push_back({demosaicKernel, strlen(demosaicKernel)});
    program = cl::Program(context, source);
    program.build({device});
    kernel = cl::Kernel(program, "demosaic");
    const size_t numPixels = width * height;

    inputBuffer = cl::Buffer(context, CL_MEM_READ_ONLY, numPixels * 2);
    outputBuffer = cl::Buffer(context, CL_MEM_WRITE_ONLY, numPixels * 4);
    
}

// Constructor for GPUImageProcessor
GPUImageProcessor::~GPUImageProcessor() {
	std::cout << "GPU Image Processor created." << std::endl;
    }

// Implementation of ProcessImage for GPU
void GPUImageProcessor::ProcessImage(uint8_t *image_data) {
	cl::NDRange globalSize(width_ / 2, height_ / 2);

	auto tic = std::chrono::high_resolution_clock::now();
	queue.enqueueWriteBuffer(inputBuffer, CL_TRUE, 0, width_ * height_ * 2, reinterpret_cast<uint16_t *>(image_data));

	kernel.setArg(0, inputBuffer);
	kernel.setArg(1, outputBuffer);
	kernel.setArg(2, width_);
	auto toc = std::chrono::high_resolution_clock::now();
	profile_info.copy_buffer += std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count();

	tic = std::chrono::high_resolution_clock::now();
	//queue.enqueueNDRangeKernel(kernel, cl::NullRange, globalSize, localSize);
	//queue.finish();

	// Run Kernel Asynchronously
	//cl::Event kernel_event;
	//queue.enqueueNDRangeKernel(kernel, cl::NullRange, globalSize, cl::NullRange, nullptr, &kernel_event);

	queue.enqueueNDRangeKernel(kernel, cl::NullRange, globalSize);
	queue.finish();
	toc = std::chrono::high_resolution_clock::now();
	profile_info.endian_conv += std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count();
	tic = std::chrono::high_resolution_clock::now();

	processed_image = cv::Mat(height_, width_, CV_8UC4);

	queue.enqueueReadBuffer(outputBuffer, CL_TRUE, 0, width_ * height_* 4, processed_image.data);
	toc = std::chrono::high_resolution_clock::now();
	profile_info.resize += std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count();

	// Wait only for this kernel (DO NOT CALL queue.finish())
	//kernel_event.wait();

	// test bit conversion

	//queue.enqueueReadBuffer(outputBuffer, CL_TRUE, 0, numPixels, output_data);
	//void* mapped_output = queue.enqueueMapBuffer(outputBuffer, CL_TRUE, CL_MAP_READ, 0, numPixels);
	//queue.enqueueUnmapMemObject(outputBuffer, mapped_output);
	//queue.finish(); // Ensure buffer is unmapped before next frame

	tic = std::chrono::high_resolution_clock::now();

	cv::imshow("VIDEO", processed_image);
	toc = std::chrono::high_resolution_clock::now();
	profile_info.visualize += std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count();
}


