#ifndef IMAGEPROCESSOR_H
#define IMAGEPROCESSOR_H

#include <iostream>
#include <imx/linux/dma-buf.h>
#include <cstdint>


#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
//#include <opencv2/core/ocl.hpp>

#include <CL/opencl.hpp>

#include <g2d.h>
#include <g2dExt.h>

struct ProfileInfo {
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

// Abstract class
class ImageProcessor {
protected:
    struct ProfileInfo profile_info = {};
    int width_;
    int height_;
private:
    uint32_t visualize_num_ = 100;
    uint32_t count_frames_ = 0;
public:
    // Pure virtual function
    ImageProcessor() {};
    virtual void ProcessImage(uint8_t* image_data) = 0;  // Pure virtual function
    virtual void ProcessImage(struct dma_buf_phys *buf) = 0;  // Overload for struct
    virtual ~ImageProcessor() {}
    bool SaveImage();
    void PrintProfiling();
    cv::Mat processed_image;
};

// Derived class for CPU processing
class CPUImageProcessor : public ImageProcessor {
public:
    CPUImageProcessor(int width, int height); // Constructor
    ~CPUImageProcessor(); // Constructor
    void convertBigToLittleEndian(cv::Mat& image);
    void ProcessImage(uint8_t *image_data) override;
    void ProcessImage(struct dma_buf_phys *buf) override {
        std::cout << "CPUImageProcessor does not use dma_buf_phys!" << std::endl;
    }

};

// Derived class for G2D processing
class G2DImageProcessor : public ImageProcessor {
public:
    G2DImageProcessor(int width, int height, uint32_t pixel_format); // Constructor
    ~G2DImageProcessor(); // Constructor
    void ProcessImage(uint8_t* image_data) override {
        std::cout << "G2DImageProcessor does not use uint8_t* image data!" << std::endl;
    }
    void ProcessImage(struct dma_buf_phys *buf_addr) override;

private:
    struct g2d_surfaceEx src, dst;
    struct g2d_buf *d_buf;
    enum g2d_format g2_format;
    void *g2d_handle = NULL;
};

// Derived class for GPU processing
class GPUImageProcessor : public ImageProcessor {
public:
    GPUImageProcessor(int width, int height);
    ~GPUImageProcessor();
    void ProcessImage(uint8_t *image_data) override;

    void ProcessImage(struct dma_buf_phys *buf) override {
        std::cout << "GPUImageProcessor does not use dma_buf_phys!" << std::endl;
    }
private:
    cl::Device device;
    cl::Context context;
    cl::CommandQueue queue;
    cl::Program program;
    cl::Program::Sources source;
    cl::Kernel kernel;
    cl::Buffer inputBuffer;
    cl::Buffer outputBuffer;
};

#endif // IMAGEPROCESSOR_H
