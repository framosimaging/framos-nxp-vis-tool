#include "buffers.hpp"
#include <iostream>
#include <cstring>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <cerrno>
#include <cerrno>


/*
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
*/
/*
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
*/
/*
void *dma_buf_map(int dma_fd, size_t size) {
    void *mapped_addr = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, dma_fd, 0);
    if (mapped_addr == MAP_FAILED) {
        perror("Failed to mmap DMA buffer");
        return NULL;
    }
    return mapped_addr;
}
*/
/*
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
    *//*

    // Cleanup
    munmap(dma_ptr, size);

}
*/

bool yioctl(int fd, unsigned long request, void* arg) {
    uint32_t retry_attempts = 0;
    static constexpr uint32_t max_retry = 2;
    while (::ioctl(fd, request, arg) == -1 && retry_attempts < max_retry) {
        ++retry_attempts;
    }
    return retry_attempts != max_retry;
}

V4l2Buffers::V4l2Buffers(int32_t fd, int32_t dma_mem, uint32_t size_image)
  :fd_(fd), dma_mem_(dma_mem), size_image_(size_image) {
  v4l2_memory_ = (dma_mem == 1) ? V4L2_MEMORY_DMABUF : V4L2_MEMORY_MMAP;
  memset(&req_bufs_, 0, sizeof(req_bufs_));
}

V4l2Buffers::~V4l2Buffers() {
  if (stream_on_)
    V4l2Buffers::StopStream();
  V4l2Buffers::ReleaseBuffers();
}

bool V4l2Buffers::RequestBuffers() {
  req_bufs_.count = buf_count_;
  req_bufs_.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req_bufs_.memory = v4l2_memory_;
  bool success = yioctl(fd_, VIDIOC_REQBUFS, &req_bufs_);
  if (!success) {
    std::cerr << "Requesting Buffer failed with error " << strerror(errno) << std::endl;
    return false;
  }
  if (req_bufs_.count < buf_count_) {
    std::cerr << "Insufficient buffer memory " << req_bufs_.count << std::endl;
    return false;
  }

  buffers.resize(req_bufs_.count);
  return true;
}

bool V4l2Buffers::Initmmap(){

  for (uint32_t i = 0; i < buffers.size(); i++) {
    struct v4l2_buffer buf {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    bool success = yioctl(fd_, VIDIOC_QUERYBUF, &buf);
    if (!success) {
      std::cerr << "VIDIOC_QUERYBUF failed" << std::endl;
      return false;
    }
    void *buffer_start = ::mmap (NULL, 
      size_image_, 
      PROT_READ | PROT_WRITE, 
      MAP_SHARED, 
      fd_, 
      buf.m.offset);
      if (buffer_start == nullptr) {
        std::cout << "mmap failed" << std::endl;
        return false;
      }

    if (buffer_start == MAP_FAILED) {
        std::cerr << "mmap failed for buffer " << i << std::endl;
        return false;
    }
    std::memset(buffer_start, 0, size_image_);

    buffers[i].index = buf.index;
    buffers[i].rawLength = size_image_;
    buffers[i].rawData = static_cast<uint8_t*>(buffer_start);

    }

   return true;
}

bool V4l2Buffers::QueueAllBuffers() {
    bool success;
    /*
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
    */
  for (uint32_t i = 0; i < buffers.size(); i++){
    struct v4l2_buffer buf {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    success = yioctl(fd_, VIDIOC_QBUF, &buf);
    if (!success) {
      std::cerr << "VIDIOC_QBUF failed with error" << strerror(errno) << std::endl;
      return false;
    }
  }

  return true;
}

bool V4l2Buffers::DequeueBuffers(uint8_t **image_data) {
  bool success;
  buf_.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf_.memory = V4L2_MEMORY_MMAP;
  
  /*if (dma_mem_ == 1)
    buf.memory = V4L2_MEMORY_DMABUF;
  else 
    buf.memory = V4L2_MEMORY_MMAP;
  */
  success = yioctl(fd_, VIDIOC_DQBUF, &buf_);
  if (!success) {
    std::cerr << "VIDIOC_DQBUF failed" << strerror(errno) << std::endl;
    return false;
  }
  *image_data = &buffers[buf_.index].rawData[0];

  return true;
}

bool V4l2Buffers::QueueBuffers() {
  bool success;

  success = yioctl(fd_, VIDIOC_QBUF, &buf_);
  if (!success) {
    std::cerr << "VIDIOC_QBUF failed with error" << strerror(errno) << std::endl;
    return false;
  }

  return true;
}

bool V4l2Buffers::CaptureFrame(uint8_t* output) {
  struct v4l2_buffer buf {};
  bool success;

  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (dma_mem_ == 1)
    buf.memory = V4L2_MEMORY_DMABUF;
  else 
    buf.memory = V4L2_MEMORY_MMAP;

  success = yioctl(fd_, VIDIOC_DQBUF, &buf);
    if (!success) {
      std::cerr << "VIDIOC_DQBUF failed" << std::endl;
      return false;
   }
    // REFACTOR
    //frame = cv::Mat(1080 + 1080 / 2, 1920, CV_8UC1, &buffers[buf.index].rawData[0]);
    memcpy(output, &buffers[buf.index].rawData[0], size_image_);
    success = yioctl(fd_, VIDIOC_QBUF, &buf);
    if (!success) {
      std::cerr << "VIDIOC_QBUF failed with error" << strerror(errno) << std::endl;
      return false;
    }

    return true;
}

bool V4l2Buffers::StartStream() {
  std::cout << "Start streaming" << std::endl;
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  bool success = yioctl(fd_, VIDIOC_STREAMON, &type);
  if (!success) {
        std::cerr << "VIDIOC_STREAMON failed" << std::endl;
        return false;
    }
  stream_on_ = true;
  return true;
}

bool V4l2Buffers::StopStream() {
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  bool success = yioctl(fd_, VIDIOC_STREAMOFF, &type);
  if (!success) {
        std::cerr << "VIDIOC_STREAMOFF failed" << std::endl;
        return false;
  }
  stream_on_ = false;
  return true;
}

void V4l2Buffers::ReleaseBuffers() {
  for (uint32_t i = 0; i < buffers.size(); i++) {
    if (buffers[i].rawData != nullptr) {
      int ret = ::munmap(buffers[i].rawData, buffers[i].rawLength);
      if (ret < 0) {
	std::cerr << "Buffer " << i << "not unmapped: " << strerror(errno) << std::endl;
      }
    }
    // Set the pointer to nullptr for safety
    buffers[i].rawData = nullptr;
    buffers[i].rawLength = 0;
  }
}