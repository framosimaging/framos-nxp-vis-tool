#include "buffers.hpp"
#include <iostream>
#include <cstring>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <cerrno>
#include <cerrno>
#include <linux/dma-heap.h>
#include <imx/linux/dma-buf.h>
#include <unistd.h>
#include <fcntl.h>

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
  memset(&buf_addrs_, 0, sizeof(buf_addrs_));
  memset(&buf_addr, 0, sizeof(buf_addr));
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

bool V4l2Buffers::QueueAllBuffers() {
  for (uint32_t i = 0; i < buffers.size(); i++){
    struct v4l2_buffer buf {};
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = v4l2_memory_;
    buf.index = i;
    buf.m.fd = buffers[i].dma_fd;
    bool success = yioctl(fd_, VIDIOC_QBUF, &buf);
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
  buf_.memory = v4l2_memory_;
  
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
  buf_addr = buf_addrs_[buf_.index];
  //image_data = std::span<uint8_t>(buffers[buf_.index].rawData, buffers[buf_.index].rawLength);

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

    if (buffers[i].dma_fd > -1) {
	close(buffers[i].dma_fd);
    }
    // Set the pointer to nullptr for safety
    buffers[i].rawData = nullptr;
    buffers[i].rawLength = 0;
  }
}


bool MMAPBuffers::AllocateBuffers() {
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

    struct v4l2_exportbuffer expbuf;
    memset(&expbuf, 0, sizeof(expbuf));
    expbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    expbuf.index = i;
    if (ioctl(fd_, VIDIOC_EXPBUF, &expbuf) < 0) {
       std::cerr << "VIDIOC_EXPBUF" << strerror(errno) <<  std::endl;
       return false;
    }
    std::cout << " EXPBUFFER PLANE \n" << expbuf.plane << std::endl;
    std::cout << " EXPBUFFER FLAGS \n" << expbuf.flags << std::endl;
    std::cout << " EXPBUFFER FD \n" << expbuf.fd << std::endl;
    if (ioctl(expbuf.fd, DMA_BUF_IOCTL_PHYS, &buf_addrs_[i]) < 0) {
	std::cerr << " EXPBUFFER DMA_BUF_IOCTL_PHYS error \n" << expbuf.fd << std::endl;
	return false;
    }
  }
   return true;
}

DMABuffers::~DMABuffers() {
    close(dma_fd_);
}

bool DMABuffers::AllocateBuffers() {
  std::cout << "Allocating and Queing buffers for dma buffers" << std::endl;
  dma_fd_ = open("/dev/dma_heap/linux,cma", O_RDWR | O_CLOEXEC);
  if (dma_fd_ < 0) {
    std::cerr << "Failed to open DMA heap " << std::endl;
    return false;
  }

  for (uint32_t i = 0; i < buffers.size(); i++) {
    struct dma_heap_allocation_data alloc_data;
    memset(&alloc_data, 0, sizeof(alloc_data));
    alloc_data.len = size_image_;
    alloc_data.fd_flags = O_RDWR;
    if (ioctl(dma_fd_, DMA_HEAP_IOCTL_ALLOC, &alloc_data) < 0) {
      std::cerr << "Failed to allocate dma memory index " << i << std::endl;
      close(dma_fd_);
      return false;
    }

    buffers[i].dma_fd = alloc_data.fd;
    void *buffer_start = mmap(NULL, alloc_data.len, PROT_READ | PROT_WRITE, MAP_SHARED, buffers[i].dma_fd, 0);

    if (buffer_start == MAP_FAILED) {
        std::cerr << "mmap failed " << i << std::endl;
        goto cleanup;
    }
    
    buffers[i].index = i; // TODO is this needed?
    buffers[i].rawLength = size_image_;
    buffers[i].rawData = static_cast<uint8_t*>(buffer_start);
    if (ioctl(buffers[i].dma_fd, DMA_BUF_IOCTL_PHYS, &buf_addrs_[i]) < 0) {
	std::cerr << " EXPBUFFER DMA_BUF_IOCTL_PHYS error \n" << buffers[i].dma_fd << std::endl;
	return false;
    }
  }

  return true;

cleanup:
    std::cerr << "Cleaning up after failure..." << std::endl;
    for (uint32_t j = 0; j <= buffers.size(); j++) {
        if (buffers[j].dma_fd > 0) {
            close(buffers[j].dma_fd);
        }
        if (buffers[j].rawData) {
            munmap(buffers[j].rawData, size_image_);
        }
    }
    close(dma_fd_);
    return false;
} 


bool DMAGPUBuffers::AllocateBuffers() {
  return true;
}