#ifndef V4L2_BUFFERS_H
#define V4L2_BUFFERS_H

#include <vector>
//#include <cstring>
#include <cstdint>
#include <linux/videodev2.h>
#include <imx/linux/dma-buf.h>

struct V4l2Buffer {
    uint8_t *rawData = nullptr;
    uint32_t rawLength = 0;
    uint32_t index = 0;
    int32_t dma_fd = -1; // file descriptor used for dma buffer
};

class V4l2Buffers {
public:
  V4l2Buffers(int32_t fd, int32_t dma_mem, uint32_t size_image);
  ~V4l2Buffers();
  std::vector<V4l2Buffer> buffers;
  bool RequestBuffers();
  virtual bool AllocateBuffers() = 0;
  bool QueueAllBuffers();
  bool CaptureFrame(uint8_t* output);
  bool StartStream();
  bool QueueBuffers();
  bool DequeueBuffers(uint8_t **image_data);
  void ReleaseBuffers();
  bool StopStream();

  struct dma_buf_phys buf_addr;

protected:

  int32_t dma_mem_;
  enum v4l2_memory v4l2_memory_;
  int32_t fd_;
  uint32_t size_image_;
  struct v4l2_requestbuffers req_bufs_;
  static constexpr uint32_t buf_count_ = 3;
  struct v4l2_buffer buf_ {};
  bool stream_on_ = false;
  struct dma_buf_phys buf_addrs_[buf_count_];
};

class MMAPBuffers : public V4l2Buffers
{
public:
  MMAPBuffers(int32_t fd, int32_t dma_mem, uint32_t size_image)
    :V4l2Buffers(fd, dma_mem, size_image) {}
  
  bool AllocateBuffers() override; // 
  ~MMAPBuffers() {}

private:
  bool test2;
};

class DMABuffers : public V4l2Buffers
{
public:
  DMABuffers(int32_t fd, int32_t dma_mem, uint32_t size_image)
    :V4l2Buffers(fd, dma_mem, size_image) {
  };
  ~DMABuffers();
  bool AllocateBuffers() override; // 

private:
  int dma_fd_;
};

#endif // V4L2_BUFFERS_H