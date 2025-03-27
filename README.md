# Framos Visualization / Streaming Tool for NXP platforms

This tool enables streaming using v4l2 framework of camera sensors on NXP platforms using available optimizations to reduce resources usage. The tool is made as an alternative for `gstreamer` for developers who need features not supported by `gstreamer` or would prefer more hands on approach to implement and optimize their systems.

Key features:

- Image sensor streaming using v4l2 framework
- Use direct access memory (DMA) for streaming to reduce usage resources
- Also use dma memory buffers on GPU to avoid copying of buffers to GPU
- Use GPU for image transformations (changing of image formats, debayering)
- Use v4l2 or vivante controls to set stream properties / ISP functions
- Output the image as OpenCV `CV:Mat` object for easy debugging.
- No additional libraries needed - just download the tool and run it.

## Limitations

The tool was created for  i.MX 8M Plus platform and [Framos FSM:GO sensors](https://github.com/framosimaging/framos-nxp-drivers) but could be adapted to other NXP platforms and sensors. The binaries are available for [Linux 6.6.3_1.0.0 NXP version](https://www.nxp.com/design/design-center/software/embedded-software/i-mx-software/embedded-linux-for-i-mx-applications-processors:IMXLINUX)
but could be build for other versions as well.

## Using the tool

If you just want to get the stream and visualization as fast as possible download the application from `./build/display_image` and configuration `./config/config.json` and save it to the platform. Open the wayland terminal, or connect to the board via Terra term or ssh connection and run

```bash
./display_image -c config.json
```

You might need to set up the configuration `./config/config.json` file:

```bash
{
 "camera_id": "/dev/video2",
 "subdevice_id": "/dev/v4l-subdev1",
 "resolution": [1920, 1080],
 "pixel_format": "YUYV",
 "verbose": true,
 "v4l2_subdevice_config": {
     "Frame rate": 10,
     "Gain": 300
  },
"use_gpu": false,
"dma_mem": true
}
```

Descriptions of available parameters:

- Video device and subdevice indices check with `v4l2-ctl --list-devices` to find all available devices if the values are different than defauls:
	- `camera_id` defaults to `/dev/video2` 
	- `subdevice_id` subdevice defaults for setting camera controls defaults to `/dev/v4l-subdev1` check with `v4l2-ctl --device=/dev/v4l-subdev1 --list-ctrls` to list all available controls
- `pixel_format` - most important parameter 
	- `YUYV`, `NV12`, `NV16` supported formats if you want to use Image Signal Processor (ISP) supported are  this will automaticaly set image processor pipelint to use g2d api and dma buffers.
	- `RG10` and `RG12` for raw image streaming
- `resolution` - set appropriate resolution supported for your sensor (isp mode in `start_isp.sh` should use the same mode)
- `verbose` - set to true for profiling of application 
- `use_gpu` - only important if raw stream is used, use false if you want to debug the application using opencv, set to true if you want to move some computation (endian conversion, debayering) to GPU
- `v4l2_subdevice_config` - you can use this settings to set stream properties using subdevice, check 
`v4l2-ctl --device=/dev/v4l-subdev1 --list-ctrls` available features for your sensor. To change `data rate`, `frame rate`, `shutter` you must use this controls, you can use vivante controls to set exposure, gain, auto white balance and other ISP functions.
- `dma_mem` cache memory parameter, set to `false` if you wo not want to use cache memory, set to `true` to use dma cache (preferred as this speeds up the system, use the other options only if you need cache memory for different part of your system)

## Architecture of the system

There are three main parts of the system

- Image Streaming - streaming using v4l2 framework
- Image Processing - conversion of image and visualization
- Setting image controls - set streaming properties using v4l2 / vivante controls

## Image Streaming 

 This part uses v4l2 framework and follows closely the standard basic example given in [Linux documentation](https://kernel.org/doc/html/latest/userspace-api/media/v4l/v4l2grab.c.html). It should work without the need to change anything. It allocates the buffer memory where the image from sensors will be stored and loads this buffers in the local memory or saves them in GPU. NXP drivers support two main modes of streaming direct access memory or mmap memory (but not userptr). To both of this approaches the gpu export buffers are added for maximal efficiency. You should use direct access memory which will speed up copying of the buffer for maximal efficiency (this is particularly helpful if you plan to use cpu mode). The only reason why one should switch to mmap mode is if you need cache memory for other parts of your system. You can read more about the difference between this modes in [Linux Kernel documentation](https://kernel.org/doc/html/latest/userspace-api/media/v4l/io.html)

## Description of the available Image Processing Modes

This part copies the frame to local memory and applies image transformations needed to visualize the frame using OpenCV. The default image transformation used here are

- convert from big Endian to little Endian (for raw stream)
- debayering
- tranformation of image from 10 and 12 bits to 8 bits to enable visualization using OpenCV
- visualization of image using OpenCV

In the Image Processing mode you can use function ProcessImage which you can typically adapt to your use case.

### Default mode (using ISP and gpu optimizations)

This is the default and preferred mode that you should use in most cases - unless you need a raw image there is no reason to use other modes. Main features:

- mode is turned on by setting pixel_format to (`YUYV`, `NV12` or `NV16`)
- mode uses optimized g2d api by Verisilicone for transforming image formats.
- ISP functionality is automatically turned on, you can change it with xml configuration file or via vivante controls
- the image is output as CV:Mat RGBA object which you can further adapt to your use case.

G2d Api is Intellectual property of Verisilicone so unfortunately there is no open source code that you can adapt. To investigate the capabilities of this API you can read the chapter 2 of
[i.MX Graphics User's Guide](https://www.nxp.com/docs/en/user-guide/IMX_GRAPHICS_USERS_GUIDE.pdf). The typical problem is to transform between image format or resizing the image using Vivante 
GC520L. The great use of this library is that you can share memory buffers with other libraries supported by NXP (OpenCV, OpenGL ES, OpenVX) in order to avoid costly memory copies which should reduce your reduce your resources usage and speed up your application. Check Sharing Buffers between APIs using G2D Buffers subchapter for more details. You can find more code examples using this api in the [g2d samples nxp repository](https://github.com/nxp-imx/g2d-samples). 

The easiest way to use ISP functionality is to adapt the xml file, if you need to dynamically adapt the parameters you can use [Vivante controls](#Vivante-controls).

### CPU mode for raw image

This is a simple mode using OpenCV functionality on CPU for converting image to Little Endian, Debayering, and scaling image for OpenCV manipulation. You could use this mode if you want raw images without ISP functionality or for simple experiments.

### GPU mode for raw image

This is more advanced mode that uses OpenCL for image transformations on GPU. This allows you a lot of flexibility for modifying your image on GPU, this mode allows you to implement any transformation on GPU that can be done via OpenCL framework.

## Image/Streaming controls

You can change ISP options using vivante external controls or v4l2 if you want to bypass ISP, or set data rate or frame rate. 
You can use only use ISP (vivante) controls for setting ISP image if you use ISP supported formats (`YUYV`, `NV12` or `NV16`).

### Vivante controls

There is a simple example of turning auto exposure on and off commented in `display_image.cpp`. You can also change the default parameters defined in isp xml configuration file in the similar way. To set the functionality that you need use `i.MX 8M Plus Camera and Display Guide.pdf` documentation and the appropriate string controls defined in `ioctl_cmds.h`.

You can also get the value of controls while streaming for example to check if auto exposure is enabled use:

``` bash
v4l2-ctl -d /dev/video2 --set-ctrl=viv_ext_ctrl='{<id>:<ae.g.en>}'
v4l2-ctl -d /dev/video2 -C viv_ext_ctrl
```

To set the value you can use:

``` bash
v4l2-ctl -d /dev/video2 -c viv_ext_ctrl='{<id>:<ae.s.en>;<enable>:true}'
```

### V4l2 controls

To use subdevice controls (V4L2_CID controls defined in imx<ID>_mipi.c) driver you can set the values in this way. Some controls are not supported as Vivante controls (frame rate and data rate for instance so you need this approach). Fow raw image streaming Vivante controls are not enabled so you must use this controls.

Here is an example of configuration that you can add to configuration file.

``` bash
"v4l2_subdevice_config": {
	"Data rate": 2,
	"Frame rate": 50,
	"Exposure": 10000,
	"Gain": 10,
}
```

You can use also this commands from terminal while streaming, for example

``` bash
v4l2-ctl -d /dev/v4l-subdev1 -l
v4l2-ctl -d /dev/v4l-subdev1 -c frame_rate=20
```

## Building The application

### Requirements

Installed toolchain for NXP for full image or some arm compiler for arm64 with all libraries available, installed cmake (tested on version 3.27.5).
You should install the toolchain as described in the instructions in `i.MX_Yocto_Project_User's_Guide` document from the documentation. Here is the needed configuration:

``` bash
DISTRO=fsl-imx-wayland MACHINE=imx8mp-lpddr4-evk source imx-setup-release.sh -b build-wayland
bitbake imx-image-full -c populate_sdk -k
```

## Building

Go to project directory and run

```bash
source /opt/fsl-imx-full-wayland/6.6-nanbield/environment-setup-armv8a-poky-linux # put your toolchain path here
cd build
cmake ..
cmake --build .
```

the display_image application is located at `$REPO/build/display_image`. Copy the application to NXP platform and run it with

```bash
./display_image -c config.json
```

For rebuilding the repo run:

```bash
cmake --build . --target clean && cmake .. && cmake --build .
```

## Other information

Files with examples

```bash
~/isp-imx-4.2.2.24.1/appshell/v4l_drm_test/video_test.cpp (file with example how to use dma memory for gpu)
~/isp-imx-4.2.2.24.1/appshell/vvext/vvext.cpp (file with example how to set vivante controls)
```

Gstreamer commands using dma buffers:

``` bash
gst-launch-1.0 -v v4l2src device=/dev/video2 io-mode=mmap ! queue ! waylandsink
gst-launch-1.0 -v v4l2src device=/dev/video2 io-mode=dma-buff ! queue ! waylandsink
```

## Useful documents

- [i.MX 8M Plus Camera and Display Guide](https://www.nxp.com/docs/en/user-guide/iMX8MP_CAMERA_DISPLAY_GUIDE.pdf)
- [i.MX Graphics User's Guide](https://www.nxp.com/docs/en/user-guide/IMX_GRAPHICS_USERS_GUIDE.pdf)
