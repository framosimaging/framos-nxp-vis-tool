# Framos Visualization / Streaming Tool for NXP platforms

This tool enables streaming using v4l2 framework of camera sensors on NXP platforms using available optimizations to reduce resources usage. The tool is made as an alternative for `gstreamer` for developers who need features not supported by `gstreamer` or would prefer more hands on approach to implement optimizations for their systems.

Key features:

- Use direct access memory (DMA) for streaming to reduce usage resources
- Use GPU for image transformations (changing of image formats, debayering)
- Use v4l2 or vivante controls to set stream properties / ISP functions
- Output the image as OpenCV `CV:Mat` object for easy debugging.

## Limitations

The tool was created for iMX MP platform and Framos FSM:GO sensors but could be adapted to other NXP platforms and sensors. The binaries are available for [Linux 6.6.3_1.0.0 NXP version](https://www.nxp.com/design/design-center/software/embedded-software/i-mx-software/embedded-linux-for-i-mx-applications-processors:IMXLINUX)
but could be build for other versions as well.

## Using the tool

If you just want to get the stream and visualization as fast as possible download the application from `./build/display_image` and configuration `./config/config.json` and save it to the platform.

Open the wayland terminal or connect to the board via Terra term or ssh connection and run

```bash
./display_image -c config.json -m 1
```

You might need to set up the configuration file:

```bash
{
	"camera_id": "/dev/video2",
	"subdevice_id": "/dev/v4l-subdev1",
	"resize": [1280, 720],
	"resolution": [1920, 1080],
	"pixel_format": "YUYV",
	"verbose": true,
	"v4l2_subdevice_config": {
		"Frame rate": 10,
		"Gain": 300
	},
	"use_gpu": false
}
```

Descriptions of available parameters:

- Video device and subdevice indices check with `v4l2-ctl --list-devices` to find all available devices if the values are different than defauls:
	- `camera_id` defaults to `/dev/video2` 
	- `subdevice_id` subdevice defaults for setting camera controls defaults to `/dev/v4l-subdev1` check with `v4l2-ctl --device=/dev/v4l-subdev1 --list-ctrls` to list all available controls
- `pixel_format` - most important parameter 
	- `YUYV`, `NV12`, `NV16` supported formats if you want to use Image Signal Processor (ISP) supported are  this will automaticaly set image processor pipelint to use g2d api and dma buffers.
	- `RG12` for raw image streaming, `RG10` will be supported soon
- `verbose` - set to true for profiling of application 
- `use_gpu` - only important if raw stream is used, use false if you want to debug the application using opencv, set to true if you want to move some computation (endian conversion, debayering) to GPU
- `v4l2_subdevice_config` - you can use this settings to set stream properties using subdevice, check 
`v4l2-ctl --device=/dev/v4l-subdev1 --list-ctrls` available features for your sensor. To change `data rate`, `frame rate`, `shutter` you must use this controls, you can use vivante controls to set exposure, gain, auto white balance and other ISP functions.
- `-m` cache memory parameter, set to 0 if you wo not want to use cache memory, set to 1 to use dma cache (preferred as this speeds up the system, use the other options only if you need cache memory for different part of your system)

## Architecture of the system

There are three main parts of the system

- Image Streaming (Buffers)
- Image Processing Conversion of image and visualization
- Setting image controls ()

## Image Streaming (Buffers)

 This part is uses v4l2 framework and follows closely the standard basic example given in [Linux documentation](https://kernel.org/doc/html/latest/userspace-api/media/v4l/v4l2grab.c.html). It allocates the buffer memory where the image from sensors will be stored and loads this buffers in the local memory or saves them in GPU. NXP drivers support two main modes of streaming direct access memory or mmap memory (but not userptr).To both of this approaches the gpu export buffers are added for maximal efficiency. You should use direct access memory which will speed up copying of the buffer for maximal efficiency (this is particularly helpful if you plan to use cpu mode). The only reason why one should switch to mmap mode is if you need cache memory for other parts of your system. You can read more about the difference between this modes in [Linux Kernel documentation](https://kernel.org/doc/html/latest/userspace-api/media/v4l/io.html)

## Description of the available Image Processing Modes

In the Image Processing mode you can use function ProcessImage to adapt it for your use case. Currently it is used for debayering and visualization of Image by OpenCV.

### Default mode (using ISP and gpu optimizations)

This is the default and preferred mode that you should use in most cases unless you need a raw image there is no reason to use other modes. Main features:
- mode is turned on by setting pixel_format to (`YUYV`, `NV12` or `NV16`)
- mode uses g2d api by Verisiliocne for transforming image formats.

### CPU mode for raw image

Use OpenCV on CPU for converting image to Little Endian, Debayering, and scaling image for OpenCV manipulation.

### GPU mode for raw image

Use of OpenCL for image transformations. This allows you a lot of flexibility for modifying your image on GPU.

## Image/Streaming controls

You can use ISP (vivante) controls for setting ISP image if you use ISP supported formats (`YUYV`, `NV12` or `NV16`).

### Vivante controls

There is a simple example of turning auto exposure on and off commented in `display_image.cpp`. You can also change the default parameters defined in isp xml configuration file in the similar way. To set the functionality that you need use `i.MX 8M Plus Camera and Display Guide.pdf` documentation and the appropriate string controls defined in `ioctl_cmds.h`.

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

You can use also this commands from terminal while streaming

``` bash
v4l2-ctl -d /dev/v4l-subdev1 -l
v4l2-ctl -d /dev/v4l-subdev1 -c frame_rate=20
```

## Building The application

### Requirements

Installed toolchain for NXP for full image or some arm compiler for arm64 with all libraries available, install cmake (tested on version 3.27.5).
You should install the toolchain as described in the instructions in i.MX_Yocto_Project_User's_Guide document from documentation. Here is the needed configuration:

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
./display_image -c config.json -m 1
```

For rebuilding the repo run:

```bash
cmake --build . --target clean
cmake ..
cmake --build .
```

## Other information 

Files with examples
~/isp-imx-4.2.2.24.1/appshell/v4l_drm_test/video_test.cpp (file with example how to use dma memory for gpu)

Gstreamer commands to use dma buffers:

``` bash
gst-launch-1.0 -v v4l2src device=/dev/video2 io-mode=mmap ! queue ! waylandsink
gst-launch-1.0 -v v4l2src device=/dev/video2 io-mode=dma-buff ! queue ! waylandsink
```

## Useful documents

- [i.MX 8M Plus Camera and Display Guide](https://www.nxp.com/docs/en/user-guide/iMX8MP_CAMERA_DISPLAY_GUIDE.pdf)
- [i.MX Graphics User's Guide](https://www.nxp.com/docs/en/user-guide/IMX_GRAPHICS_USERS_GUIDE.pdf)
