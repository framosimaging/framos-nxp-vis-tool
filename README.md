# Streaming / visualization using v4l2src


## Requirements

Installed toolchain for NXP or some arm compiler for arm64 with all libraries available, install cmake (tested on version 3.27.5).

## Building

Go to project directory and run

```bash
mkdir build
cd build
cmake ..
cmake --build .
```

the display_image application is located at `$REPO/build/display_image`. Copy the application to NXP platform and run it with.
For rebuilding the repo run:

```bash
cmake --build . --target clean
cmake ..
cmake --build .
```

## Building only streaming

You need to have a compiler as one obtained from toolchain in https://github.com/framosimaging/framos-nxp-drivers 

``` bash
$CXX display_image.cpp -o display_image
```

### Building with `opencv` for visualization

For opencv you need to install toolchain for full image

``` bash
DISTRO=fsl-imx-wayland MACHINE=imx8mp-lpddr4-evk source imx-setup-release.sh -b build-wayland
bitbake imx-image-full -c populate_sdk -k
```

or find a way to add opencv to imx-image-multimedia. Then build with

``` bash
source /opt/fsl-imx-full-wayland/6.6-nanbield/environment-setup-armv8a-poky-linux # put your toolchain path here
$CXX display_image.cpp -o display_image $(pkg-config --cflags --libs opencv4)
```

Copy the `display_image` to target. When prompted copy libraries `libopencv_ts*` and `libopencv_superres*` from toolchain path on host (`/opt/fsl-imx-full-wayland/6.6-nanbield/sysroots/cortexa53-crypto-poky-linux/usr/lib/`)  libraries to `/usr/lib` on target. Also you will need to flash full image (`imx-image-full-imx8mpevk.wic`).

## Building with dma buffers for gpu

``` bash
source /opt/fsl-imx-full-wayland/6.6-nanbield/environment-setup-armv8a-poky-linux # put your toolchain path here
$CXX display_image.cpp -o display_image $(pkg-config --cflags --libs opencv4) -lg2d
```

Based on your installation you might need to copy the library from toolchain build

``` bash
cd ~/imx-yocto-bsp/build-wayland/tmp/work/armv8a-mx8mp-poky-linuximx-gpu-g2d/
sudo cp -v ./6.4.11.p2.4/sysroot-destdir/usr/lib/libg2d* /opt/fsl-imx-full-wayland/6.6-nanbield/sysroots/armv8a-poky-linux/usr/lib
```

## Streaming configuration

``` bash
./display_image -c config.json -m 2
```
-m is for choosing of memory (0 mmap, 1 for dma, 2 dma gpu).
The configuration is set using json file.

### V4l2 controls

To use subdevice controls (V4L2_CID controls defined in imx<ID>_mipi.c) driver you can set the values in this way. Some controls are not supported as Vivante controls (frame rate and data rate for instance). For other controls (exposure, gain) we recommend using Vivante controls to avoid confusion later.

``` bash
"v4l2_subdevice_config": {
	"Data rate": 2,
	"Frame rate": 50,
	"Exposure": 10000,
	"Gain": 10,
}
```

-v 1 is for profiling, 

## Tools

Change code to save image frame.raw.
Using analyze_raw.py to transform the image.

## Other info

Files with examples
~/nxp/isp-vvcam/vvcam/v4l2/video/video.c (driver implementation from NXP, our driver is subdevice of this driver)
~/isp-imx-4.2.2.24.1/appshell/v4l_drm_test/video_test.cpp (file with example how to use dma memory for gpu)

Useful commands

``` bash
gst-launch-1.0 -v v4l2src device=/dev/video2 io-mode=mmap ! queue ! waylandsink
gst-launch-1.0 -v v4l2src device=/dev/video2 io-mode=dma-buff ! queue ! waylandsink
```

Subdev commands
v4l2-ctl -d /dev/v4l-subdev1 -l
v4l2-ctl -d /dev/v4l-subdev1 -c frame_rate=20

useful documents:
i.MX Graphics User's Guide