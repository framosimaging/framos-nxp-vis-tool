import numpy as np
import matplotlib.pyplot as plt
import binascii
from PIL import Image
import cv2
def read_nv12_image(file_path, width, height):
    with open(file_path, 'rb') as f:
        frame_size = width * height
        y_plane = np.frombuffer(f.read(frame_size), dtype=np.uint8).reshape((height, width))
        uv_plane = np.frombuffer(f.read(frame_size // 2), dtype=np.uint8).reshape((height // 2, width))
    nv12 = np.vstack((y_plane, uv_plane))
    return nv12

def nv12_to_rgb(nv12_image):
    """
    Convert NV12 data to RGB using OpenCV.
    """
    # Convert NV12 to BGR (OpenCV default) and then to RGB
    bgr_image = cv2.cvtColor(nv12_image, cv2.COLOR_YUV2BGR_NV12)
    rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
    return rgb_image
     
def read_raw_image(file_path, height, bit_format):
    """
    Reads a raw image file and returns it as a NumPy array.

    :param file_path: Path to the raw image file.
    :param width: Width of the image.
    :param height: Height of the image.
    :return: NumPy array representing the image.
    """
    # Read the raw file
    with open(file_path, 'rb') as file:
        raw_data = file.read()

    hex_data = binascii.hexlify(raw_data)
    hex_string = hex_data.decode('utf-8')

    numpy_dtype = np.uint8 if (bit_format == 8) else np.uint16

    # Convert the raw data to a NumPy array
    image = np.frombuffer(raw_data, dtype=numpy_dtype)

    #divisors = arr[image.size % arr == 0]
    # width often has additional bits to fit 64 /32 format that is why it is not reliable
    # so it is better to compute it from height
    width = image.size // height
    image_tmp = image.reshape((height, width))

    # image1 = image[:,0:(width // 2 + 1)]
    # image2 = image[:,(width // 2 + 1):]
    # plt.imsave("image" + ".png", image_tmp, cmap='gray')
    # plt.close()
    return image_tmp
    """
    plt.imsave("image1.png", image1, cmap='gray')
    plt.close()
    plt.imsave("image2.png", image2, cmap='gray')
    plt.close()
    print("bla bla")

    """
    return image

# Example usage:
bit_format = 8
image_name = "frame"
DIR = 'c:\\Projects\\Auterion\\analyze_raw\\'
file_path =  DIR + image_name + '.raw'
# file_path = 'C:\\cti_tech\\nvidia\\test_pattern_org.raw'

output_path = DIR + "out" + image_name + '.png'
# width = 3568  # Replace with the actual width of your image
height = 1080  # Replace with the actual height of your image
width = 1920
# Read and display the image
# image = read_raw_image(file_path, height, bit_format)
# if bit_format != 8:
# image = (image << 4).astype(np.uint16)
# if bit_format != 8:
# 	image = (image << 4).astype(np.uint16)


nv12_image = read_nv12_image(file_path, width, height)
rgb_image = nv12_to_rgb(nv12_image)



im = Image.fromarray(rgb_image)
im.save(output_path)



# tr_image = transform_image(image, width, height)
# display_image(image, output_path)