#!/usr/bin/env python3

# Script to save images for dataset capture. Uses the same settings as the runtime input.


import rpc, rospy, io, serial, serial.tools.list_ports, socket, struct, sys, cv2, pytesseract, os
import numpy as np

from openmv_camera.srv import CameraDetection, CameraDetectionResponse
from PIL import Image


def get_image(pixformat_str, framesize_str):

    # Add image to camera's buffer, with specified sensor settings.
    result = interface.call("jpeg_image_snapshot", "%s,%s" % (pixformat_str, framesize_str))

    print(result)

    if result is not None:

        size = struct.unpack("<I", result)[0] # Get size of image to create bytearray
        img = bytearray(size)

        print("Size: ", size)

        result = interface.call("jpeg_image_read") # Actually read the image.

        if result is not None:
            # GET BYTES NEEDS TO EXECUTE NEXT IMMEDIATELY WITH LITTLE DELAY NEXT.
            # Read all the image data in one very large transfer.
            interface.get_bytes(img, 5000) # timeout
            print(img)
        else:
            return None
        return img

    else:
        print("Failed to get Remote Frame!")

        return None
    

# From bytearray into suitable np.array for use with cv2.
def format_image(image):
    data2 = image.copy()
    image = Image.open(io.BytesIO(image))
    image = image.convert("L")
    #image = image.convert("RGB")   
    image = np.array(image)
    
    image2 = np.asanyarray(data2, dtype="uint8")
    image2 = cv2.imdecode(image2, cv2.IMREAD_GRAYSCALE)
    
    return [image, image2]


def formated_image():
    # Get image
    image = get_image("sensor.RGB565", "sensor.QVGA")
    image, image2 = format_image(image)

    return [image, image2]

def get_new_image(class_name, numpy=False, start_index=1):
    class_name += '_image_'
    start_index -= 1

    if numpy:
        print("Attention: numpy used to save images. Use same config for runtime use.")
    else:
        print("Attention: PIL used to save images. Use same config for runtime use.")
    index = 0
    while True:
        image, image2 = formated_image()
        print("Shape using PIL:")
        print(image.shape)
        print("Shape using NP:")
        print(image2.shape)
        title = "Detected image"
        title2 = "Using numpy"
        cv2.imshow(title, image)
        cv2.imshow(title2, image2)

        key = cv2.waitKey(0)
        if key == ord('n'):
            # Continue if photo output is not correct/stable
            continue
        elif key == ord('s'):
            # Save image
            index += 1
            if numpy:
                cv2.imwrite(class_name + "np_" + str(index + start_index) + '.jpg', image)
            else:
                cv2.imwrite(class_name + str(index + start_index) + '.jpg', image)
        elif key == ord('q'):
            # Quit script
            break
    cv2.destroyAllWindows()


def initRPC(port):
    global interface
    interface = rpc.rpc_usb_vcp_master(port)


def portInformation():
    print("\nAvailable Ports:\n")

    for port, desc, hwid in serial.tools.list_ports.comports():
        print("{} : {} [{}]".format(port, desc, hwid))


def get_path(base, dataset_name, class_name):
    class_name += '.class'
    return base + dataset_name + '/' + class_name + '/'


if __name__ == '__main__':
    
    port = '/dev/ttyACM0' # default value for port
    use_numpy = True # Use numpy to save images
    dataset_base_path = 'maze_ws/src/devices/openmv_camera/scripts/datasets/'

    start_index = 1 # The index to start saving images from
    class_name = 'F' # The name of the class to capture
    dataset_name = 'dataset4'# The name of the dataset to capture
    
    save_path = get_path(dataset_base_path, dataset_name, class_name)

    print("Saving images to: ", save_path)
    
    # Create dir if not exists
    if not os.path.exists(save_path):
        os.makedirs(save_path)
    
    # Change dir
    os.chdir(save_path)

    portInformation()

    initRPC(port)

    # Numpy is used to save the images and feed them to the model
    get_new_image(class_name, numpy=use_numpy, start_index=start_index)