#!/usr/bin/env python3

# Script to save images for dataset capture. Uses the same settings as the runtime input.


import rpc, rospy, io, serial, serial.tools.list_ports, socket, struct, sys, cv2, pytesseract
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
    #image = Image.open(io.BytesIO(image))
    image = cv2.imdecode(image)
    print(image.shape)
    #image = np.array(image)
    return image


def formated_image():
    # Get image
    image = get_image("sensor.RGB565", "sensor.QVGA")
    image = format_image(image)

    return image

def get_new_image(class_name):
    class_name += '_image_'

    index = 0
    while True:
        image = formated_image()
        title = "Detected image"
        cv2.imshow(title, image)
        key = cv2.waitKey(0)
        if key == ord('n'):
            # Continue if photo output is not correct/stable
            continue
        elif key == ord('s'):
            # Save image
            index += 1
            cv2.imwrite(class_name + str(index) + '.jpg', image)
        elif key == ord('q'):
            break
    cv2.destroyAllWindows()


def initRPC(port):
    global interface
    interface = rpc.rpc_usb_vcp_master(port)


def portInformation():
    print("\nAvailable Ports:\n")

    for port, desc, hwid in serial.tools.list_ports.comports():
        print("{} : {} [{}]".format(port, desc, hwid))


if __name__ == '__main__':
    # Initialize the node
    port = '/dev/ttyACM0' # default value for port
    
    class_name = 'H' # The name of the dataset to capture

    portInformation()

    initRPC(port)

    get_new_image(class_name)