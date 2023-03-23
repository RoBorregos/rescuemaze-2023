#!/usr/bin/env python3

# Script to use as camera's controller. Initializes camera service.

# Original script from: https://github.com/openmv/openmv/blob/master/tools/rpc/rpc_image_transfer_jpg_as_the_controller_device.py

import rpc, rospy, io, serial, serial.tools.list_ports, socket, struct, sys, cv2, pytesseract
import numpy as np

from openmv_camera.srv import CameraDetection, CameraDetectionResponse
from PIL import Image

# TODO: implement function to detect letter in image.
def process_image(image):
    text = pytesseract.image_to_string(image)

    if debug:
        for i in range(len(text)):
            letter = text[i].upper()
            print('letter:', letter)
    
    for i in range(len(text)):
        letter = text[i].upper()
        if letter == 'H' or letter == 'S' or letter == 'U':
            return letter
    
    return 'X'


def get_image(pixformat_str, framesize_str):
    if debug: print("Getting Remote Frame...")

    # Add image to camera's buffer, with specified sensor settings.
    result = interface.call("jpeg_image_snapshot", "%s,%s" % (pixformat_str, framesize_str))
    
    if debug: print("Result: ", result)

    if result is not None:

        size = struct.unpack("<I", result)[0] # Get size of image to create bytearray
        img = bytearray(size)

        if debug: print("Size: ", size)

        result = interface.call("jpeg_image_read") # Actually read the image.

        if result is not None:
            # GET BYTES NEEDS TO EXECUTE NEXT IMMEDIATELY WITH LITTLE DELAY NEXT.
            # Read all the image data in one very large transfer.
            interface.get_bytes(img, 5000) # timeout
            if debug_image: print(img)
        else:
            return None
        return img

    else:
        if debug: print("Failed to get Remote Frame!")

        return None
    

def unpack_res(res):
    if debug: print(res)

    return str(bytes(res).decode())


# From bytearray into suitable np.array for use with cv2.
def format_image(image):
    image = Image.open(io.BytesIO(image))
    image = np.array(image)
    return image


def detect_any(req):

    # First try to detect if there is red, yellow, or green.
    detection = unpack_res(interface.call('detect_dominant_color'))

    if (debug):
        print("Dominant color detected:", end=" ")
        print(detection)
    
    if detection == 'r' or detection == 'y' or detection == 'g':
        return CameraDetectionResponse(detection)

    # If there is no color, detect if there is a letter.

    # Get image
    image = get_image("sensor.RGB565", "sensor.QVGA")
    image = format_image(image)

    if debug: print(image.shape)


    if image is None:
        print("Error, image not obtained from RPC.")
        return CameraDetectionResponse('X')

    # Process image
    detection = process_image(image)

    if True and debug_image and image is not None:
        cv2.imshow("DebugCamera", image)
        key = cv2.waitKey(0)

    # If valid result, return it. Else, return failure/unknown character.
    if detection == 'H' or detection == 'S' or detection == 'U':
        return CameraDetectionResponse(detection)

    return CameraDetectionResponse('X') # Character to signal that no suitable data was obtained.


def initRPC(port):
    global interface
    interface = rpc.rpc_usb_vcp_master(port)

def portInformation():
    print("\nAvailable Ports:\n")

    for port, desc, hwid in serial.tools.list_ports.comports():
        print("{} : {} [{}]".format(port, desc, hwid))

if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('openmv_camera')
    rospy.loginfo("Open mv camera service initialized")

    name = rospy.get_name() + "/"

    port = '/dev/ttyACM0' # default value for port
    global debug, debug_image
    debug = False
    debug_image = False

    # Get parameters
    if rospy.has_param(name + "debug"):
        debug = rospy.get_param(name + "debug")

    if rospy.has_param(name + "debug_image"):
        debug_image = rospy.get_param(name + "debug_image")

    if rospy.has_param(name + "port"):
        port = rospy.get_param(name + "port")

    if (debug): portInformation()

    initRPC(port)

    # Start the service
    s = rospy.Service('openmv_camera', CameraDetection, detect_any)

    # Spin the node to keep it alive
    rospy.spin()