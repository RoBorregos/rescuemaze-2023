#!/usr/bin/env python3

# Script to use as camera's controller. Initializes camera service.
# TODO: Check options for reducing latency for tflite model.

# Original script from: https://github.com/openmv/openmv/blob/master/tools/rpc/rpc_image_transfer_jpg_as_the_controller_device.py

import rpc, rospy, io, serial, serial.tools.list_ports, socket, struct, sys, cv2, pytesseract, os
import numpy as np
#import tensorflow as tf
import tflite_runtime.interpreter as tflite # Use on Jetson

from openmv_camera.srv import CameraDetection, CameraDetectionResponse
from PIL import Image


def initInterpreter():
    # Initialize interpreter only at startup
    global interpreter
    #interpreter = tf.lite.Interpreter(model_path="model.tflite")
    interpreter = tflite.Interpreter(model_path="model.tflite") # Use with jetson
    interpreter.allocate_tensors()

def process_image(image):
    # Use interpreter tutorial: https://www.tensorflow.org/api_docs/python/tf/lite/Interpreter
    if debug:
        print(os.getcwd())
        print(os.listdir())

    output = interpreter.get_output_details()[0]  # Model has single output.
    
    if debug:
        print("Output:")
        print(output)

    input = interpreter.get_input_details()[0]  # Model has single input.
    #input_data = tf.constant(1., shape=[1, 1])

    shape = interpreter.get_input_details()[0]['shape'] # get shape of input -> [1, 224, 224, 3]
    
    image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB) # convert to RGB -> make image shape [240, 320, 3]
    image = cv2.resize(image, (shape[1], shape[2])) # resize to input shape
    image = image.reshape(shape) # reshape image to input shape -> [1, 224, 224, 3]

    interpreter.set_tensor(input['index'], image)
    interpreter.invoke()

    res = interpreter.get_tensor(output['index'])[0] # Get probabilities of each class.

    sum_p = 0
    max_p = 0
    for i in range(len(res)):
        sum_p += res[i]
        if res[max_p] < res[i]:
            max_p = i

    output = ['F', 'H', 'S', 'U']

    if debug:
        print("Probabilities of most likely:")
        print(res[max_p]/sum_p)
        print("Most likely:", res[max_p])
        print("All probabilities: ", res)

    # If the probability of the most likely character is greater than 50%, return it. Else return unknown.
    if (res[max_p] / sum_p) > 0.3:
        return output[max_p]

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
    # Use PIL to read image as grayscale.
    # image = Image.open(io.BytesIO(image))
    # image = image.convert("L")
    # image = np.array(image)

    # Use numpy to read image as grayscale.
    image = np.asanyarray(image, dtype="uint8")
    image = cv2.imdecode(image, cv2.IMREAD_GRAYSCALE)
    
    if debug:
        print(image.shape)

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

    if image is None:
        print("Error, image not obtained from RPC.")
        return CameraDetectionResponse('X')

    # Process image
    detection = process_image(image)

    if debug_image and image is not None:
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
    direction = "Left or right"

    # Get parameters
    if rospy.has_param(name + "debug"):
        debug = rospy.get_param(name + "debug")

    if rospy.has_param(name + "debug_image"):
        debug_image = rospy.get_param(name + "debug_image")

    if rospy.has_param(name + "port"):
        port = rospy.get_param(name + "port")

    # Ensure Model.tflite is located at working_dir
    if rospy.has_param(name + "working_dir"):
        working_dir = rospy.get_param(name + "working_dir")
        os.chdir(working_dir)

    if rospy.has_param(name + "direction"):
        direction = rospy.get_param(name + "direction")

    if (debug): portInformation()

    initRPC(port)
    initInterpreter()

    # Start the service
    s = rospy.Service('openmv_camera_' + direction, CameraDetection, detect_any)

    # Spin the node to keep it alive
    rospy.spin()