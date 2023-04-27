#!/usr/bin/env python3

# Script to save images to jetson to test if cameras lenses are calibrated


import rpc, rospy, serial, serial.tools.list_ports, struct, cv2, rospkg, os
import numpy as np


def get_image_right(pixformat_str, framesize_str):
    if debug: print("Getting Remote Frame...")

    # Add image to camera's buffer, with specified sensor settings.
    resultRight = interfaceRight.call("jpeg_image_snapshot", "%s,%s" % (pixformat_str, framesize_str))

    if resultRight is not None:

        size = struct.unpack("<I", resultRight)[0] # Get size of image to create bytearray
        img = bytearray(size)

        if debug: print("Size: ", size)

        resultRight = interfaceRight.call("jpeg_image_read") # Actually read the image.

        if resultRight is not None:
            # GET BYTES NEEDS TO EXECUTE NEXT IMMEDIATELY WITH LITTLE DELAY NEXT.
            # Read all the image data in one very large transfer.
            interfaceRight.get_bytes(img, 5000) # timeout
            
        else:
            return None
        return img

    else:
        if debug: print("Failed to get Remote Frame!")

        return None
    
    
def get_image_left(pixformat_str, framesize_str):
    if debug: print("Getting Remote Frame...")

    # Add image to camera's buffer, with specified sensor settings.
    resultLeft = interfaceLeft.call("jpeg_image_snapshot", "%s,%s" % (pixformat_str, framesize_str))

    if resultLeft is not None:

        size = struct.unpack("<I", resultLeft)[0] # Get size of image to create bytearray
        img = bytearray(size)

        if debug: print("Size: ", size)

        resultLeft = interfaceLeft.call("jpeg_image_read") # Actually read the image.

        if resultLeft is not None:
            # GET BYTES NEEDS TO EXECUTE NEXT IMMEDIATELY WITH LITTLE DELAY NEXT.
            # Read all the image data in one very large transfer.
            interfaceLeft.get_bytes(img, 5000) # timeout
            
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


def save_images():
    # Get image
    image_right = get_image_right("sensor.RGB565", "sensor.QVGA")
    image_left = get_image_left("sensor.RGB565", "sensor.QVGA")
    image_right = format_image(image_right)
    image_left = format_image(image_left)

    # Save images
    cv2.imwrite("RightCam.jpg", image_right)
    cv2.imwrite("LeftCam.jpg", image_left)
    


def initRPC(portRight, portLeft):
    global interfaceRight, interfaceLeft
    interfaceRight = rpc.rpc_usb_vcp_master(portRight)
    interfaceLeft = rpc.rpc_usb_vcp_master(portLeft)


def portInformation():
    print("\nAvailable Ports:\n")

    for port, desc, hwid in serial.tools.list_ports.comports():
        print("{} : {} [{}]".format(port, desc, hwid))


if __name__ == '__main__':
   
    portRight = '/dev/port3' # default value for port
    portLeft = '/dev/port4' # default value for port

    global debug
    debug = True
    direction = "Left or right"

    if (debug): portInformation()

    # Change working directory
    rospack = rospkg.RosPack()

    path = rospack.get_path('openmv_camera')
    os.chdir(path + "/scripts/tests")

    initRPC(portRight, portLeft)
    save_images()
    