# Script to load in openmv as rpc slave.

# TODO: tune conditions for discarding blobs (elongation and convexity).

# Original script from: https://github.com/openmv/openmv/blob/master/scripts/examples/08-RPC-Library/34-Remote-Control/image_transfer_jpg_as_the_remote_device_for_your_computer.py

import image, network, omv, rpc, sensor, struct, pyb

# Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
# Calibrate using Tools > Machine Vision > Threshold Editor


# Minimums and maximums for the LAB L, A, and B channels respectively.
thresholds = [(2, 100, 42, 89, -83, 51), # red_thresholds
              (32, 83, -57, -14, -25, 52), # green_thresholds
              (31, 80, -11, 42, 17, 69)] # yellow_thresholds

"""Past thresholds  

thresholds = [(11, 31, 19, 53, 11, 64), # red_thresholds
              (32, 52, -55, -11, -27, 35), # green_thresholds
              (50, 75, -33, 24, 49, 91)] # yellow_thresholds

thresholds = [(27, 68, 33, 70, -1, 54), # red_thresholds
              (32, 66, -57, -25, 4, 40), # green_thresholds
              (30, 62, -19, 0, 20, 60)] # yellow_thresholds
"""

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)

sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking

# Flip the image 180 degrees
#sensor.set_vflip(True)
#sensor.set_hmirror(True)

omv.disable_fb(True)

red_led = pyb.LED(1)
green_led = pyb.LED(2)
blue_led = pyb.LED(3)

red_led.on()
blue_led.on()
green_led.on()

interface = rpc.rpc_usb_vcp_slave()

# Call Backs

# Callbacks for returning jpg image

# Returns the frame buffer jpg size to store the image in.
def jpeg_image_snapshot(data):
    pixformat, framesize = bytes(data).decode().split(",")
    sensor.set_pixformat(eval(pixformat))
    sensor.set_framesize(eval(framesize))
    img = sensor.snapshot().compress(quality=90)
    return struct.pack("<I", img.size())

def jpeg_image_read_cb():
    interface.put_bytes(sensor.get_fb().bytearray(), 5000) # timeout

# Read data from buffer
def jpeg_image_read(data):
    interface.schedule_callback(jpeg_image_read_cb)
    return bytes()
    
# Callback for detecting colors. Returns the initial representing the color with greatest blob area.
def detect_dominant_color(data):
    sensor.set_auto_gain(False) # must be turned off for color tracking
    sensor.set_auto_whitebal(False) # must be turned off for color tracking
    img = sensor.snapshot()
    max = 'X'
    max_area = 0
    for blob in img.find_blobs(thresholds, pixels_threshold=200, area_threshold=200):
        # Check conditions for discarting blobs. Try with blob.elongation and blob.convexity()
        # once real targets are available
        if blob.elongation() > 0.5:
            pass

        if blob.area() < max_area:
            continue
        
        max_area = blob.area()

        if blob.code() == 1:
            max = 'r'
        if blob.code() == 2:
            max = 'g'
        if blob.code() == 4:
            max = 'y'

    return str(max).encode()

# Register call backs.
interface.register_callback(jpeg_image_snapshot)
interface.register_callback(jpeg_image_read)
interface.register_callback(detect_dominant_color)

# Once all call backs have been registered we can start
# processing remote events. interface.loop() does not return.

interface.loop()
