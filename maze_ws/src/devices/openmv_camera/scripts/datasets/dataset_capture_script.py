# Dataset Capture Script - By: oscar - mié. mar 22 2023
# Runs on openmv

# Use this script to control how your OpenMV Cam captures images for your dataset.
# You should apply the same image pre-processing steps you expect to run on images
# that you will feed to your model during run-time.
# Script to load in openmv as rpc slave.

import image, network, omv, rpc, sensor, struct, pyb


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)

sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking

red_led = pyb.LED(1)
green_led = pyb.LED(2)
blue_led = pyb.LED(3)

red_led.on()
blue_led.on()
green_led.on()

# Flip the image 180 degrees
# sensor.set_vflip(True)
# sensor.set_hmirror(True)

omv.disable_fb(True)

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
    
# Register call backs.
interface.register_callback(jpeg_image_snapshot)
interface.register_callback(jpeg_image_read)

# Once all call backs have been registered we can start
# processing remote events. interface.loop() does not return.
interface.loop()
