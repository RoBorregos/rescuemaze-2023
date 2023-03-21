# Script to load in openmv as rpc slave.

# Original script from: https://github.com/openmv/openmv/blob/master/scripts/examples/08-RPC-Library/34-Remote-Control/image_transfer_jpg_as_the_remote_device_for_your_computer.py

import image, network, omv, rpc, sensor, struct

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
sensor.set_vflip(True)
sensor.set_hmirror(True)

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
    
# Callback for detecting colors
# TODO: implement function.
def detect_dominant_color(data):
    res = 'X'
    return str(res).encode()

# Register call backs.
interface.register_callback(jpeg_image_snapshot)
interface.register_callback(jpeg_image_read)
interface.register_callback(detect_dominant_color)

# Once all call backs have been registered we can start
# processing remote events. interface.loop() does not return.

interface.loop()
