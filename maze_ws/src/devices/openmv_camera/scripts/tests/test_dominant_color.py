# Turn on led depending on the detected color. Test with openmv ide
#
# This example shows off multi color blob tracking using the OpenMV Cam.

import sensor, image, time, math, pyb


# Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
# Calibrate using Tools > Machine Vision > Threshold Editor

# Minimums and maximums for the LAB L, A, and B channels respectively.
thresholds = [(27, 68, 33, 70, -1, 54), # red_thresholds
              (32, 66, -57, -25, 4, 40), # green_thresholds
              (30, 62, -19, 0, 20, 60)] # yellow_thresholds

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
# Flip the image 180 degrees
sensor.set_vflip(True)
sensor.set_hmirror(True)

clock = time.clock()

# Init leds

red_led = pyb.LED(1)
green_led = pyb.LED(2)
blue_led = pyb.LED(3)

# Only blobs that with more pixels than "pixel_threshold" and more area than "area_threshold" are
# returned by "find_blobs" below. Change "pixels_threshold" and "area_threshold" if you change the
# camera resolution. Don't set "merge=True" becuase that will merge blobs which we don't want here.

def detect_dominant_color():
    img = sensor.snapshot()
    max = 'X'
    max_area = 0
    for blob in img.find_blobs(thresholds, pixels_threshold=200, area_threshold=200):
        # Check conditions for discarting blobs. Try with blob.elongation and blob.convexity()
        # once real targets are available
        if blob.elongation() > 0.5:
            pass
        
        if blob.code() == 1:
            img.draw_edges(blob.min_corners(), color=(255, 0, 0))
        if blob.code() == 2:
            img.draw_edges(blob.min_corners(), color=(0, 255, 0))
        if blob.code() == 4:
            img.draw_edges(blob.min_corners(), color=(255, 255, 0))
        

        if blob.area() < max_area:
            continue
        
        max_area = blob.area()

        if blob.code() == 1:
            max = 'r'
        if blob.code() == 2:
            max = 'g'
        if blob.code() == 4:
            max = 'y'

    return max

def turn_led(detection):
    if detection == 'r':
        red_led.on()
        blue_led.off()
        green_led.off()
    elif detection == 'g':
        red_led.off()
        blue_led.off()
        green_led.on()
    elif detection == 'y':
        red_led.on()
        blue_led.off()
        green_led.on()
    else:
        red_led.off()
        blue_led.off()
        green_led.off()

while(True):
    clock.tick()
    detected = detect_dominant_color()
    turn_led(detected)
    time.sleep_ms(100)
