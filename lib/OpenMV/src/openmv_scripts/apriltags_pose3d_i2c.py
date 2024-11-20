# AprilTags Pixy I2C Emulation Script
#
# This script allows your OpenMV Cam to transmit AprilTag detection data like
# a Pixy (CMUcam5) tracking colors in I2C mode. This script allows you to
# easily replace a Pixy (CMUcam5) color tracking sensor with an OpenMV Cam
# AprilTag tracking sensor. Note that this only runs on the OpenMV Cam M7.
#
# P4 = SCL
# P5 = SDA
#
# Note: The tag family is TAG36H11. Additionally, in order to for the
#       signature value of a tag detection to be compatible with pixy
#       interface libraries all tag ids have 8 added to them in order
#       to move them in the color code signature range. Finally, tags
#       are all reported as color code blocks...

# Comm Parameters ############################################################

max_blocks = 255
max_blocks_per_id = 255

i2c_address = 0x12

##############################################################################

###
# Changed to using rpc.put_bytes, which has a little more control
# Plan is to make it report back the number
###

import image, math, sensor, rpc, struct, time

# Camera Setup

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(True)  # must turn this off to prevent image washout...
sensor.set_auto_whitebal(True)  # must turn this off to prevent image washout...
clock = time.clock()

# Link Setup

#bus = pyb.I2C(2, pyb.I2C.SLAVE, addr = i2c_address)
interface = rpc.rpc_i2c_slave(slave_addr=i2c_address)

# Helper Stuff

def checksum(data):
    checksum = 0
    for i in range(0, len(data), 2):
        checksum += ((data[i+1] & 0xFF) << 8) | ((data[i+0] & 0xFF) << 0)
    return checksum & 0xFFFF

def convToInt(data):
    data = int(data)
    return data

def to_object_block_format(tag):
    pose_data = struct.pack(
        "hhhhhhhhhhh",
        tag.id,
        tag.w,
        tag.h,
        convToInt(tag.rotation * 1000),
        convToInt(tag.x_translation * 1000),
        convToInt(tag.y_translation * 1000),
        convToInt(tag.z_translation * 1000),
        convToInt(tag.x_rotation * 1000),
        convToInt(tag.y_rotation * 1000),
        convToInt(tag.z_rotation * 1000),
        0x00 # we need an extra byte in order to calculate the checksum
    )

    frame_checksum = checksum(pose_data)
    # pack the struct as: <header 2 bytes> <checksum of data 2 bytes> <combined data 11 bytes>

    out = struct.pack("<hh22s", 0xAA55, frame_checksum, pose_data)

    return out

def degrees(radians):
    return (180 * radians) / math.pi

# Main Loop
# c_x is the image x center position in pixels.
# c_y is the image y center position in pixels.

f_x = (2.8 / 3.984) * 160  # find_apriltags defaults to this if not set
f_y = (2.8 / 2.952) * 120  # find_apriltags defaults to this if not set
c_x = 160 * 0.5  # find_apriltags defaults to this if not set (the image.w * 0.5)
c_y = 120 * 0.5  # find_apriltags defaults to this if not set (the image.h * 0.5)

while(True):
    clock.tick()
    img = sensor.snapshot()
    tags = img.find_apriltags(families=image.TAG36H11, fx=f_x, fy=f_y, cx=c_x, cy=c_y) # explicitly define what tag fmamily and pose variables

    # Transmit Tags #

    dat_buf = struct.pack("<b", len(tags)) # Transmit if there are tags to the master
    interface.put_bytes(dat_buf, timeout_ms = 200)

    if tags and (max_blocks > 0) and (max_blocks_per_id > 0): # new frame
    #for tag in img.find_apriltags(fx=f_x, fy=f_y, cx=c_x, cy=c_y):
        dat_buf = struct.pack("<h", 0xAA55)

        # sort by proximity
        for tag in sorted(tags, key = lambda x: abs(x.z_translation), reverse = True)[0:max_blocks]:
            img.draw_rectangle(tag.rect, color=(255, 0, 0))
            img.draw_cross(tag.cx, tag.cy, color=(0, 255, 0))

            dat_buf = to_object_block_format(tag) # package the tag for i2c
            interface.put_bytes(dat_buf, timeout_ms = 200) # send frame # 200

            print_args = ( # debug print information
                tag.id,
                tag.w,
                tag.h,
                convToInt(tag.rotation * 1000),
                convToInt(tag.x_translation * 1000),
                convToInt(tag.y_translation * 1000),
                convToInt(tag.z_translation * 1000),
                convToInt(tag.x_rotation * 1000),
                convToInt(tag.y_rotation * 1000),
                convToInt(tag.z_rotation * 1000),
            )
            print("id: %d, w %d, h %d, R %d, X %d, Y %d, Z %d, Rx %d, Ry %d, Rz %d" % print_args)

            num_tags = min(len(tags), max_blocks)


    num_tags = min(len(tags), max_blocks)
    print("%d tags(s) found - FPS %f" % (num_tags, clock.fps()))
