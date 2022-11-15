#!/home/robot/.virtualenvs/hand-eye/bin/python
from time import time
import rospy
from sensor_msgs.msg import Image
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import pyrealsense2 as rs
import cv2
import numpy as np

pipeline = rs.pipeline()
config = rs.config()
align = rs.align(rs.stream.color)
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.name))
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

profile = pipeline.start(config)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
# unit = depth_sensor.get_option(rs.option.depth_units)

# depth_unit = depth_sensor.get_option_range(rs.option.depth_units)
# depth_unit = depth_sensor.set_option(rs.option.depth_units, 0.001)

color_intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
colorizer = rs.colorizer(color_scheme = 2)
cam_info = [1, color_intrinsics.ppx, color_intrinsics.ppy, color_intrinsics.fx, color_intrinsics.fy, color_intrinsics.width, color_intrinsics.height, depth_scale]
def camera():
    global cam_info
    print("camera name: ", device_product_line)
    rgbPub = rospy.Publisher('rgbImage', Image, queue_size=10)
    depthPub = rospy.Publisher('depthImage', Image, queue_size=10)
    caminfoPub = rospy.Publisher('caminfo', numpy_msg(Floats), queue_size=10)
    rospy.init_node('camera', anonymous=True)

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        product_line = 'product_line:%s' %(device_product_line)
        frames = pipeline.wait_for_frames()
        frames = align.process(frames)
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        # Depth from depth_frame is in mm
        depth_image = np.asanyarray(depth_frame.get_data())
        colorized_depth_img = np.asanyarray(colorizer.colorize(depth_frame).get_data())
        # avg = np.average(depth_image)
        # print("avg:", avg)
        color_image = np.asanyarray(color_frame.get_data())        

        depth_colormap_dim = colorized_depth_img.shape
        color_colormap_dim = color_image.shape
        
        # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
            images = np.hstack((resized_color_image, colorized_depth_img))
        else:
            images = np.hstack((color_image, colorized_depth_img))
        

        rgbMsg = Image()
        rgbMsg.header.stamp = rospy.Time.now()
        rgbMsg.height = 480
        rgbMsg.width = 640
        rgbMsg.encoding = "bgr8"
        rgbMsg.is_bigendian = False
        rgbMsg.step = 3 * 640
        rgbMsg.data = color_image.tobytes()

        depthMsg = Image()
        depthMsg.header.stamp = rospy.Time.now()
        depthMsg.height = 480
        depthMsg.width = 640
        depthMsg.encoding = "mono16"
        depthMsg.is_bigendian = False
        depthMsg.step = 640
        depthMsg.data = depth_image.tobytes()

        cv2.imshow("color image", color_image)
        cv2.waitKey(3)

        # rospy.loginfo(np.shape(color_image))
        rgbPub.publish(rgbMsg)
        depthPub.publish(depthMsg)
        caminfoPub.publish(np.array(cam_info, dtype=np.float32))
        rate.sleep()

if __name__ == '__main__':
    try:
        camera()
    except rospy.ROSInterruptException:
        pass