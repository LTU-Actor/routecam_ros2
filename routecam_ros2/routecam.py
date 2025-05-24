import gi

import rclpy.qos
import time
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import cv2 as cv
import numpy as np
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
STREAM_INITIAL_TIMEOUT = 10         # time to wait before determining the stream cannot be found


class Routecam(Node):
    
    pipeline = None
    
    
    def __init__(self):
        super().__init__("routecam")
        qos_profile = rclpy.qos.QoSProfile(
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
            depth=5,
        )
        self.image_pub = self.create_publisher(Image, "routecam/image_raw", qos_profile=qos_profile)
        self.info_pub = self.create_publisher(CameraInfo, "routecam/camera_info", 1)
        self.bridge = CvBridge()
        
        self.declare_parameter("hostname", "")
        hostname = self.get_parameter("hostname")
        self.rtsp_uri = f"rtsp://{hostname.value}:5005/routecam"
        
        # gstreamer pipeline
        self.active = False
        Gst.init(None)
        routecam_pipeline = f"rtspsrc location={self.rtsp_uri} latency=0 ! rtpjitterbuffer ! decodebin ! queue leaky=2 ! videoconvert ! video/x-raw, width={FRAME_WIDTH}, height={FRAME_HEIGHT}, format=BGR ! appsink drop=true name=sink"
        
        self.pipeline = Gst.parse_launch(routecam_pipeline)
        self.sink = self.pipeline.get_by_name("sink")
        self.sink.set_property("emit-signals", True)
        self.sink.set_property("sync", False)
        self.sink.connect("new-sample", self.on_new_sample)
        self.pipeline.set_state(Gst.State.PLAYING)
        
        self.info = CameraInfo()
        self.info.width = FRAME_WIDTH
        self.info.height = FRAME_HEIGHT
        self.info.binning_x = 0
        self.info.binning_y = 0
        self.info.header.frame_id = "route_cam_link" 
        self.info.distortion_model = "rational_polynomial"
        self.info.d = [0.006529257187577666, -0.13277793184289408, 0.1680380154205031, -0.0775994795123907]
        self.info.k = [594.619643383718, -2.338996198816343, 680.1318100627273, 0.0, 593.0999623886435, 371.09535258839816, 0.0, 0.0, 1.0]
        self.info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.info.p = [594.619643383718, -2.338996198816343, 680.1318100627273, 0.0, 0.0, 593.0999623886435, 371.09535258839816, 0.0, 0.0, 0.0, 1.0, 0.0]
        
        
        self.loop = GLib.MainLoop()
        threading.Thread(target=self.loop.run, daemon=True).start()
        
        time.sleep(STREAM_INITIAL_TIMEOUT)
        
        if not self.active:
            self.get_logger().log(f"Could not connect to {self.rtsp_uri}, stream timed out after {STREAM_INITIAL_TIMEOUT} seconds", 40)
            self.destroy_node()
            exit(-1)
        
        
    def on_new_sample(self, sink):
        sample = sink.emit("pull-sample")
        buf = sample.get_buffer()
        success, map_info = buf.map(Gst.MapFlags.READ)
        if not success:
            self.get_logger().log(f"Unable to grab frame from {self.rtsp_uri}", 40)
            self.loop.quit()
            self.destroy_node()
            return 0
        if not self.active:
            self.get_logger().log(f"Stream has started from {self.rtsp_uri}", 20)
            self.active = True
        frame = np.frombuffer(map_info.data, dtype=np.uint8)
        frame = frame.reshape((FRAME_HEIGHT, FRAME_WIDTH, 3))
        frame = cv.rotate(frame, cv.ROTATE_180)
        
        
        # start additional OpenCV preprocessing
        
        
        # convert cv frame to ros image
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
        self.info_pub.publish(self.info)
        buf.unmap(map_info)
        return 0
  
  
        
def main(args=None):
    rclpy.init(args=args)
    node = Routecam()
    
    rclpy.spin(node)
    node.destroy_node()
    node.loop.quit()
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()