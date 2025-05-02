import gi

import rclpy.qos
import rclpy.utilities
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import cv2 as cv
import numpy as np
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


ROUTECAM_RTSP_URI = "rtsp://192.168.0.11:5005/routecam"
FRAME_WIDTH = 1920
FRAME_HEIGHT = 1080


class Routecam(Node):
    
    pipeline = None
    
    
    def __init__(self):
        super().__init__("routecam")
        qos_profile = rclpy.qos.QoSProfile(
            history=rclpy.qos.QoSHistoryPolicy.KEEP_ALL,
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
        )
        self.image_pub = self.create_publisher(Image, "routecam", qos_profile=qos_profile)
        self.bridge = CvBridge()
        
        # gstreamer pipeline
        Gst.init(None)
        routecam_pipeline = f"rtspsrc location={ROUTECAM_RTSP_URI} latency=0 ! decodebin ! queue leaky=2 ! videoconvert ! video/x-raw, width={FRAME_WIDTH}, height={FRAME_HEIGHT}, format=BGR ! appsink drop=1 name=sink"
        
        self.pipeline = Gst.parse_launch(routecam_pipeline)
        self.sink = self.pipeline.get_by_name("sink")
        self.sink.set_property("emit-signals", True)
        self.sink.set_property("sync", False)
        self.sink.connect("new-sample", self.on_new_sample)
        self.pipeline.set_state(Gst.State.PLAYING)
        self.loop = GLib.MainLoop()
        threading.Thread(target=self.loop.run, daemon=True).start()
        # self.loop.run()
        
    def on_new_sample(self, sink):
        sample = sink.emit("pull-sample")
        buf = sample.get_buffer()
        success, map_info = buf.map(Gst.MapFlags.READ)
        if not success:
            return 0
        frame = np.frombuffer(map_info.data, dtype=np.uint8)
        frame = frame.reshape((FRAME_HEIGHT, FRAME_WIDTH, 3))
        frame = cv.rotate(frame, cv.ROTATE_180)
        
        
        # start additional OpenCV preprocessing
        
        
        # convert cv frame to ros image
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
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