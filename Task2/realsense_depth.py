# Importing Packages
import pyrealsense2 as rs
import numpy as np


class DepthCamera:
    def __init__(self,width = 1280,height = 720):
        print("[INFO] : Depth Camera Initialised")
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.depth_frame = None
        config = rs.config()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))
        found_rgb = False
        ## Checking for RGB Stream
        for s in device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            print("The demo requires Depth camera with Color sensor")
            exit(0)
        ## The Depth stream's Max Resolution is 1280X720
        ## The RGB Stream's Max Possible Resolution is 1920X1080

        ## Enabling streams at appropriate resolution
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, 30)

        # Start streaming
        self.pipeline.start(config)
        ## Initialising Frame and Depth_fram
        self.frames = None
        self.depth_frame = None

        ## Initialising Align to Align the Depth and RGB Stream .
        ## Important if both are of different resolutions
        self.align = rs.align(rs.stream.color)
        # self.depth_to_disparity = rs.disparity_transform(True)
        # self.disparity_to_depth = rs.disparity_transform(False)

    def get_frame(self):
        
        ## Waiting for frames
        self.frames = self.pipeline.wait_for_frames()
        ## Aligning Depth and RGB streams
        self.frames = self.align.process(self.frames)

        # Depth Frame
        self.depth_frame = self.frames.get_depth_frame()
        # Colour Frame
        color_frame = self.frames.get_color_frame()

        ## Uncomment to obtain depth image as a numpy array
        #depth_image = np.asanyarray(self.depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        
        ## Uncomment to colorise the depth image
       # colorized_depth = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())

        # Show the two frames together:
        #images = np.hstack((color_frame, aligned_depth_frame))
        #plt.imshow(images)

        ## If there is no colour frame, then return False
        if not color_frame:
                return False, None
            
        return True, color_image

    ## Function to get distance of camera from a point of the image which is identified by a pixel
    def get_depth(self,x,y):
        
        depth = self.depth_frame.get_distance(x,y)
        return depth
    ## Stop Reading
    def release(self):
        self.pipeline.stop()