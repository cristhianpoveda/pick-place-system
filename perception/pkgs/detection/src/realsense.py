import cv2
import sys
from ultralytics import YOLO
import pyrealsense2 as rs
import numpy as np

model = YOLO('best.pt')
w = 640
h = 480

pipeline = rs.pipeline()
config = rs.config()

pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

config.enable_stream(rs.stream.depth, w, h, rs.format.z16, 30)
config.enable_stream(rs.stream.color, w, h, rs.format.bgr8, 30)

pipeline.start(config)

size = (w, h)

if len(sys.argv) == 1:
    video_name='video.avi'
else:
    name=sys.argv[1]
    video_name=name+'.avi'

video = cv2.VideoWriter(video_name, cv2.VideoWriter_fourcc(*'MJPG'), 12, size)

try:
    while True:

        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            print('No image received')
            continue
        
        color_image = np.asanyarray(color_frame.get_data())

        # INFERENCE
        detections = model(color_image, conf=0.7)
        annotated_frame = detections[0].plot()

        video.write(annotated_frame)
        if cv2.waitKey(1) == ord('q'):
            break

finally:
    pipeline.stop()
    video.release()