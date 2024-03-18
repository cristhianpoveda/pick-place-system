import cv2
import sys
from ultralytics import YOLO

model = YOLO('best.pt')

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

frame_width = int(cap.get(3)) 
frame_height = int(cap.get(4))
size = (frame_width, frame_height) 

if len(sys.argv) == 1:
    video_name='video.avi'
else:
    name=sys.argv[1]
    video_name=name+'.avi'

video = cv2.VideoWriter(video_name, cv2.VideoWriter_fourcc(*'MJPG'), 12, size)

while True:

    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    
    # INFERENCE
    detections = model(frame, conf=0.7)
    keypoints = detections[0].keypoints.xy
    print(len(keypoints))
    base_x = int(keypoints[0][0][0])
    base_y = int(keypoints[0][0][1])
    thread_x = int(keypoints[0][1][0])
    thread_y = int(keypoints[0][1][1])
    print(base_x, base_y, thread_x, thread_y)
    annotated_frame = detections[0].plot()

    video.write(annotated_frame)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
video.release()