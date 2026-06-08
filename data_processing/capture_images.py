import cv2
import os
import numpy as np

FRAME_OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../output_data/frame_data")
FRAME_FILENAME = "frames.txt"
VIDEO_OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../output_data/video")

frame_output_path = os.path.join(FRAME_OUTPUT_DIR, FRAME_FILENAME)
video_output_path = os.path.join(VIDEO_OUTPUT_DIR, 'output.mp4')

# init camera
cap = cv2.VideoCapture(0)
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = 30.0 # Set a default FPS

# Define codec and VideoWriter object (uses 'mp4v' for MP4)
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
video_output = cv2.VideoWriter(video_output_path, fourcc, fps, (frame_width, frame_height))

with open(frame_output_path, "w", encoding="utf-8") as file:

    # Inf loop of camera loading frames to file while also displaying 
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("Failed to read frames")
            break

        # Live camera view
        cv2.imshow('Camera Stream', frame)

        # save frame in txt
        file.write(np.array_str(frame))

        # add frame to video
        video_output.write(frame)

        # press 'q' to break loop
        if cv2.waitKey(1) & 0xFF == ord('q'): break

cap.release()
video_output.release()
cv2.destroyAllWindows()

