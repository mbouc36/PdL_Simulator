import cv2
import time

# Initialize webcam (0 is typically the default built-in camera)
cap = cv2.VideoCapture(0)

# Initialize variables to track time
prev_frame_time = 0
new_frame_time = 0

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Get the time after reading the current frame
    new_frame_time = time.time()

    # Calculate FPS (1 divided by time elapsed between frames)
    # Avoid division by zero by ensuring a positive delta
    time_delta = new_frame_time - prev_frame_time
    fps = 1 / time_delta if time_delta > 0 else 0
    prev_frame_time = new_frame_time

    # Format FPS to an integer string
    fps_text = f"FPS: {int(fps)}"

    # Overlay the FPS text onto the live video frame
    cv2.putText(frame, fps_text, (7, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (100, 255, 0), 2, cv2.LINE_AA)

    # Display the frame
    cv2.imshow('Camera FPS Stream', frame)

    # Press 'q' on the keyboard to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
