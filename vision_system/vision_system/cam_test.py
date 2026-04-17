import cv2
from pupil_apriltags import Detector

# ---------------------------
# Camera pipeline (your working setup)
# ---------------------------
pipeline = (
    "nvarguscamerasrc sensor-id=1 ! "
    "video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1 ! "
    "nvvidconv ! video/x-raw, format=BGRx ! "
    "videoconvert ! video/x-raw, format=BGR ! appsink drop=1 max-buffers=1"
)

cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Error: camera not opened")
    exit()

# ---------------------------
# AprilTag detector
# ---------------------------
detector = Detector(
    families="tag36h11",   # most common family
    nthreads=2,
    quad_decimate=2.0,     # speed vs accuracy tradeoff
    quad_sigma=0.0,
    refine_edges=1
)

# ---------------------------
# Main loop
# ---------------------------
while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    tags = detector.detect(
        gray,
        estimate_tag_pose=False,
        camera_params=None,
        tag_size=None
    )

    # Draw detections
    for tag in tags:
        corners = tag.corners.astype(int)

        # Draw polygon outline
        for i in range(4):
            pt1 = tuple(corners[i])
            pt2 = tuple(corners[(i + 1) % 4])
            cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

        # Center point
        cX, cY = int(tag.center[0]), int(tag.center[1])
        cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)

        # Tag ID label
        cv2.putText(
            frame,
            f"ID: {tag.tag_id}",
            (cX - 20, cY - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 0, 0),
            2
        )

    cv2.imshow("AprilTag Detection", frame)

    if cv2.waitKey(1) == 27:  # ESC to quit
        break

cap.release()
cv2.destroyAllWindows()