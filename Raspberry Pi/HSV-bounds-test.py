import cv2
import numpy as np
from picamera2 import Picamera2, Preview

# -------------------------------------------------------------------
# 1. Define your HSV bounds here (Hue: 0–179, Sat/Val: 0–255)
# -------------------------------------------------------------------
hsv_bounds = {
    "Red1":   {"lower": (0,   150, 100), "upper": (10,  255, 255)},
    "Red2":   {"lower": (170, 150, 100), "upper": (180, 255, 255)},
    "Green":  {"lower": (50,  80,  60),  "upper": (90,  255, 255)},
    "Blue":   {"lower": (100, 70,  40),  "upper": (140, 255, 255)},
    "Yellow": {"lower": (18,  100, 100), "upper": (35,  255, 255)},
}

# -------------------------------------------------------------------
# 2. Initialize Picamera2 and start the preview (RGB888)
# -------------------------------------------------------------------
picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"format": "RGB888", "size": (640, 480)}
)
picam2.configure(config)
picam2.start()

print("Camera started. Press 'q' to quit.")

# -------------------------------------------------------------------
# 3. Main loop: grab frames, convert, mask, and display
# -------------------------------------------------------------------
try:
    while True:
        # 3.1 Capture an RGB888 frame (shape: H×W×3)
        frame = picam2.capture_array()

        # 3.2 Convert directly from BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 3.3 Build masks for each colour range
        masks = []
        for name, bounds in hsv_bounds.items():
            lower = np.array(bounds["lower"], dtype=np.uint8)
            upper = np.array(bounds["upper"], dtype=np.uint8)
            mask = cv2.inRange(hsv, lower, upper)
            masks.append((name, mask))

        # 3.4 Stack masks side by side for visual comparison
        mask_bgr = [cv2.cvtColor(m, cv2.COLOR_GRAY2BGR) for _, m in masks]
        combined = cv2.hconcat(mask_bgr)

        # 3.5 Annotate each sub-window with its colour name
        w = combined.shape[1] // len(masks)
        for i, (name, _) in enumerate(masks):
            cv2.putText(combined, name, (i*w + 10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

        # 3.6 Display original frame and combined masks
        cv2.imshow("Frame (RGB888)", frame)
        cv2.imshow("HSV Masks", combined)

        # 3.7 Exit on 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    picam2.stop()
    cv2.destroyAllWindows()
