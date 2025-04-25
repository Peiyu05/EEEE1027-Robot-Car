import cv2
import numpy as np
from picamera2 import Picamera2
import time

# Load templates for each direction
template_paths = {
    'Up': '/home/pi/new-ref-4/up.png',
    'Down': '/home/pi/new-ref-4/down.png',
    'Left': '/home/pi/new-ref-4/left.png',
    'Right': '/home/pi/new-ref-4/right.png'
}

templates = {}
for direction, path in template_paths.items():
    template = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if template is None:
        print(f"[ERROR] Could not load template for {direction} from {path}")
    else:
        templates[direction] = template

# Function to perform matching and label the best direction
def detect_and_label(frame, templates, threshold=0.85):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    result_frame = frame.copy()

    best_direction = None
    best_score = threshold  # Start with threshold as the minimum acceptable score
    best_location = None
    best_template = None

    for direction, template in templates.items():
        h, w = template.shape[:2]
        res = cv2.matchTemplate(gray, template, cv2.TM_CCOEFF_NORMED)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

        # Update if we find a higher match score
        if max_val > best_score:
            best_score = max_val
            best_direction = direction
            best_location = max_loc
            best_template = template

    # Only draw the best match if it's above the threshold
    if best_direction is not None and best_score >= threshold:
        x, y = best_location
        h, w = best_template.shape[:2]
        cv2.rectangle(result_frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
        cv2.putText(result_frame, best_direction, (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    return result_frame

# Initialize Pi Camera
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"})
picam2.configure(config)
picam2.start()
time.sleep(2)  # Warm-up time

try:
    while True:
        frame = picam2.capture_array()
        annotated = detect_and_label(frame, templates, threshold=0.7)
        cv2.imshow("Arrow Direction Detection", annotated)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    cv2.destroyAllWindows()
    picam2.close()
