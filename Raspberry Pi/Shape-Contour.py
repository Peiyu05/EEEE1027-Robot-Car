import cv2
import numpy as np
from picamera2 import Picamera2
import time

# Function to detect and label shapes using Canny-based contour detection
def detect_shapes_frame(frame, min_area=1000):
    # Convert to grayscale and blur
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Edge detection
    edges = cv2.Canny(blurred, 50, 150)
    # Dilate to close gaps
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    edges = cv2.dilate(edges, kernel, iterations=1)

    # Find contours on edge map
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area:
            continue  # skip small noise contours

        # Approximate contour to polygon
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
        vertices = len(approx)

        # Classify shape
        if vertices == 3:
            shape = "Triangle"
        elif vertices == 4:
            shape = "Rectangle"
        elif vertices == 5:
            shape = "Pentagon"
        elif vertices == 6:
            shape = "Hexagon"
        else:
            # Use circularity for circles vs partial circles vs other polygons
            circularity = 4 * np.pi * area / (peri * peri)
            if circularity > 0.7:
                shape = "Circle"
            elif circularity > 0.4:
                shape = "Partial Circle"
            else:
                shape = f"Polygon({vertices})"

        # Draw and annotate on full frame
        cv2.drawContours(frame, [approx], -1, (0, 255, 0), 2)
        # Compute centroid for label
        M = cv2.moments(cnt)
        if M["m00"]:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            cX, cY = approx.ravel()[0], approx.ravel()[1]

        cv2.putText(frame, shape, (cX - 40, cY - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (255, 0, 0), 2)
    return frame

# Main routine: initialize Picamera2 and process full frames
def main():
    # Initialize camera
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={'size': (640, 480), 'format': 'RGB888'})
    picam2.configure(config)
    picam2.start()

    try:
        while True:
            frame = picam2.capture_array()
            annotated = detect_shapes_frame(frame)

            cv2.imshow("Shape Detection - Full Frame", annotated)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
    finally:
        cv2.destroyAllWindows()
        picam2.close()

if __name__ == '__main__':
    main()

