import cv2
import numpy as np
from picamera2 import Picamera2
import time

# Initialize the Pi Camera
try:
    picam2 = Picamera2()
    camera_info = picam2.global_camera_info()
    if not camera_info:
        raise RuntimeError("No cameras detected!")
    print("Camera detected:", camera_info)
    
    # Configure camera for color output
    picam2.configure(picam2.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)}))
    picam2.start()
    time.sleep(2)  # Allow camera to warm up
    
except Exception as e:
    print(f"Error initializing camera: {e}")
    exit(1)

def detect_black_line(frame):
    # Convert to grayscale for detection
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    
    # Apply binary threshold to detect dark areas (black line)
    _, binary = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
    
    # Analyze bottom section of the image
    height, width = binary.shape
    bottom_section = binary[int(height * 0.8):height, :]
    
    # Find contours of black areas
    contours, _ = cv2.findContours(bottom_section, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    return contours, int(height * 0.8)

try:
    while True:
        # Capture frame in original color
        frame = picam2.capture_array()
        
        # Detect black line contours
        contours, y_offset = detect_black_line(frame)
        
        # Copy frame for display
        display_frame = frame.copy()
        
        # Process detected contours
        if contours:
            for contour in contours:
                # Get bounding rectangle for each black line segment
                x, y, w, h = cv2.boundingRect(contour)
                # Adjust y coordinate to match original frame
                y += y_offset
                
                # Draw rectangle around black line
                cv2.rectangle(display_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                
                # Add text label
                cv2.putText(display_frame, "Black Line", (x, y - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Add status text at top
            cv2.putText(display_frame, "Black Line Detected", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            # No line detected
            cv2.putText(display_frame, "No Black Line Detected", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        # Display the frame with annotations
        cv2.imshow('Color Feed with Line Detection', display_frame)
        
        # Exit on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Program stopped by user")

finally:
    # Cleanup
    picam2.stop()
    cv2.destroyAllWindows()
