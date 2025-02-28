import cv2
import numpy as np
from picamera2 import Picamera2
import time

# Initialize the Pi Camera
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)}))
picam2.start()
time.sleep(2)  # Allow camera to warm up

def process_frame(frame):
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    
    # Apply binary threshold (black lines on white background)
    # Adjust the threshold value (150) based on your lighting conditions
    _, binary = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)
    
    # Optional: Invert if your line is white on black background
    # binary = cv2.bitwise_not(binary)
    
    return binary

try:
    while True:
        # Capture frame
        frame = picam2.capture_array()
        
        # Process the frame
        processed_frame = process_frame(frame)
        
        # Display the result
        cv2.imshow('Line Detection', processed_frame)
        
        # Calculate simple line position (for robot control)
        # Get the bottom portion of the image
        height, width = processed_frame.shape
        bottom_section = processed_frame[int(height*0.8):height, :]
        
        # Find black pixels (value 0)
        black_pixels = np.where(bottom_section == 0)[1]
        
        if len(black_pixels) > 0:
            # Calculate center of black pixels
            line_center = int(np.mean(black_pixels))
            # Print position (you can use this to control your robot)
            print(f"Line center: {line_center} (image width: {width})")
        else:
            print("No line detected")
        
        # Exit on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Program stopped by user")

finally:
    # Cleanup
    picam2.stop()
    cv2.destroyAllWindows()
