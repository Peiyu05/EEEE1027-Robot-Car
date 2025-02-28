import cv2
import numpy as np
from picamera2 import Picamera2
import RPi.GPIO as GPIO
import time

# Set GPIO mode to BCM
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Define motor control pins
IN1 = 17  # Left Motors Forward
IN2 = 18  # Left Motors Backward
ENA = 27  # Left Motors Enable (PWM)
IN3 = 22  # Right Motors Forward
IN4 = 23  # Right Motors Backward
ENB = 24  # Right Motors Enable (PWM)

# Set up motor pins as outputs
motor_pins = [IN1, IN2, ENA, IN3, IN4, ENB]
for pin in motor_pins:
    GPIO.setup(pin, GPIO.OUT)

# Set up PWM for speed control (100Hz)
pwm_a = GPIO.PWM(ENA, 100)  # Left motors speed
pwm_b = GPIO.PWM(ENB, 100)  # Right motors speed
pwm_a.start(0)
pwm_b.start(0)

# Color range for black in HSV (slightly tightened for robustness)
lower_black = np.array([0, 0, 0])
upper_black = np.array([179, 255, 40])  # Lowered value from 56 to 40

# Initialize the Pi Camera
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)}))
picam2.start()
time.sleep(2)  # Camera warm-up

# Motor control functions
def set_speed(left_speed, right_speed):
    """Set speed for left and right motors independently (0-100)"""
    pwm_a.ChangeDutyCycle(min(max(left_speed, 0), 100))
    pwm_b.ChangeDutyCycle(min(max(right_speed, 0), 100))

def stop():
    """Stop all motors"""
    set_speed(0, 0)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

def forward(left_speed, right_speed):
    """Move robot forward"""
    set_speed(left_speed, right_speed)
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

def turn_left(left_speed, right_speed):
    """Turn robot left (left motors backward, right motors forward)"""
    set_speed(left_speed, right_speed)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

def turn_right(left_speed, right_speed):
    """Turn robot right (left motors forward, right motors backward)"""
    set_speed(left_speed, right_speed)
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

def backward(left_speed, right_speed):
    """Move robot backward"""
    set_speed(left_speed, right_speed)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

class LineDetector:
    def __init__(self):
        self.width = 640
        self.height = 480
        self.center = self.width // 2  # 320
        self.center_threshold = self.width // 4  # Increased from 80 to 160 pixels
        self.no_line_counter = 0  # To limit backward movement
        self.last_cx = None  # Track last known line position

    def detect_lines(self, frame):
        """Detect black lines and return centroid position"""
        # Preprocess frame
        hsv = cv2.cvtColor(frame.copy(), cv2.COLOR_RGB2HSV)
        blur = cv2.GaussianBlur(hsv, (7, 7), 0)  # Increased blur for noise reduction
        black_mask = cv2.inRange(blur, lower_black, upper_black)
        
        # Optional: Erode and dilate to clean up the mask
        kernel = np.ones((5, 5), np.uint8)
        black_mask = cv2.erode(black_mask, kernel, iterations=1)
        black_mask = cv2.dilate(black_mask, kernel, iterations=1)

        # Find contours
        black_cnts, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return self.contour_detection(black_cnts, frame, "Black")

    def contour_detection(self, contours, frame, color_name):
        """Process contours and control robot movement"""
        if contours:
            # Sort contours by area and take the largest (most reliable) one
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if 300 < area < 100000:  # Lowered min area from 500 to 300
                M = cv2.moments(c)
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                # Visualize
                cv2.line(frame, (cx, 0), (cx, self.height), (255, 0, 0), 1)
                cv2.line(frame, (0, cy), (self.width, cy), (255, 0, 0), 1)
                cv2.circle(frame, (cx, cy), 5, (255, 255, 255), -1)
                cv2.drawContours(frame, [c], -1, (0, 255, 0), 2)

                # Control logic with wider threshold
                if abs(cx - self.center) < self.center_threshold:
                    print(f"{color_name} line centered - going straight")
                    forward(25, 25)
                    self.no_line_counter = 0
                elif cx > self.center + self.center_threshold:
                    print(f"{color_name} line too far right - turning left")
                    turn_left(50, 50)  # Slightly reduced turn speed
                    self.no_line_counter = 0
                elif cx < self.center - self.center_threshold:
                    print(f"{color_name} line too far left - turning right")
                    turn_right(50, 50)  # Slightly reduced turn speed
                    self.no_line_counter = 0

                self.last_cx = cx  # Update last known position
                return cx

        # No line detected
        self.no_line_counter += 1
        if self.no_line_counter <= 10:  # Limit backward to ~1 second (at 10 FPS)
            print(f"No {color_name} line detected - moving backward briefly")
            backward(30, 30)  # Reduced speed, symmetrical
        else:
            print(f"No {color_name} line detected - stopping")
            stop()  # Stop after brief backward to wait or recover
        return None

    def run_detection(self):
        """Main loop for line detection and following"""
        forward(35, 35)  # Initial move
        time.sleep(0.2)
        try:
            while True:
                frame = picam2.capture_array()
                self.detect_lines(frame)
                cv2.imshow("Line Detection", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        except KeyboardInterrupt:
            print("Program stopped by user")
        finally:
            stop()
            picam2.stop()
            cv2.destroyAllWindows()
            GPIO.cleanup()

if __name__ == "__main__":
    detector = LineDetector()
    detector.run_detection()