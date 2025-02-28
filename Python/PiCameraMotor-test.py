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

# Initialize the Pi Camera
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)}))
picam2.start()
time.sleep(2)  # Camera warm-up

# Motor control functions
def set_speed(left_speed, right_speed):
    """Set speed for left and right motors independently (0-100)"""
    pwm_a.ChangeDutyCycle(left_speed)
    pwm_b.ChangeDutyCycle(right_speed)

def stop():
    """Stop all motors"""
    set_speed(0, 0)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

def forward(spdA, spdB):
    """Move robot forward"""
    set_speed(spdA, spdB)
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

def turn_left(spdA, spdB):
    """Turn robot left (left motors backward, right motors forward)"""
    set_speed(spdA, spdB)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

def turn_right(spdA, spdB):
    """Turn robot right (left motors forward, right motors backward)"""
    set_speed(spdA, spdB)
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

# Image processing function
def process_frame(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    _, binary = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)  # Black = 0, white = 255
    return binary

# Line following logic
def follow_line():
    forward(35, 35)  # Initial move
    time.sleep(0.2)
    try:
        while True:
            frame = picam2.capture_array()
            processed_frame = process_frame(frame)
            cv2.imshow('Line Detection', processed_frame)
            
            height, width = processed_frame.shape
            bottom_section = processed_frame[int(height * 0.8):height, :]  # Bottom 20%
            
            black_pixels = np.where(bottom_section == 0)[1]  # Detect black line (fixed to bottom_section)
            
            if len(black_pixels) > 0:
                line_center = int(np.mean(black_pixels))
                print(f"Line center: {line_center} (image width: {width})")
                
                center_threshold = width // 8  # ~80 pixels, tighter for 3 cm line
                center = width // 2  # 320

                if abs(line_center - center) < center_threshold:
                    print("Going straight - black line centered")
                    forward(25, 25)
                elif line_center > center + center_threshold:
                    print("Turning right - black line to right")
                    turn_right(60, 60)  # Reduced speed for smoother turns
                elif line_center < center - center_threshold:
                    print("Turning left - black line to left")
                    turn_left(60, 60)  # Reduced speed for smoother turns
            else:
                print("No line detected, stopping")
                stop()
            
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
    follow_line()