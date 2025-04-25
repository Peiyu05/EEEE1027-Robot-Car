import cv2
import numpy as np
import os
import math
import time
import random
from picamera2 import Picamera2
from collections import deque
import RPi.GPIO as GPIO

# ---------------------------
# Motor and GPIO Setup
# ---------------------------
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

IN1 = 17
IN2 = 18
ENA = 27
IN3 = 22
IN4 = 23
ENB = 24

motor_pins = [IN1, IN2, ENA, IN3, IN4, ENB]
for pin in motor_pins:
    GPIO.setup(pin, GPIO.OUT)

pwm_a = GPIO.PWM(ENA, 1000)
pwm_b = GPIO.PWM(ENB, 1000)
pwm_a.start(0)
pwm_b.start(0)

def set_speed(left_speed, right_speed):
    pwm_a.ChangeDutyCycle(left_speed)
    pwm_b.ChangeDutyCycle(right_speed)

def stop():
    set_speed(0, 0)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

def forward(spdA, spdB):
    set_speed(spdA, spdB)
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

def backward(spdA, spdB):
    set_speed(spdA, spdB)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

# ---------------------------
# Global HSV Ranges and Priority Colors
# ---------------------------
COLOR_RANGES = {
    'red':    [(0, 150, 100), (10, 255, 255)],
    'green':  [(40, 50, 50), (80, 255, 255)],
    'blue':   [(90, 100, 100), (130, 255, 255)],
    'yellow': [(18, 100, 100), (35, 255, 255)]
}

# Choose your two priority colors
PRIORITY_COLORS = ['red', 'blue']  # Change as needed
 
# ---------------------------
# Camera Initialization
# ---------------------------
def initialize_camera():
    try:
        picam2 = Picamera2()
        config = picam2.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)})
        picam2.configure(config)
        picam2.start()
        time.sleep(2)
        return picam2
    except RuntimeError as e:
        print(f"Camera initialization failed: {e}")
        return None

picam2 = initialize_camera()
if picam2 is None:
    print("Exiting program. Camera could not be initialized.")
    exit()

# ---------------------------
# Process Frame for Black Line Detection
# ---------------------------
def process_frame(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
    inverted = cv2.bitwise_not(binary)
    kernel = np.ones((5,5), np.uint8)
    eroded = cv2.erode(inverted, kernel, iterations=1)
    final = cv2.bitwise_not(eroded)
    return final

# ---------------------------
# PID Controller
# ---------------------------
class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=320):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.last_error = 0
        self.integral = 0

    def update(self, current_value, dt):
        error = self.setpoint - current_value
        self.integral += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0
        self.last_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

pid_black = PID(2.0, 0.0, 0.2)
pid_color = PID(2.5, 0.0, 0.25)

# ---------------------------
# Color Line Detection
# ---------------------------
def detect_colored_line(frame, color_name):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower, upper = COLOR_RANGES[color_name]
    mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    return mask

# ---------------------------
# Main Loop
# ---------------------------
def follow_line_with_color_priority():
    forward(35, 35)
    time.sleep(0.1)
    global last_time
    last_time = time.time()

    try:
        while True:
            frame = picam2.capture_array()
            height, width, _ = frame.shape
            roi_color = frame[int(height * 0.6):int(height * 0.9), :]

            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time

            color_detected = False
            for color in PRIORITY_COLORS:
                color_mask = detect_colored_line(roi_color, color)
                line_pixels = np.where(color_mask > 200)[1]

                if len(line_pixels) > 0:
                    line_center = int(np.mean(line_pixels))
                    correction = pid_color.update(line_center, dt)
                    base_speed = 130
                    left_speed = max(min(base_speed - correction, 90), 0)
                    right_speed = max(min(base_speed + correction, 90), 0)
                    forward(left_speed, right_speed)

                    cv2.circle(frame, (line_center, int(height * 0.75)), 10, (255, 0, 0), -1)
                    color_detected = True
                    print(f"Following {color} line, center: {line_center}")
                    break  # Stop at first matched priority color

            if not color_detected:
                processed_frame = process_frame(frame)
                roi = processed_frame[int(height * 0.6):int(height * 0.9), :]
                line_pixels = np.where(roi < 50)[1]

                if len(line_pixels) > 0:
                    line_center = int(np.mean(line_pixels))
                    correction = pid_black.update(line_center, dt)
                    base_speed = 130
                    left_speed = max(min(base_speed - correction, 90), 0)
                    right_speed = max(min(base_speed + correction, 90), 0)
                    forward(left_speed, right_speed)
                    cv2.circle(frame, (line_center, int(height * 0.75)), 10, (0, 0, 255), -1)
                    print(f"Following black line, center: {line_center}")
                else:
                    stop()
                    print("No line detected. Reversing slightly.")
                    backward(40, 40)
                    time.sleep(0.3)
                    stop()

            cv2.imshow("Line Follow", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        print("Program stopped by user.")
    finally:
        stop()
        picam2.stop()
        cv2.destroyAllWindows()
        GPIO.cleanup()

if __name__ == "__main__":
    follow_line_with_color_priority()
