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

# Define motor control pins
IN1 = 17   # Left Motors Forward
IN2 = 18   # Left Motors Backward
ENA = 27   # Left Motors Enable (PWM)
IN3 = 22   # Right Motors Forward
IN4 = 23   # Right Motors Backward
ENB = 24   # Right Motors Enable (PWM)

motor_pins = [IN1, IN2, ENA, IN3, IN4, ENB]
for pin in motor_pins:
    GPIO.setup(pin, GPIO.OUT)

pwm_a = GPIO.PWM(ENA, 1000)  # 100Hz for left motors
pwm_b = GPIO.PWM(ENB, 1000)  # 100Hz for right motors
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
# Camera Initialization
# ---------------------------
def initialize_camera():
    try:
        picam2 = Picamera2()
        config = picam2.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)})
        picam2.configure(config)
        picam2.start()
        time.sleep(2)  # warm-up
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
    """
    Process the frame to detect the black line.
    Converts to grayscale, thresholds, inverts, erodes, and inverts back.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    # Use a lower threshold (90) to better capture the black line
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

pid_controller = PID(Kp=2.0, Ki=0.0, Kd=0.2, setpoint=320)

# ---------------------------
# Main Loop: Line Following with PID Only
# ---------------------------
prev_detections = deque()
last_time = time.time()

def follow_line_with_pid():
    forward(35, 35)
    time.sleep(0.1)
    global last_time
    line_center = 320  # default value

    try:
        while True:
            frame = picam2.capture_array()  # capture original color frame

            # --- Line Detection ---
            processed_frame = process_frame(frame)
            #cv2.imshow("Processed Line Detection", processed_frame)

            height, width = processed_frame.shape
            roi = processed_frame[int(height * 0.6):int(height * 0.9), :]
            # For a black line on white background, assume pixels with value < 50 are part of the line.
            line_pixels = np.where(roi < 50)[1]

            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time

            if len(line_pixels) > 0:
                line_center = int(np.mean(line_pixels))
                correction = pid_controller.update(line_center, dt)
                base_speed = 130
                left_speed = max(min(base_speed - correction, 90), 0)
                right_speed = max(min(base_speed + correction, 90), 0)
                forward(left_speed, right_speed)
                # Overlay detected line center on the live color frame.
                center_y = int(height * 0.75)
                cv2.circle(frame, (line_center, center_y), 10, (0,0,255), -1)
                cv2.imshow("Line Follow", frame)
                print(f"Line center: {line_center}, Correction: {correction}")
            else:
                stop()
                img_center = width // 2
                bck_dir = int(math.copysign(1, (line_center - img_center)))
                spda = 40
                spdb = 40
                if bck_dir == 1:
                    spdb += 20
                if bck_dir == -1:
                    spda += 20
                backward(spda, spdb)
                time.sleep(0.3)
                stop()

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        print("Program stopped by user")
    finally:
        stop()
        picam2.stop()
        cv2.destroyAllWindows()
        GPIO.cleanup()

if __name__ == "__main__":
    follow_line_with_pid()
