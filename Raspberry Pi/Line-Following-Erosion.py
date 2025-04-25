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
picam2.configure(
    picam2.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)})
)
picam2.start()
time.sleep(2)  # Camera warm-up


def tmillis():
    return time.time_ns() // 1_000_000


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

def backward(spdA, spdB):
    """Move robot forward"""
    set_speed(spdA, spdB)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)


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


# Image processing function with morphological erosion
def process_frame(frame):
    # Convert the frame to grayscale.
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

    # Apply threshold to get a binary image:
    # Original: black line = 0, white background = 255.
    _, binary = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)

    # Invert the binary image so that the black line becomes white (foreground).
    inverted = cv2.bitwise_not(binary)

    # Define a 5x5 kernel for erosion.
    kernel = np.ones((5, 5), np.uint8)

    # Apply erosion on the inverted image to thin the white region (originally the black line).
    eroded = cv2.erode(inverted, kernel, iterations=1)

    # Invert the eroded image back so that the thinned line appears black again on a white background.
    final = cv2.bitwise_not(eroded)

    return final


# Line following logic
def follow_line():
    forward(35, 35)  # Initial move
    time.sleep(0.1)
    try:
        while True:
            frame = picam2.capture_array()
            processed_frame = process_frame(frame)
            cv2.imshow("Line Detection", processed_frame)

            height, width = processed_frame.shape
            bottom_section = processed_frame[
                int(height * 0.8) : height, :
            ]  # Bottom 20%

            # Detect black pixels in the bottom section (assuming black line on white background)
            black_pixels = np.where(bottom_section == 0)[1]

            if len(black_pixels) > 0:
                line_center = int(np.mean(black_pixels))
                print(f"Line center: {line_center} (image width: {width})")

                center_threshold = (
                    width // 13
                )  # Adjusted threshold for a 1.8 cm wide line
                center = width // 2  # Center of the image

                if abs(line_center - center) < center_threshold:
                    print("Going straight - black line centered")
                    forward(30, 30)
                elif line_center > center + center_threshold:
                    print("Turning right - black line to right")
                    turn_right(40, 40)  # Reduced speed for smoother turns
                elif line_center < center - center_threshold:
                    print("Turning left - black line to left")
                    turn_left(40, 40)  # Reduced speed for smoother turns
            else:
                curr_millis = tmillis()
                print("No line detected, stopping")
                stop()
                if (tmillis() - curr_millis > 2500):
                    backward(40,40)
                    time.sleep(0.2)
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
    follow_line()
