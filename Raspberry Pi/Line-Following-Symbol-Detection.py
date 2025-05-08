import cv2
import numpy as np
import time
from picamera2 import Picamera2
from tflite_runtime.interpreter import Interpreter
import RPi.GPIO as GPIO

# ---------------------------
# Motor and GPIO Setup
# ---------------------------
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
IN1, IN2, ENA = 17, 18, 27
IN3, IN4, ENB = 22, 23, 24
for pin in [IN1, IN2, ENA, IN3, IN4, ENB]:
    GPIO.setup(pin, GPIO.OUT)

pwm_a = GPIO.PWM(ENA, 1000)
pwm_b = GPIO.PWM(ENB, 1000)
pwm_a.start(0)
pwm_b.start(0)
BASE_SPEED = 35
MIN_SPEED = 35

def set_speed(ls, rs):
    pwm_a.ChangeDutyCycle(ls)
    pwm_b.ChangeDutyCycle(rs)

def stop_motors():
    set_speed(0, 0)
    for pin in [IN1, IN2, IN3, IN4]:
        GPIO.output(pin, GPIO.LOW)

def forward(ls, rs):
    set_speed(ls, rs)
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

def backward(ls, rs):
    set_speed(ls, rs)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

# ---------------------------
# Line-Following Setup
# ---------------------------
COLOR_RANGES = {
    "red": [(0, 150, 100), (10, 255, 255)],
    "green": [(50, 80, 60), (90, 255, 255)],
    "blue": [(100, 80, 40), (140, 255, 255)],
    "yellow": [(18, 100, 100), (35, 255, 255)],
}
PRIORITY_COLORS = ["red", "blue"]

class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=320):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.setpoint = setpoint
        self.last_error = 0
        self.integral = 0

    def update(self, current, dt):
        error = self.setpoint - current
        self.integral += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0
        self.last_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

pid_black = PID(0.5, 0.01, 0.1)
pid_color = PID(0.5, 0.01, 0.1)

def process_frame(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
    inv = cv2.bitwise_not(binary)
    kern = np.ones((5, 5), np.uint8)
    eroded = cv2.erode(inv, kern, iterations=1)
    return cv2.bitwise_not(eroded)

def detect_colored_line(frame, color):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    low, high = COLOR_RANGES[color]
    mask = cv2.inRange(hsv, np.array(low), np.array(high))
    return cv2.erode(mask, np.ones((5, 5), np.uint8), iterations=1)

# ---------------------------
# Detection Setup
# ---------------------------
# Define the four specific symbols to detect
SYMBOL_INDICES = [4, 5, 9, 10]  # Distance, Face recognition, Stop, Traffic
DISTANCE_SYMBOL_IDX = 4
SYMBOL_CONF_THRESHOLD = 0.85

MODEL_PATH = "model_unquant.tflite"
LABELS_PATH = "labels.txt"
with open(LABELS_PATH) as f:
    labels = [l.strip() for l in f]
interpreter = Interpreter(model_path=MODEL_PATH)
interpreter.allocate_tensors()
i_det = interpreter.get_input_details()
o_det = interpreter.get_output_details()
h_tf, w_tf = i_det[0]["shape"][1:3]

# ---------------------------
# Encoder Setup
# ---------------------------
# Define encoder pins
ENCODER_RIGHT = 5   # BCM 5, Physical Pin 29
ENCODER_LEFT = 6    # BCM 6, Physical Pin 31

# Constants for distance calculation
WHEEL_CIRCUM = 22.0         # Wheel circumference in cm
PULSES_PER_REVOLUTION = 39.36  # Pulses per full revolution

# Variables to track encoder counts
enc_left_count = 0
enc_right_count = 0
distance = 0
tracking_distance = False  # Flag to track if we're measuring distance

# Set up encoder pins as inputs with pull-ups
GPIO.setup(ENCODER_LEFT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(ENCODER_RIGHT, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Encoder callback functions
def update_left_encoder(channel):
    global enc_left_count, distance, tracking_distance
    enc_left_count += 1  # Increment on each rising edge
    if tracking_distance:
        # Update distance calculation in real-time
        distance = (enc_left_count / PULSES_PER_REVOLUTION) * WHEEL_CIRCUM

def update_right_encoder(channel):
    global enc_right_count
    enc_right_count += 1  # Increment on each rising edge

# Add event detection for encoders
GPIO.add_event_detect(ENCODER_LEFT, GPIO.RISING, callback=update_left_encoder)
GPIO.add_event_detect(ENCODER_RIGHT, GPIO.RISING, callback=update_right_encoder)

def start_distance_tracking():
    """Initialize encoder counting and start distance tracking"""
    global enc_left_count, enc_right_count, distance, tracking_distance
    
    # Reset encoder counts
    enc_left_count = 0
    enc_right_count = 0
    distance = 0
    tracking_distance = True
    
    print("Starting continuous distance measurement...")

# ---------------------------
# Main Loop
# ---------------------------
cam = Picamera2()
cfg = cam.create_preview_configuration(main={"format":"RGB888","size":(640,480)})
cam.configure(cfg)
cam.start()
time.sleep(2)
last_time = time.time()
distance_print_time = time.time()  # Time tracker for distance reporting

# Define zoom parameters for symbol detection
ZOOM_FACTOR = 1.5  # Zoom factor for symbol detection
ZOOM_CENTER_X = 320  # Center X coordinate (middle of frame)
ZOOM_CENTER_Y = 240  # Center Y coordinate (middle of frame)

try:
    while True:
        frame = cam.capture_array()
        h, w, _ = frame.shape
        
        # Create zoomed frame for symbol detection
        # Calculate zoom region boundaries
        zoom_width = int(w / ZOOM_FACTOR)
        zoom_height = int(h / ZOOM_FACTOR)
        zoom_x = max(0, min(ZOOM_CENTER_X - zoom_width // 2, w - zoom_width))
        zoom_y = max(0, min(ZOOM_CENTER_Y - zoom_height // 2, h - zoom_height))
        
        # Extract zoomed region
        zoom_frame = frame[zoom_y:zoom_y+zoom_height, zoom_x:zoom_x+zoom_width]
        # Resize back to original dimensions for display
        zoom_frame_display = cv2.resize(zoom_frame, (w, h))
        
        # Line following section - only take bottom 30 pixels
        line_roi = frame[h-30:h, :]
        
        # Process zoomed frame for symbol detection
        inp = cv2.resize(zoom_frame, (w_tf, h_tf)).astype(np.float32)/255.0
        interpreter.set_tensor(i_det[0]["index"], np.expand_dims(inp, 0))
        interpreter.invoke()
        out = interpreter.get_tensor(o_det[0]["index"])[0]
        idx = int(np.argmax(out))
        conf = float(out[idx])

        # Print distance every second if tracking is active
        if tracking_distance and time.time() - distance_print_time > 1.0:
            print(f"Distance: {distance:.2f} cm, Left: {enc_left_count}, Right: {enc_right_count}")
            distance_print_time = time.time()

        # Symbol Detection - only react to the four specified symbols
        if idx in SYMBOL_INDICES and conf >= SYMBOL_CONF_THRESHOLD:
            stop_motors()
            print(f"Detected symbol {labels[idx]} (idx {idx}), conf {conf:.2f}")
            
            if idx == DISTANCE_SYMBOL_IDX and not tracking_distance:  # If distance symbol detected
                start_distance_tracking()
                print("Distance tracking enabled - continuing line following")
                time.sleep(1)  # Brief pause before continuing
            else:  # For the other three symbols
                print("Pausing 2s")
                time.sleep(2)
                
            forward(65, 65)
            time.sleep(0.3)
            last_time = time.time()
            continue
        # If detected something else, just continue with line following (no action needed)

        # Line-following
        now = time.time()
        dt = now - last_time
        last_time = now
        roi = frame[int(h*0.6):int(h*0.9), :]
        followed = False
        for color in PRIORITY_COLORS:
            mask = detect_colored_line(roi, color)
            pts = np.where(mask>200)[1]
            if pts.size:
                center = int(np.mean(pts))
                corr = pid_color.update(center, dt)
                forward(max(min(BASE_SPEED-corr, MIN_SPEED), 0), max(min(BASE_SPEED+corr, MIN_SPEED), 0))
                followed = True
                break
        if not followed:
            proc = process_frame(frame)
            ptsb = np.where(proc[int(h*0.6):int(h*0.9), :]<50)[1]
            if ptsb.size:
                center = int(np.mean(ptsb))
                corr = pid_black.update(center, dt)
                forward(max(min(BASE_SPEED-corr, MIN_SPEED), 0), max(min(BASE_SPEED+corr, MIN_SPEED), 0))
            else:
                stop_motors()
                backward(45, 45)
                time.sleep(0.3)
                stop_motors()

        # Display distance on frame if tracking
        if tracking_distance:
            cv2.putText(zoom_frame_display, f"Distance: {distance:.2f} cm", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
        # Display symbol detection confidence
        detected_text = f"Symbol: {labels[idx]} ({conf:.2f})"
        symbol_color = (0, 255, 0) if idx in SYMBOL_INDICES else (0, 0, 255)  # Green for target symbols, red otherwise
        cv2.putText(zoom_frame_display, detected_text, (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, symbol_color, 2)

        # Display frames in separate windows
        cv2.imshow("Symbol Detection", zoom_frame_display)
        cv2.imshow("Line Following", line_roi)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    if tracking_distance:
        print(f"Final distance: {distance:.2f} cm")
    stop_motors()
    cam.stop()
    cv2.destroyAllWindows()
    GPIO.cleanup()