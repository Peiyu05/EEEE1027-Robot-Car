import cv2
import numpy as np
import math
import time
from picamera2 import Picamera2
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

pwm_a = GPIO.PWM(ENA, 100)
pwm_b = GPIO.PWM(ENB, 100)
pwm_a.start(35)
pwm_b.start(35)

def set_speed(ls, rs):
    pwm_a.ChangeDutyCycle(ls)
    pwm_b.ChangeDutyCycle(rs)

def stop_motors():
    set_speed(0, 0)
    for pin in [IN1, IN2, IN3, IN4]:
        GPIO.output(pin, GPIO.LOW)

def forward(ls, rs):
    set_speed(ls, rs)
    GPIO.output(IN1, GPIO.HIGH); GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH); GPIO.output(IN4, GPIO.LOW)

def backward(ls, rs):
    set_speed(ls, rs)
    GPIO.output(IN1, GPIO.LOW); GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW); GPIO.output(IN4, GPIO.HIGH)

def turn_left():
    forward(30, 70)

def turn_right():
    forward(70, 30)

# ---------------------------
# PID-based Line Following
# ---------------------------
class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=320):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.setpoint = setpoint
        self.last_error = 0
        self.integral   = 0

    def update(self, current, dt):
        error = self.setpoint - current
        self.integral += error * dt
        derivative = (error - self.last_error)/dt if dt>0 else 0
        self.last_error = error
        return self.Kp*error + self.Ki*self.integral + self.Kd*derivative

pid_black = PID(0.5, 0.01, 0.1)
pid_color = PID(0.5, 0.01, 0.1)
BASE_SPEED = 30
MIN_SPEED  = 30

# ---------------------------
# Color Range Definitions
# ---------------------------
COLOR_RANGES = {
    "red":    [(  0,150,100), ( 10,255,255)],
    "green":  [( 50, 80,  60), ( 90,255,255)],
    "blue":   [(100, 80,  40), (140,255,255)],
    "yellow": [( 18,100,100), ( 35,255,255)],
}
PRIORITY_COLORS = ["red","blue"]

# Additional red range (for hue wrap-around)
RED_RANGE2 = [(170, 150, 100), (180, 255, 255)]

def process_frame(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray,100,255,cv2.THRESH_BINARY)
    inv = cv2.bitwise_not(binary)
    kern = np.ones((5,5),np.uint8)
    er  = cv2.erode(inv, kern, iterations=1)
    return cv2.bitwise_not(er)

def detect_colored_line(frame, color):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    low, high = COLOR_RANGES[color]
    mask = cv2.inRange(hsv, np.array(low), np.array(high))
    return cv2.erode(mask, np.ones((5,5),np.uint8), iterations=1)

# ---------------------------
# Improved Shape Detection
# ---------------------------
# HSV color bounds
lower_yellow = np.array([15, 60, 100])
upper_yellow = np.array([45, 255, 255])
lower_green = np.array([50, 80, 60])
upper_green = np.array([90, 255, 255])
lower_red   = np.array([0, 150, 100])
upper_red   = np.array([10, 255, 255])
lower_blue  = np.array([100, 80, 40])
upper_blue  = np.array([140,255,255])

# HSV bounds for arrow detection (blue arrows)
arrow_minHSV = np.array([100, 100, 50])
arrow_maxHSV = np.array([140, 255, 255])


def detect_shape(cnt, frame, hsv_frame):
    shape = "unknown"
    area = cv2.contourArea(cnt)
    if area < 300 or len(cnt) < 3:
        return shape
    peri = cv2.arcLength(cnt, True)
    eps = 0.03 * peri if area > 2000 else 0.08 * peri
    approx = cv2.approxPolyDP(cnt, eps, True)
    v = len(approx)

    circularity = (4 * math.pi * area) / (peri * peri) if peri > 0 else 0
    if v == 3:
        return "triangle"

    if 4 <= v <= 8:
        maskHSV = cv2.inRange(hsv_frame, arrow_minHSV, arrow_maxHSV)
        maskHSV = cv2.morphologyEx(maskHSV, cv2.MORPH_CLOSE, np.ones((5,5),np.uint8))
        maskHSV = cv2.morphologyEx(maskHSV, cv2.MORPH_OPEN,  np.ones((3,3),np.uint8))
        x,y,w,h = cv2.boundingRect(cnt)
        arrow_region = maskHSV[y:y+h, x:x+w]
        if arrow_region.size > 0:
            blurIm = cv2.GaussianBlur(arrow_region, (9, 9), 0)
            corners = cv2.goodFeaturesToTrack(blurIm, 2, 0.7, 15)
        if corners is not None and len(corners) >= 2:
            corners = np.int0(corners)
            x0, y0 = corners[0].ravel()
            x1, y1 = corners[1].ravel()
            x0, y0 = x0 + x, y0 + y
            x1, y1 = x1 + x, y1 + y

            cv2.circle(frame, (x0, y0), 5, (0, 0, 255), -1)
            cv2.circle(frame, (x1, y1), 5, (0, 0, 255), -1)

            am, bm = (x0 + x1) / 2, (y0 + y1) / 2
            cv2.circle(frame, (int(am), int(bm)), 3, (255, 0, 0), -1)

            (cx, cy), radius = cv2.minEnclosingCircle(cnt)
            cv2.circle(frame, (int(cx), int(cy)), int(radius), (0, 0, 255), 2)
            cv2.line(frame, (int(cx), int(cy)), (int(am), int(bm)), (255, 0, 0), 2)

            angle = math.degrees(math.atan2(bm - cy, am - cx))
            if -45 <= angle < 45:
                return "arrow (right)"
            elif 45 <= angle < 135:
                return "arrow (down)"
            elif -180 <= angle <= -135 or 135 <= angle <= 180:
                return "arrow (left)"
            elif -135 < angle < -45:
                return "arrow (up)"

    if v == 4:
        x, y, w, h = cv2.boundingRect(approx)
        ar = w / float(h)
        shape = "square" if 0.95 <= ar <= 1.05 else "rectangle"
    elif v == 5:
        shape = "pentagon"
    elif v == 6:
        shape = "hexagon"
    elif v > 6:
        circularity = (4 * math.pi * area) / (peri * peri)
        shape = "full circle" if circularity > 0.8 else "partial circle"
    return shape

# ---------------------------
# Camera Init
# ---------------------------
cam = Picamera2()
cfg = cam.create_preview_configuration(main={"format":"RGB888","size":(640,480)})
cam.configure(cfg)
cam.start()
time.sleep(2)

last_time = time.time()
final_shape_text = "_____"
shape_counter=0
try:
    while True:
        frame = cam.capture_array()
        h, w, _ = frame.shape

        # ---- LINE-FOLLOWING ----
        now = time.time(); dt = now - last_time; last_time = now
        roi = frame[int(h*0.6):int(h*0.9), :]
        did_follow = False
        for color in PRIORITY_COLORS:
            mask = detect_colored_line(roi, color)
            pts  = np.where(mask>200)[1]
            if pts.size:
                center = int(np.mean(pts))
                corr   = pid_color.update(center, dt)
                ls = max(min(BASE_SPEED - corr, MIN_SPEED), 0)
                rs = max(min(BASE_SPEED + corr, MIN_SPEED), 0)
                forward(ls, rs)
                did_follow = True
                break
        if not did_follow:
            proc = process_frame(frame)
            ptsb = np.where(proc[int(h*0.6):int(h*0.9),:]<50)[1]
            if ptsb.size:
                center = int(np.mean(ptsb))
                corr   = pid_black.update(center, dt)
                ls = max(min(BASE_SPEED - corr, MIN_SPEED), 0)
                rs = max(min(BASE_SPEED + corr, MIN_SPEED), 0)
                forward(ls, rs)
            else:
                stop_motors()
                backward(75,75)
                time.sleep(0.3)
                stop_motors()

        # display line-following crop (bottom 30px)
        line_roi = frame[h-30:h, :]
        cv2.imshow("Line Following", line_roi)

        # ---- SHAPE/ARROW DETECTION ----
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        gray      = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur      = cv2.GaussianBlur(gray, (9,9), 0)
        thresh    = cv2.adaptiveThreshold(blur,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                          cv2.THRESH_BINARY,15,2)
        edges     = cv2.Canny(thresh,70,40)
        cnts, _   = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for c in cnts:
            if cv2.contourArea(c) > 100:
                shape = detect_shape(c, frame, hsv_frame)
                shape_text = shape if shape.startswith("arrow") else shape
                if final_shape_text != shape_text:
                    shape_counter += 1
                else:
                    shape_counter = 0
                if shape_counter >= 5:
                    final_shape_text = shape_text
                print(f"Detected shape: {shape}")
                cv2.drawContours(frame, [c], -1, (0, 0, 255), 2)
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    cv2.putText(frame, final_shape_text, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        cv2.imshow("Shape/Arrow Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    stop_motors()
    cam.stop()
    cv2.destroyAllWindows()
    GPIO.cleanup()