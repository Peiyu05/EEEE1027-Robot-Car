import RPi.GPIO as GPIO
import time

# Set GPIO mode to BCM
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Define motor control pins (from your code)
# Left Side Motors
IN1 = 17  # BCM 17, Physical Pin 11 - Left Motors Forward
IN2 = 18  # BCM 18, Physical Pin 12 - Left Motors Backward
ENA = 27  # BCM 27, Physical Pin 13 - Left Motors Enable (PWM)

# Right Side Motors
IN3 = 22  # BCM 22, Physical Pin 15 - Right Motors Forward
IN4 = 23  # BCM 23, Physical Pin 16 - Right Motors Backward
ENB = 24  # BCM 24, Physical Pin 18 - Right Motors Enable (PWM)

# Define encoder pins (temporary placeholders; adjust these later)
ENCODER_RIGHT = 5   # BCM 5, Physical Pin 29 (example)
ENCODER_LEFT = 6  # BCM 6, Physical Pin 31 (example)

# Constants for distance calculation (from your Arduino code)
WHEEL_CIRCUM = 22.0         # Wheel circumference in cm
PULSES_PER_REVOLUTION = 39.36  # Pulses per full revolution

# Variables to track encoder counts
enc_left_count = 0
enc_right_count = 0

# Set up motor pins as outputs
motor_pins = [IN1, IN2, ENA, IN3, IN4, ENB]
for pin in motor_pins:
    GPIO.setup(pin, GPIO.OUT)

# Set up PWM for speed control (frequency = 100Hz)
pwm_a = GPIO.PWM(ENA, 100)  # Left motors speed
pwm_b = GPIO.PWM(ENB, 100)  # Right motors speed
pwm_a.start(0)
pwm_b.start(0)

# Set up encoder pins as inputs with pull-ups
GPIO.setup(ENCODER_LEFT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(ENCODER_RIGHT, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Motor control functions
def set_speed(speed):
    """Set speed for all motors (0-100)"""
    pwm_a.ChangeDutyCycle(speed)
    pwm_b.ChangeDutyCycle(speed)

def stop():
    """Stop all motors"""
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    set_speed(0)

def forward():
    """Move robot forward"""
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    set_speed(35)  # 35% duty cycle as requested

# Encoder callback functions (single-pin, rising edge only)
def update_left_encoder(channel):
    global enc_left_count
    enc_left_count += 1  # Increment on each rising edge

def update_right_encoder(channel):
    global enc_right_count
    enc_right_count += 1  # Increment on each rising edge

# Add event detection for encoders
GPIO.add_event_detect(ENCODER_LEFT, GPIO.RISING, callback=update_left_encoder)
GPIO.add_event_detect(ENCODER_RIGHT, GPIO.RISING, callback=update_right_encoder)

distance = 0

# Main function to move forward and measure distance
def run_robot():
    global distance
    try:
        print("Starting movement...")
        forward()  # Start moving forward at 35% speed
        
        start_time = time.time()
        while time.time() - start_time < 2:  # Run for 2 seconds
            # Calculate distance
            encoder_average = (enc_left_count + enc_right_count) / 2.0
            # distance = (encoder_average / PULSES_PER_REVOLUTION) * WHEEL_CIRCUM
            distance = (enc_left_count / PULSES_PER_REVOLUTION) * WHEEL_CIRCUM
            
            # Print status
            print(f"Left: {enc_left_count}, Right: {enc_right_count}, Distance: {distance:.2f} cm")
            time.sleep(0.1)  # Update every 100ms
        
        print("Stopping...")
        stop()

    except KeyboardInterrupt:
        print("\nStopped by user")
        
    finally:
        print(f"Left: {enc_left_count}, Right: {enc_right_count}, Distance: {distance:.2f} cm (final)") 
        stop()
        GPIO.cleanup()

# Run the program
if __name__ == "__main__":
    run_robot()

