import RPi.GPIO as GPIO
import time

# Set GPIO mode to BCM
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Define motor control pins
IN1 = 17  # BCM 17, Physical Pin 11 - Left Motors Forward
IN2 = 18  # BCM 18, Physical Pin 12 - Left Motors Backward
ENA = 27  # BCM 27, Physical Pin 13 - Left Motors Enable (PWM)
IN3 = 22  # BCM 22, Physical Pin 15 - Right Motors Forward
IN4 = 23  # BCM 23, Physical Pin 16 - Right Motors Backward
ENB = 24  # BCM 24, Physical Pin 18 - Right Motors Enable (PWM)

# Set up all pins as outputs
motor_pins = [IN1, IN2, ENA, IN3, IN4, ENB]
for pin in motor_pins:
    GPIO.setup(pin, GPIO.OUT)

# Set up PWM for speed control (frequency = 100Hz)
pwm_a = GPIO.PWM(ENA, 100)  # Left motors speed
pwm_b = GPIO.PWM(ENB, 100)  # Right motors speed

# Start PWM with 0% duty cycle
pwm_a.start(0)
pwm_b.start(0)

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

def forward(speed):
    """Move robot forward at specified duty cycle"""
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    set_speed(speed)

def backward(speed):
    """Move robot backward at specified duty cycle"""
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    set_speed(speed)

def left(speed):
    """Turn robot left at specified duty cycle"""
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    set_speed(speed)

def right(speed):
    """Turn robot right at specified duty cycle"""
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    set_speed(speed)

# Test function for the experiment
def run_experiment():
    # Duty cycles as specified
    forward_duties = [45, 50, 55]
    reverse_duties = [45, 50, 55]
    left_duties = [85, 90, 95, 100]
    right_duties = [75, 80, 85]
    stop_duty = [0]

    try:
        print("Starting Experiment...\nPlace robot on smooth cardboard surface.")
        input("Press Enter to begin...")

        # Test Forward
        print("\nTesting Forward Movements:")
        for duty in forward_duties:
            print(f"Forward at {duty}% duty cycle")
            forward(duty)
            time.sleep(1)  # Move for 1 second
            stop()
            input("Measure distance (cm) and deviation (°), then press Enter...")

        # Test Reverse
        print("\nTesting Reverse Movements:")
        for duty in reverse_duties:
            print(f"Reverse at {duty}% duty cycle")
            backward(duty)
            time.sleep(1)
            stop()
            input("Measure distance (cm) and deviation (°), then press Enter...")

        # Test Left Turn
        print("\nTesting Left Turns:")
        for duty in left_duties:
            print(f"Left turn at {duty}% duty cycle")
            left(duty)
            time.sleep(1)
            stop()
            input("Measure angle deviation from 90° (+ for >90°, - for <90°), then press Enter...")

        # Test Right Turn
        print("\nTesting Right Turns:")
        for duty in right_duties:
            print(f"Right turn at {duty}% duty cycle")
            right(duty)
            time.sleep(1)
            stop()
            input("Measure angle deviation from 90° (+ for >90°, - for <90°), then press Enter...")

        # Test Stop
        print("\nTesting Stop:")
        for duty in stop_duty:
            print(f"Stop at {duty}% duty cycle")
            stop()
            time.sleep(1)
            input("Confirm 0 cm movement, then press Enter...")

        print("\nExperiment Complete!")

    except KeyboardInterrupt:
        print("\nStopped by user")
        
    finally:
        stop()
        GPIO.cleanup()

if __name__ == "__main__":
    run_experiment()