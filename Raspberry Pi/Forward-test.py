import RPi.GPIO as GPIO
import time

# Set GPIO mode to BCM
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Define motor control pins (adjust based on your wiring)
IN1 = 17  # Left Motors Forward
IN2 = 18  # Left Motors Backward
ENA = 27  # Left Motors Enable (PWM)
IN3 = 22  # Right Motors Forward
IN4 = 23  # Right Motors Backward
ENB = 24  # Right Motors Enable (PWM)

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
    """Set speed for all motors (0-100)."""
    pwm_a.ChangeDutyCycle(speed)
    pwm_b.ChangeDutyCycle(speed)

def stop():
    """Stop all motors."""
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    set_speed(0)

def forward(speed):
    """Move robot forward at specified duty cycle."""
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    set_speed(speed)

def test_movement():
    """Interactive forward movement test."""
    wait_time = 1  # seconds to run at each duty cycle
    print("Interactive Forward Movement Test")
    print("Enter a duty cycle (0-100) to test, or 'q' to quit.\n")

    try:
        while True:
            user_input = input("Duty cycle [%] (or 'q' to exit): ").strip().lower()

            if user_input == 'q':
                print("Exiting test loop.")
                break

            try:
                duty = int(user_input)
                if not 0 <= duty <= 100:
                    print("→ Please enter an integer between 0 and 100.")
                    continue
            except ValueError:
                print("→ Invalid input. Enter an integer (0 to 100) or 'q'.")
                continue

            print(f"→ Running at {duty}% for {wait_time}s...")
            forward(duty)
            time.sleep(wait_time)
            stop()
            input("Measure distance traveled, then press Enter to continue...\n")

        print("Forward Movement Test Complete!")

    except KeyboardInterrupt:
        print("\nTest interrupted by user.")

    finally:
        stop()
        GPIO.cleanup()

if __name__ == "__main__":
    test_movement()
