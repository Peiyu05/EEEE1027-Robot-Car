import time

import RPi.GPIO as GPIO

# Set GPIO mode to BCM
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Define motor control pins (adjust based on your wiring)
# Left Side Motors (controlled together)
IN1 = 17  # BCM 17, Physical Pin 11 - Left Motors Forward
IN2 = 18  # BCM 18, Physical Pin 12 - Left Motors Backward
ENA = 27  # BCM 27, Physical Pin 13 - Left Motors Enable (PWM)

# Right Side Motors (controlled together)
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


def validate_duty_cycle_input(prompt):
    while True:
        try:
            duty_cycle_in = float(input(prompt))
        except ValueError:
            print("Input is not a number.\nPlease try again.")
            continue
        if duty_cycle_in == -1:
            # exit condition
            break
        elif duty_cycle_in < 0:
            print("Duty cycle cannot be less than 0.\nPlease try again.")
        elif duty_cycle_in > 100:
            print("Duty cycle cannot be greater than 100.\nPlease try again.")
        else:
            # input validation complete
            break
    return duty_cycle_in


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


def left(speed):
    """Move robot forward at specified duty cycle"""
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    set_speed(speed)


wait_time = 1


# Test function to demonstrate left movement with specified duties
def test_movement():

    try:
        print("Starting Left Movement Test...\nPlace robot on smooth surface.")
        input("Press Enter to begin...")

        duty_cycle_in = validate_duty_cycle_input(
            "Please enter desired duty cycle D between 0 and 100: "
        )
        while duty_cycle_in != -1:
            print(f"\nTesting Left movement @ {duty_cycle_in}% duty cycle...")
            left(duty_cycle_in)
            time.sleep(wait_time)  # Move for 1 seconds
            stop()
            input(f"Measure angle (degrees), then press Enter to continue...")
            duty_cycle_in = validate_duty_cycle_input(
                "\nPlease enter the next desired duty cycle D between 0 and 100: "
            )

        print("\nLeft Movement Test Complete!")

    except KeyboardInterrupt:
        print("\nStopped by user")

    finally:
        stop()
        GPIO.cleanup()


# Run the test
if __name__ == "__main__":
    test_movement()

