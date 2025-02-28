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

# Test function for the experiment
def run_experiment():
    # Duty cycles representing each range
    duty_cycles = [
        10,  # 0 < x ≤ 20
        30,  # 20 < x ≤ 40
        50,  # 40 < x ≤ 60
        70,  # 60 < x ≤ 80
        90   # 80 < x ≤ 100
    ]
    runs_per_duty = 10
    track_length = 89.1  # 3 A4 papers (cm)

    try:
        print("Starting Experiment...\nPlace robot at start of 3 A4 paper track (89.1 cm).")
        input("Press Enter to begin...")

        # Results dictionary to store counts
        results = {duty: {"Success": 0, "Astray": 0, "Stutter": 0} for duty in duty_cycles}

        for duty in duty_cycles:
            print(f"\nTesting Duty Cycle {duty}% (Range {max(0, duty-20)}-{min(100, duty+10)}):")
            for run in range(1, runs_per_duty + 1):
                print(f"Run {run}/10 at {duty}%")
                forward(duty)
                # Let it run for a reasonable time to cover 89.1 cm (adjust based on observation)
                time.sleep(3)  # Adjust this based on your robot's speed
                stop()
                
                # Manual input for outcome
                while True:
                    outcome = input("Enter outcome (S for Success, A for Astray, T for Stutter): ").upper()
                    if outcome in ['S', 'A', 'T']:
                        break
                    print("Invalid input. Use S, A, or T.")
                
                if outcome == 'S':
                    results[duty]["Success"] += 1
                elif outcome == 'A':
                    results[duty]["Astray"] += 1
                elif outcome == 'T':
                    results[duty]["Stutter"] += 1
                
                # Reset robot to start
                input("Reset robot to start of track, then press Enter...")

        # Display results
        print("\nExperiment Results:")
        print("| Duty Cycle Range | Success | Astray | Stutter | Completion Rate (%) |")
        print("|------------------|---------|--------|---------|---------------------|")
        for duty in duty_cycles:
            range_str = f"{max(0, duty-20)}<{duty}≤{min(100, duty+10)}"
            success = results[duty]["Success"]
            astray = results[duty]["Astray"]
            stutter = results[duty]["Stutter"]
            completion_rate = (success / runs_per_duty) * 100
            print(f"| {range_str:15} | {success:7} | {astray:6} | {stutter:7} | {completion_rate:18.1f} |")

    except KeyboardInterrupt:
        print("\nStopped by user")
        
    finally:
        stop()
        GPIO.cleanup()

if __name__ == "__main__":
    run_experiment()