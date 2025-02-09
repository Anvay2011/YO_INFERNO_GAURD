import RPi.GPIO as GPIO
import time

# ------------------ Ultrasonic Sensor Pins ------------------
TRIG = 16
ECHO = 18

# ------------------ Motor Pins ------------------
LEFT_MOTOR_FORWARD   = 33
LEFT_MOTOR_BACKWARD  = 31
RIGHT_MOTOR_FORWARD  = 37
RIGHT_MOTOR_BACKWARD = 35

GPIO_FREQUENCY = 1000  # 1 kHz
STOP_DISTANCE = 20  # cm
SLOW_SPEED = 40
MIN_DISTANCE = 4  # cm
MAX_DISTANCE = 8  # cm

# =============================================================================
#                           GPIO SETUP / TEARDOWN
# =============================================================================
def setup_gpio():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(TRIG, GPIO.OUT)
    GPIO.setup(ECHO, GPIO.IN)
    GPIO.setup(LEFT_MOTOR_FORWARD, GPIO.OUT)
    GPIO.setup(LEFT_MOTOR_BACKWARD, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR_FORWARD, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR_BACKWARD, GPIO.OUT)
    
    global pwm_left_forward, pwm_left_backward
    global pwm_right_forward, pwm_right_backward
    
    pwm_left_forward = GPIO.PWM(LEFT_MOTOR_FORWARD, GPIO_FREQUENCY)
    pwm_left_backward = GPIO.PWM(LEFT_MOTOR_BACKWARD, GPIO_FREQUENCY)
    pwm_right_forward = GPIO.PWM(RIGHT_MOTOR_FORWARD, GPIO_FREQUENCY)
    pwm_right_backward = GPIO.PWM(RIGHT_MOTOR_BACKWARD, GPIO_FREQUENCY)
    
    pwm_left_forward.start(0)
    pwm_left_backward.start(0)
    pwm_right_forward.start(0)
    pwm_right_backward.start(0)

def cleanup_gpio():
    pwm_left_forward.stop()
    pwm_left_backward.stop()
    pwm_right_forward.stop()
    pwm_right_backward.stop()
    GPIO.cleanup()

# =============================================================================
#                          MOTOR CONTROL
# =============================================================================
def set_motors(left_speed, right_speed):
    if left_speed > 0:
        pwm_left_forward.ChangeDutyCycle(left_speed)
        pwm_left_backward.ChangeDutyCycle(0)
    else:
        pwm_left_forward.ChangeDutyCycle(0)
        pwm_left_backward.ChangeDutyCycle(abs(left_speed))

    if right_speed > 0:
        pwm_right_forward.ChangeDutyCycle(right_speed)
        pwm_right_backward.ChangeDutyCycle(0)
    else:
        pwm_right_forward.ChangeDutyCycle(0)
        pwm_right_backward.ChangeDutyCycle(abs(right_speed))

# =============================================================================
#                          ULTRASONIC SENSOR
# =============================================================================
def get_distance():
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    start_time = time.time()
    stop_time = time.time()

    while GPIO.input(ECHO) == 0:
        start_time = time.time()
    while GPIO.input(ECHO) == 1:
        stop_time = time.time()
    
    elapsed_time = stop_time - start_time
    distance = (elapsed_time * 34300) / 2
    
    if distance < MIN_DISTANCE:
        distance = MIN_DISTANCE
    elif distance > MAX_DISTANCE:
        distance = MAX_DISTANCE
    
    return distance

# =============================================================================
#                              MAIN
# =============================================================================
def main():
    setup_gpio()
    try:
        while True:
            distance = get_distance()
            print(f"Distance: {distance:.2f} cm")
            
            if distance < STOP_DISTANCE:
                print("Obstacle detected! Stopping.")
                set_motors(0, 0)
                time.sleep(1)
                set_motors(-SLOW_SPEED, SLOW_SPEED)  # Turn
                time.sleep(1)
            else:
                set_motors(SLOW_SPEED, SLOW_SPEED)
            
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("User interrupted.")
    finally:
        cleanup_gpio()

if __name__ == "__main__":
    main()
