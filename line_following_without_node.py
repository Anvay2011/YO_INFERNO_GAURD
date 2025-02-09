#!/usr/bin/env python3
import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

# =============================================================================
#                           PIN & PARAM CONFIG
# =============================================================================

# ------------------ Motor Pins (BOARD) ------------------
LEFT_MOTOR_FORWARD   = 33
LEFT_MOTOR_BACKWARD  = 31
RIGHT_MOTOR_FORWARD  = 37
RIGHT_MOTOR_BACKWARD = 35

# ------------------ Motor PWM Settings -------------------
GPIO_FREQUENCY = 1000   # 1 kHz

# ------------------ Line-Follow ------------------------
BLACK_LINE_THRESHOLD    = 70  # For horizontal black line detection
THRESHOLD_SENSITIVITY   = 70  # For general line threshold
DEAD_ZONE               = 100  # Pivot threshold around center

# Speeds (0–100 duty cycle)
BASE_SPEED  = 70
FULL_SPEED  = 60

# =============================================================================
#                           GPIO SETUP / TEARDOWN
# =============================================================================
def setup_gpio():
    GPIO.setmode(GPIO.BOARD)

    # Motor pins
    GPIO.setup(LEFT_MOTOR_FORWARD, GPIO.OUT)
    GPIO.setup(LEFT_MOTOR_BACKWARD, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR_FORWARD, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR_BACKWARD, GPIO.OUT)

    # PWM objects
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
#                          MOTOR & PIVOT HELPERS
# =============================================================================
def set_motors(left_speed, right_speed):
    """Sets motor speeds in [-100..100]."""
    # Left motor
    if left_speed > 0:
        pwm_left_forward.ChangeDutyCycle(left_speed)
        pwm_left_backward.ChangeDutyCycle(0)
    else:
        pwm_left_forward.ChangeDutyCycle(0)
        pwm_left_backward.ChangeDutyCycle(abs(left_speed))

    # Right motor
    if right_speed > 0:
        pwm_right_forward.ChangeDutyCycle(right_speed)
        pwm_right_backward.ChangeDutyCycle(0)
    else:
        pwm_right_forward.ChangeDutyCycle(0)
        pwm_right_backward.ChangeDutyCycle(abs(right_speed))

# =============================================================================
#                              MAIN
# =============================================================================
def main():
    setup_gpio()
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Cannot open camera. Exiting...")
        cleanup_gpio()
        return

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            # ------------------- Line Follow -------------------------
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            _, thresh = cv2.threshold(gray, THRESHOLD_SENSITIVITY, 255, cv2.THRESH_BINARY_INV)

            M = cv2.moments(thresh, False)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                cx = frame.shape[1] // 2
                cy = frame.shape[0] // 2

            error = cx - (frame.shape[1] // 2)
            if error < -DEAD_ZONE:
                direction_text = "Pivot LEFT"
                set_motors(FULL_SPEED, -FULL_SPEED)
            elif error > DEAD_ZONE:
                direction_text = "Pivot RIGHT"
                set_motors(-FULL_SPEED, FULL_SPEED)
            else:
                direction_text = "Forward"
                set_motors(BASE_SPEED, BASE_SPEED)

            # Optional Overlays
            cv2.circle(frame, (cx, cy), 10, (255,0,0), -1)
            cv2.putText(frame, direction_text, (10,40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)

            # Show frames
            cv2.imshow("Frame", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("User interrupted.")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        cleanup_gpio()

# =============================================================================
#                               ENTRY
# =============================================================================
if __name__ == "__main__":
    main()
