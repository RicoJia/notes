from __future__ import division
import time

# Import the PCA9685 module.
import Adafruit_PCA9685

# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()

# Configure min and max servo pulse lengths
servo_min = 150.0  # Min pulse length out of 4096, corresponding to 0 degrees
servo_max = 640.0  # Max pulse length out of 4096 corresponding to 180 degrees

# angle is in deg
def get_pwm(angle): 
    return int((servo_max-servo_min) * (angle/180.0) + servo_min)

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)

while True:
    for channel_num in range(6): 
        pwm_val = get_pwm(90)
        pwm.set_pwm(channel_num, 0, pwm_val)
        print(pwm_val)
    time.sleep(1)
    # for channel_num in range(6): 
    #     pwm_val = get_pwm(0)
    #     pwm.set_pwm(channel_num, 0, pwm_val)
    #     print(pwm_val)
    # time.sleep(1)

