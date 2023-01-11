from adafruit_servokit import ServoKit
import busio

from board import SCL, SDA
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor
import time

def main():
    angle = 80

    i2c = busio.I2C(SCL, SDA)

    kit = ServoKit(channels = 16)

    pca = PCA9685(i2c, address=0x40)
    pca.frequency = 100

    pca.channels[15].duty_cycle = 0xFFFF
    pca.channels[9].duty_cycle = 0xFFFF

    motorR = motor.DCMotor(pca.channels[14], pca.channels[13])
    motorL = motor.DCMotor(pca.channels[10], pca.channels[11])
    motorR.throttle = -0.6
    motorL.throttle = 0.6
    kit.servo[7].angle = angle
    time.sleep(0.25)

    kit.servo[7].angle = 80
    motorR.throttle = 0
    motorL.throttle = -0
    time.sleep(0.2)

    kit.servo[7].angle = 165
    motorR.throttle = 0.25
    motorL.throttle = -1
    time.sleep(0.83)

    kit.servo[7].angle = 80
    motorR.throttle = 0
    motorL.throttle = -0


if __name__ == '__main__':
    main()