# import cv2
import keyboard
from adafruit_servokit import ServoKit
import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor
import time

def main():
    # setup()
    # startGame()
    angle = 90
    try:
        i2c = busio.I2C(SCL, SDA)

        kit = ServoKit(channels = 16)

        pca = PCA9685(i2c, address=0x40)
        pca.frequency = 100

        pca.channels[15].duty_cycle = 0xFFFF
        pca.channels[9].duty_cycle = 0xFFFF

        motorR = motor.DCMotor(pca.channels[14], pca.channels[13])
        motorL = motor.DCMotor(pca.channels[10], pca.channels[11])
        motorR.throttle = 0.3
        motorL.throttle = -0.3
        kit.servo[7].angle = angle
        stop = False
        while True:
            print(angle)
            if keyboard.is_pressed('w'):
                motorR.throttle = 0.3
                motorL.throttle = -0.3
            elif keyboard.is_pressed('s'):
                motorR.throttle = -0.3
                motorL.throttle = 0.3
            if keyboard.is_pressed('a'): 
                angle -= 1 
            elif keyboard.is_pressed('d'): 
                angle = angle + 1
            if keyboard.is_pressed('q'):
                break
            if angle <= 30:
                angle = 30
            if angle >= 165:
                angle = 165
            kit.servo[7].angle = angle
            time.sleep(0.01)
        kit.servo[7].angle = 90
        motorR.throttle = 0
        motorL.throttle = 0

        pca.deinit()
    except KeyboardInterrupt:
        motorR.throttle = 0
        motorL.throttle = 0

        pca.deinit()

def setup():
    i2c = busio.I2C(SCL, SDA)

    kit = ServoKit(channels = 16)

    pca = PCA9685(i2c, address=0x40)
    pca.frequency = 100

    pca.channels[15].duty_cycle = 0xFFFF
    pca.channels[9].duty_cycle = 0xFFFF

    motorR = motor.DCMotor(pca.channels[14], pca.channels[13])
    motorL = motor.DCMotor(pca.channels[10], pca.channels[11])
    motorR.throttle = 0.6
    motorL.throttle = -0.6

def startGame():
    angle = 80
    kit.servo[7].angle = angle
    try:
        i2c = busio.I2C(SCL, SDA)

        kit = ServoKit(channels = 16)

        pca = PCA9685(i2c, address=0x40)
        pca.frequency = 100

        pca.channels[15].duty_cycle = 0xFFFF
        pca.channels[9].duty_cycle = 0xFFFF

        motorR = motor.DCMotor(pca.channels[14], pca.channels[13])
        motorL = motor.DCMotor(pca.channels[10], pca.channels[11])
        motorR.throttle = 0.6
        motorL.throttle = -0.6
        while True:
            print("test")
            if cv2.waitKey(0) == ord('w'):
                motorR.throttle = 0.6
                motorL.throttle = -0.6
            elif cv2.waitKey(0) == ord('s'):
                motorR.throttle = -0.6
                motorL.throttle = 0.6
            if cv2.waitKey(0) == ord('a'):
                kit.servo[7].angle = angle - 1
            elif cv2.waitKey(0) == ord('d'):
                kit.servo[7].angle = angle + 1
    except KeyboardInterrupt:
        motorR.throttle = 0
        motorL.throttle = 0

        pca.deinit()

if __name__ == '__main__' :
    main()