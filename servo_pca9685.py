from machine import Pin, I2C
import time

# PCA9685 constants
PCA9685_ADDR = 0x40
MODE1 = 0x00
PRESCALE = 0xFE
LED0_ON_L = 0x06

# Setup I2C on GPIO19 (SCL), GPIO18 (SDA)
i2c_1 = I2C(1, scl=Pin(19), sda=Pin(18), freq=400000)

def pca9685_init():
    i2c_1.writeto_mem(PCA9685_ADDR, MODE1, b'\x00')  # normal mode
    pca9685_set_pwm_freq(50)  # 50Hz for servo


def pca9685_set_pwm_freq(freq_hz):
    prescale_val = int(25000000.0 / (4096 * freq_hz) - 1)
    old_mode = i2c_1.readfrom_mem(PCA9685_ADDR, MODE1, 1)[0]
    sleep_mode = (old_mode & 0x7F) | 0x10
    i2c_1.writeto_mem(PCA9685_ADDR, MODE1, bytes([sleep_mode]))  # go to sleep
    i2c_1.writeto_mem(PCA9685_ADDR, PRESCALE, bytes(
        [prescale_val]))  # set prescale
    i2c_1.writeto_mem(PCA9685_ADDR, MODE1, bytes([old_mode]))  # wake up
    time.sleep_ms(5)
    i2c_1.writeto_mem(PCA9685_ADDR, MODE1, bytes(
        [old_mode | 0xA1]))  # auto-increment


def pca9685_set_pwm(channel, on, off):
    reg = LED0_ON_L + 4 * channel
    data = bytes([on & 0xFF, on >> 8, off & 0xFF, off >> 8])
    i2c_1.writeto_mem(PCA9685_ADDR, reg, data)

# --- Servo Control ---


def pca9685_set_servo_us(channel, us):
    # 1 tick = (1/50Hz) / 4096 = ~4.88us
    ticks = int(us * 4096 / 20000)
    pca9685_set_pwm(channel, 0, ticks)


def pca9685_set_servo_angle(channel, angle):
    min_us = 500
    max_us = 2500
    delta_us = max_us - min_us
    us_per_degree = delta_us / 180
    us = angle * us_per_degree + min_us
    print(f"angle: {angle}, us: {us}")
    pca9685_set_servo_us(channel, us)


# --- Run it ---
pca9685_init()

try:
    while True:
        # print("Move to min (0°)")
        # set_servo_us(0, 500)
        pca9685_set_servo_angle(0, 10)
        pca9685_set_servo_angle(1, 10)
        time.sleep(0.2)

        # print("Move to center (90°)")
        # set_servo_us(0, 1500)
        pca9685_set_servo_angle(0, 90)
        pca9685_set_servo_angle(1, 60)
        time.sleep(0.5)

        # print("Move to max (180°)")
        # pca9685_set_servo_us(0, 2500)
        # pca9685_set_servo_angle(0, 160)
        # pca9685_set_servo_angle(1, 10)
        # time.sleep(1)

        # print("Move to center (90°)")
        # pca9685_set_servo_us(0, 1500)
        # pca9685_set_servo_angle(0, 90)
        # pca9685_set_servo_angle(1, 60)
        # time.sleep(1)

except KeyboardInterrupt:
    print("Stopped.")
