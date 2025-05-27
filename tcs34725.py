from machine import Pin, I2C
import time

# TCS34725 constants
TCS34725_ADDRESS = 0x29
COMMAND_BIT = 0x80

ENABLE = 0x00
ATIME = 0x01
CONTROL = 0x0F
ID = 0x12
CDATA = 0x14

ENABLE_PON = 0x01
ENABLE_AEN = 0x02

GAIN_16X = 0x02

# PCA9685 constants
MODE1 = 0x00
PRESCALE = 0xFE
LED0_ON_L = 0x06

# i2c_0 setup on GPIO5 (SCL), GPIO4 (SDA)
i2c_0 = I2C(0, scl=Pin(5), sda=Pin(4), freq=400000)


def tcs34725_write8(reg, val):
    i2c_0.writeto_mem(TCS34725_ADDRESS, COMMAND_BIT | reg, bytes([val]))


def tcs34725_read8(reg):
    return int.from_bytes(i2c_0.readfrom_mem(TCS34725_ADDRESS, COMMAND_BIT | reg, 1), 'little')


def tcs34725_read16(reg):
    data = i2c_0.readfrom_mem(TCS34725_ADDRESS, COMMAND_BIT | reg, 2)
    return data[1] << 8 | data[0]


def tcs34725_init():
    sensor_id = tcs34725_read8(ID)
    print("Sensor ID = 0x{:02X}".format(sensor_id))  # No ID check

    tcs34725_write8(ENABLE, ENABLE_PON)
    time.sleep(0.01)
    tcs34725_write8(ENABLE, ENABLE_PON | ENABLE_AEN)

    tcs34725_write8(ATIME, 0x00)         # Max integration time
    tcs34725_write8(CONTROL, GAIN_16X)   # Set gain


def tcs34725_read_colors():
    time.sleep(0.7)  # Wait for integration
    c = tcs34725_read16(CDATA)
    r = tcs34725_read16(CDATA + 2)
    g = tcs34725_read16(CDATA + 4)
    b = tcs34725_read16(CDATA + 6)
    return r, g, b, c


def tcs34725_normalize_rgb(r, g, b, c):
    if c == 0:
        return 0, 0, 0
    return (r * 255) // c, (g * 255) // c, (b * 255) // c


def tcs34725_classify_color(r, g, b):
    # Basic threshold-based classification
    if r > 160 and g < 80 and b < 50:
        return "red", 0
    elif r < 100 and g > 100 and b < 80:
        return "green", 90
    elif r < 100 and g < 120 and b > 90:
        return "blue", 180
    else:
        return "unknown", 90


# Main
tcs34725_init()

try:
    while True:
        r, g, b, c = tcs34725_read_colors()
        nr, ng, nb = tcs34725_normalize_rgb(r, g, b, c)
        color, angle = tcs34725_classify_color(nr, ng, nb)

        # print("Raw: R={}, G={}, B={}, C={}".format(r, g, b, c))
        print("Normalized: R={}, G={}, B={}".format(nr, ng, nb))
        print(f"Detected color: {color}, setting angle: {angle}")
        print("-" * 30)
        time.sleep(0.3)
except KeyboardInterrupt:
    print("Execution stopped.")
