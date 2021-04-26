# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

# Simple demo of the TCS34725 color sensor.
# Will detect the color from the sensor and print it out every second.
import time
import board
import adafruit_tcs34725


# Create sensor object, communicating over the board's default I2C bus
i2c = board.I2C()  # uses board.SCL and board.SDA
sensor = adafruit_tcs34725.TCS34725(i2c)

# Main loop reading color and printing it every second.
while True:
    # Read the color temperature and lux of the sensor too.
    temp = sensor.color_temperature
    lux = sensor.lux
    print("Temperature: {0}K Lux: {1}".format(temp, lux))
    # Delay for a second and repeat.
    time.sleep(1.0)
