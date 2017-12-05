# The MIT License (MIT)
#
# Copyright (c) 2017 Tony DiCola for Adafruit Industries
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
"""
`adafruit_tcs34725`
====================================================

CircuitPython module for the TCS34725 color sensor.  Ported from the
micropython-adafruit-tcs34725 module by Radomir Dopieralski:
  https://github.com/adafruit/micropython-adafruit-tcs34725

See examples/simpletest.py for an example of the usage.

* Author(s): Tony DiCola
"""
import time
import ustruct

import adafruit_bus_device.i2c_device as i2c_device


# Register and command constants:
_COMMAND_BIT       = const(0x80)
_REGISTER_ENABLE   = const(0x00)
_REGISTER_ATIME    = const(0x01)
_REGISTER_AILT     = const(0x04)
_REGISTER_AIHT     = const(0x06)
_REGISTER_ID       = const(0x12)
_REGISTER_APERS    = const(0x0c)
_REGISTER_CONTROL  = const(0x0f)
_REGISTER_SENSORID = const(0x12)
_REGISTER_STATUS   = const(0x13)
_REGISTER_CDATA    = const(0x14)
_REGISTER_RDATA    = const(0x16)
_REGISTER_GDATA    = const(0x18)
_REGISTER_BDATA    = const(0x1a)
_ENABLE_AIEN       = const(0x10)
_ENABLE_WEN        = const(0x08)
_ENABLE_AEN        = const(0x02)
_ENABLE_PON        = const(0x01)
_GAINS  = (1, 4, 16, 60)
_CYCLES = (0, 1, 2, 3, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60)


class TCS34725:

    # Class-level buffer for reading and writing data with the sensor.
    # This reduces memory allocations but means the code is not re-entrant or
    # thread safe!
    _BUFFER = bytearray(3)

    def __init__(self, i2c, address=0x29):
        self._device = i2c_device.I2CDevice(i2c, address)
        sensor_id = self._read_u8(_REGISTER_SENSORID)
        self._active = False
        self.integration_time = 2.4
        # Check sensor ID is expectd value.
        sensor_id = self._read_u8(_REGISTER_SENSORID)
        if sensor_id not in (0x44, 0x10):
            raise RuntimeError('Could not find sensor, check wiring!')

    def _read_u8(self, address):
        # Read an 8-bit unsigned value from the specified 8-bit address.
        with self._device as i2c:
            self._BUFFER[0] = (address | _COMMAND_BIT) & 0xFF
            i2c.write(self._BUFFER, end=1, stop=False)
            i2c.readinto(self._BUFFER, end=1)
        return self._BUFFER[0]

    def _read_u16(self, address):
        # Read a 16-bit BE unsigned value from the specified 8-bit address.
        with self._device as i2c:
            self._BUFFER[0] = (address | _COMMAND_BIT) & 0xFF
            i2c.write(self._BUFFER, end=1, stop=False)
            i2c.readinto(self._BUFFER, end=2)
        return (self._BUFFER[0] << 8) | self._BUFFER[1]

    def _write_u8(self, address, val):
        # Write an 8-bit unsigned value to the specified 8-bit address.
        with self._device as i2c:
            self._BUFFER[0] = (address | _COMMAND_BIT) & 0xFF
            self._BUFFER[1] = val & 0xFF
            i2c.write(self._BUFFER, end=2)

    def _write_u16(self, address, val):
        # Write a 16-bit BE unsigned value to the specified 8-bit address.
        with self._device as i2c:
            self._BUFFER[0] = (address | _COMMAND_BIT) & 0xFF
            self._BUFFER[1] = (val >> 8) & 0xFF
            self._BUFFER[2] = val & 0xFF
            i2c.write(self._BUFFER)

    @property
    def active(self):
        """Get and set the active state of the sensor.  Boolean value that will
        enable/activate the sensor with a value of True and disable with a
        value of False.
        """
        return self._active

    @active.setter
    def active(self, val):
        val = bool(val)
        if self._active == val:
            return
        self._active = val
        enable = self._read_u8(_REGISTER_ENABLE)
        if val:
            self._write_u8(_REGISTER_ENABLE, enable | _ENABLE_PON)
            time.sleep(0.003)
            self._write_u8(_REGISTER_ENABLE, enable | _ENABLE_PON | _ENABLE_AEN)
        else:
            self._write_u8(_REGISTER_ENABLE,
                           enable & ~(_ENABLE_PON | _ENABLE_AEN))

    @property
    def integration_time(self):
        """Get and set the integration time of the sensor in milliseconds."""
        return self._integration_time

    @integration_time.setter
    def integration_time(self, val):
        assert 2.4 <= val <= 614.4
        cycles = int(val / 2.4)
        self._integration_time = cycles * 2.4
        self._write_u8(_REGISTER_ATIME, 256-cycles)

    @property
    def gain(self):
        """Get and set the gain of the sensor.  Should be a value of 1, 4, 16,
        or 60.
        """
        return _GAINS[self._read_u8(_REGISTER_CONTROL)]

    @gain.setter
    def gain(self, val):
        assert val in _GAINS
        self._write_u8(_REGISTER_CONTROL, _GAINS.index(val))

    @property
    def interrupt(self):
        """Get and clear the interrupt of the sensor.  Returns a bool that's
        True if the interrupt is set.  Can be set to False (and only False)
        to clear the interrupt.
        """
        return bool(self._read_u8(_REGISTER_STATUS) & _ENABLE_AIEN)

    @interrupt.setter
    def interrupt(self, val):
        assert not val
        with self._device:
            self._device.write(b'\xe6')

    def _valid(self):
        # Check if the status bit is set and the chip is ready.
        return bool(self._read_u8(_REGISTER_STATUS) & 0x01)

    @property
    def color_raw(self):
        """Read the raw RGBC color detected by the sensor.  Returns a 4-tuple of
        16-bit red, green, blue, clear component byte values (0-65535).
        """
        was_active = self.active
        self.active = True
        while not self._valid():
            time.sleep((self._integration_time + 0.9)/1000.0)
        data = tuple(self._read_u16(reg) for reg in (
            _REGISTER_RDATA,
            _REGISTER_GDATA,
            _REGISTER_BDATA,
            _REGISTER_CDATA,
        ))
        self.active = was_active
        return data

    @property
    def color_rgb_bytes(self):
        """Read the RGB color detected by the sensor.  Returns a 3-tuple of
        red, green, blue component values as bytes (0-255).
        """
        r, g, b, c = self.color_raw
        red   = int(pow((int((r/c) * 256) / 255), 2.5) * 255)
        green = int(pow((int((g/c) * 256) / 255), 2.5) * 255)
        blue  = int(pow((int((b/c) * 256) / 255), 2.5) * 255)
        return (red, green, blue)

    def temperature_and_lux(self, data):
        """Convert the 4-tuple of raw RGBC data to color temperature and lux values. Will return a 2-tuple of color temperature, lux.
        """
        r, g, b, c = data
        x = -0.14282 * r + 1.54924 * g + -0.95641 * b
        y = -0.32466 * r + 1.57837 * g + -0.73191 * b
        z = -0.68202 * r + 0.77073 * g +  0.56332 * b
        d = x + y + z
        n = (x / d - 0.3320) / (0.1858 - y / d)
        cct = 449.0 * n**3 + 3525.0 * n**2 + 6823.3 * n + 5520.33
        return cct, y

    @property
    def temperature(self):
        """Return the detected color temperature in degrees."""
        temp, lux = self.temperature_and_lux(self.color_raw)
        return temp

    @property
    def lux(self):
        """Return the detected light level in lux."""
        temp, lux = self.temperature_and_lux(self.color_raw)
        return lux

    @property
    def cycles(self):
        """Get and set the persistence cycles of the sensor."""
        if self._read_u8(_REGISTER_ENABLE) & _ENABLE_AIEN:
            return _CYCLES[self._read_u8(_REGISTER_APERS) & 0x0f]
        else:
            return -1

    @cycles.setter
    def cycles(self, val):
        enable = self._read_u8(_REGISTER_ENABLE)
        if val == -1:
            self._write_u8(_REGISTER_ENABLE, enable & ~(_ENABLE_AIEN))
        else:
            assert val in _CYCLES
            self._write_u8(_REGISTER_ENABLE, enable | _ENABLE_AIEN)
            self._write_u8(_REGISTER_APERS, _CYCLES.index(val))

    @property
    def min_value(self):
        """Get and set the minimum threshold value (AILT register) of the
        sensor as a 16-bit unsigned value.
        """
        return self._read_u16(_REGISTER_AILT)

    @min_value.setter
    def min_value(self, val):
        self._write_u16(_REGISTER_AILT, val)

    @property
    def max_value(self):
        """Get and set the minimum threshold value (AIHT register) of the
        sensor as a 16-bit unsigned value.
        """
        return self._read_u16(_REGISTER_AIHT)

    @min_value.setter
    def max_value(self, val):
        self._write_u16(_REGISTER_AIHT, val)
