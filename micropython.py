from micropython import const
from machine import SoftI2C, Pin
import ustruct
import time

# ADXL-345 Registers
_DATA_FORMAT = const(0x31)
_POWER_CTL = const(0x2D)
_DATAX0 = const(0x32)

# Range values
RANGE_2G = const(0x00)
RANGE_4G = const(0x01)
RANGE_8G = const(0x02)
RANGE_16G = const(0x03)

# Threshold for detecting movement (in g)
THRESHOLD = 0.07

class ADXL345:
    def __init__(self, i2c, address=0x53):
        self.i2c = i2c
        self.address = address
        self._initialize_sensor()
        self.x_history = []
        self.y_history = []
        self.z_history = []
        self.previous_avg = (0, 0, 0)
        self.x_offset, self.y_offset, self.z_offset = self._calibrate()
        self.initial_direction = {'x': None, 'y': None, 'z': None}
        self.crossed_start_point = {'x': False, 'y': False, 'z': False}

    def _initialize_sensor(self):
        self.i2c.start()
        self.i2c.writeto(self.address, bytearray([_POWER_CTL, 0x08]))  # power on
        self.set_range(RANGE_2G)  # Set default range to ±2g
        self.i2c.stop()
        time.sleep(0.1)  # Small delay to ensure the commands are processed

    def set_range(self, range_val):
        current_data_format = self.i2c.readfrom_mem(self.address, _DATA_FORMAT, 1)[0]
        new_data_format = (current_data_format & ~0x0F) | (range_val & 0x03) | 0x08  # Setting full resolution and range
        self.i2c.writeto(self.address, bytearray([_DATA_FORMAT, new_data_format]))
        time.sleep(0.1)  # Small delay to ensure the commands are processed

    def _update_history(self, x, y, z):
        if len(self.x_history) >= 3:
            self.x_history.pop(0)
            self.y_history.pop(0)
            self.z_history.pop(0)

        self.x_history.append(x)
        self.y_history.append(y)
        self.z_history.append(z)

    def _average_last_three(self, history):
        if len(history) < 3:
            return history[-1]
        return sum(history) // len(history)

    def _calibrate(self):
        print("Calibrating...")
        x_sum, y_sum, z_sum = 0, 0, 0
        num_samples = 100
        for _ in range(num_samples):
            x, y, z = self.read_raw()
            x_sum += x
            y_sum += y
            z_sum += z
            time.sleep(0.01)  # Small delay between readings

        x_offset = x_sum // num_samples
        y_offset = y_sum // num_samples
        z_offset = z_sum // num_samples

        print("Calibration done.")
        return x_offset, y_offset, z_offset

    def read_raw(self):
        self.i2c.start()
        self.i2c.writeto(self.address, bytearray([_DATAX0]))
        data = self.i2c.readfrom_mem(self.address, _DATAX0, 6)
        self.i2c.stop()
        
        if len(data) != 6:
            raise RuntimeError("Failed to read accelerometer data")

        x, y, z = ustruct.unpack('<3h', data)
        return x, y, z

    def read(self):
        x, y, z = self.read_raw()

        # Apply calibration offsets
        x -= self.x_offset
        y -= self.y_offset
        z -= self.z_offset

        self._update_history(x, y, z)

        avg_x = self._average_last_three(self.x_history)
        avg_y = self._average_last_three(self.y_history)
        avg_z = self._average_last_three(self.z_history)

        return avg_x, avg_y, avg_z

    def detect_direction(self, axis, delta, axis_name):
        direction = None
        if abs(delta) > THRESHOLD:
            if self.crossed_start_point[axis_name]:
                if (delta > 0 and self.initial_direction[axis_name] == "negative") or (delta < 0 and self.initial_direction[axis_name] == "positive"):
                    self.crossed_start_point[axis_name] = False
                    direction = axis_name
                    self.initial_direction[axis_name] = None  # Reset initial direction
            elif not self.initial_direction[axis_name]:
                self.initial_direction[axis_name] = "positive" if delta > 0 else "negative"
            elif (delta < 0 and self.initial_direction[axis_name] == "positive") or (delta > 0 and self.initial_direction[axis_name] == "negative"):
                self.crossed_start_point[axis_name] = True
        if direction:
            return f"{direction} ({'positive' if delta > 0 else 'negative'})"
        return None

    def get_movement_direction(self):
        current_avg = self.read()
        delta_x = (current_avg[0] - self.previous_avg[0]) / 256.0
        delta_y = (current_avg[1] - self.previous_avg[1]) / 256.0
        delta_z = (current_avg[2] - self.previous_avg[2]) / 256.0

        direction_x = self.detect_direction('x', delta_x, 'x')
        direction_y = self.detect_direction('y', delta_y, 'y')
#        direction_z = self.detect_direction('z', delta_z, 'z')

        self.previous_avg = current_avg

        return direction_x or direction_y #or direction_z

# Example usage:
i2c = SoftI2C(scl=Pin(7), sda=Pin(8))
accelerometer = ADXL345(i2c)
button = Pin(6, Pin.IN) 
# Set the range to ±8g
accelerometer.set_range(RANGE_8G)

while True:
    direction = accelerometer.get_movement_direction()
    avg_x, avg_y, avg_z = accelerometer.read()
#    if direction:
        #print(f"Movement detected: {direction}")
    if(avg_x > 120):
        print("+x")
    if(avg_y > 120):
        print("+y")
        
    if(avg_y < -120):
        print("-y")
    if(avg_x < -120):
        print("-x")
    stat =button.value()
    time.sleep(0.05)
    #print(f"X: {avg_x:>6} | Y: {avg_y:>6} | Z: {avg_z:>6}")

