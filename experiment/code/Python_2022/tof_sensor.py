import numpy as np
import VL53L1X

from time import sleep

if __name__ == "__main__":
	tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
	tof.open()
	count = 0
	while count < 20:
		tof.start_ranging(1)
		distance_in_mm = tof.get_distance()
		print(distance_in_mm)
		count += 1
	tof.stop_ranging()
