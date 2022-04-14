import numpy as np
import VL53L1X
import time 

from time import sleep

if __name__ == "__main__":
    tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
    tof.open()
    count = 0
    t = time.process_time()
    t_arr = []
    while count < 20:
        tof.start_ranging(1)
        distance_in_mm = tof.get_distance()
        count += 1
        #print(f'{distance_in_mm} with interval {time.process_time() - t}')
        print(distance_in_mm)
        t_arr.append(time.process_time() - t)
        t = time.process_time()
    tof.stop_ranging()
    print(f'{t_arr},\t{len(t_arr)}')
    print(np.array(t_arr).mean())
    print(np.array(t_arr).var())
