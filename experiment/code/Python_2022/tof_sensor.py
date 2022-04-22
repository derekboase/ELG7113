import numpy as np
import VL53L1X
import time 

from time import sleep

if __name__ == "__main__":
    tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
    tof.open()
    count = 0
    t_arr = []
    tof.start_ranging(0)
    t = time.time_ns()
    while count < 20:
        distance_in_mm = tof.get_distance()
        #print(f'{distance_in_mm} with interval {time.process_time() - t}')
        time_delta = (time.time_ns() - t)*1e-9
        t_arr.append(time_delta)
        print(distance_in_mm)
        count += 1
        t = time.time_ns()
    tof.stop_ranging()
    print(f'{t_arr},\t{len(t_arr)}')
    print(np.array(t_arr).mean())
    print(np.array(t_arr).var())
