import RPi.GPIO as GPIO
#import VL53L1X as VL
import os
import csv
import threading
from time import sleep
import time
import numpy as np
import pandas as pd
import serial as sp

OUTPUT_DATA_PATH = "data.csv"
PWM_PIN = 19 # Can use 12, 13, 19, 24

port_name = '/dev/ttyACM0'
ser = sp.Serial(port_name, 115200, timeout=0.5)

df = pd.read_csv('dataset_1.csv')
lst_duty = list(set(df['duty_cycle']))
lst_duty.sort()
lst_avg = [df['air_vel(m/s)'].loc[df['duty_cycle'] == lst_duty[i]].mean() for i in range(len(lst_duty))]
df_avg = pd.DataFrame(np.array([lst_duty, lst_avg]).T, columns=['duty_cycle','air_vel_avg(m/s)'])
def lookup_cycle(air_vel):
    try:
        lower_air_vels = df_avg[df_avg["air_vel_avg(m/s)"]<air_vel]
        result = list(lower_air_vels["duty_cycle"])[-1]
    except:
        result = 39.22
    return result

def setup():
    global pwm, tof
    #tof = VL.VL53L1X(i2c_bus=1, i2c_address=0x29)

    # Setting up the motor control PWM on pin 19 (BCM)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PWM_PIN, GPIO.OUT)
    GPIO.output(PWM_PIN, GPIO.LOW)
    pwm = GPIO.PWM(PWM_PIN, 1000)
    pwm.start(0)


# def reading_tof():
#     while True:
#         global distance_in_m
#         distance_in_m = 0.38 - tof.get_distance()*1e-3
#         print(distance_in_m)
#         calc_speed()


def changing_pwm(new_val):
    pwm.ChangeDutyCycle(new_val)
    sleep(0.01)


def user_input_pwm():
    '''
    Function prompts the user for a value in the range [0, 100] representing the duty cycle of the pwm.
    This is meant to allow the user to determine the baseline pwm signal that keeps the ball levitating
    at the opening.
    :return: None
    '''
    duty = 0
    while duty >=0:
        duty = float(input('Enter the pwm value: '))
        try:
            pwm.ChangeDutyCycle(duty)
        except ValueError:
            if duty > 100.0:
                print("[Err]\tPick value in range [0.0, 100.0], <0 to kill")
            else:
                return -1
        sleep(0.01)


def kill_pwm():
    pwm.stop(0)
    GPIO.output(PWM_PIN, GPIO.LOW)
    GPIO.cleanup()


#constants
ZETA = 1
OMEGA = 0.7071
ts = 0.4190
#adaptation rate
GAMMA = 0.2
alpha = 0.5
#initial state of system
y_p1_i, y_i, u_i = 0, 0, 0

#initial state of model
ym_p1_i, ym_i = 0, 0

#constants
radius, g, m = 38/1000, 9.8, 4/7000
rho, Vb, v_eq = 1.225, (4/3)*np.pi*(radius**3), 2.82
B = 2*g*(m-rho*Vb)/(m*v_eq)

#init with model following condition and zero for derivatives
theta1_i = -(OMEGA**2)/B
theta2_i = (2*ZETA*OMEGA)/(1+B)
theta1_n_i = theta1_i/(alpha + theta1_i**2)
theta2_n_i = theta2_i/(alpha + theta2_i**2)
theta1_p2_i, theta1_p1_i, theta2_p2_i, theta2_p1_i = 0,0,0,0

def system_model(uc_i, ym_p1_i, ym_i):
    ym_p2_o = (OMEGA**2)*uc_i - 2*ZETA*OMEGA*ym_p1_i - (OMEGA**2)*ym_i
    ym_p1_o = ym_p1_i + ts*ym_p2_o
    ym_o = ym_i + ts*ym_p1_o

    system_model_dict = {"ym_p2":ym_p2_o,
                          "ym_p1":ym_p1_o,
                          "ym":ym_o}
    return system_model_dict

    return system_dict

def controller(y_i, uc_i, y_p1_i, th1, th2):
    control_signal = th1*(y_i - uc_i) - th2*y_p1_i
    return control_signal

def Adaptation_Law_Model(y_i, ym_i, ym_p1_i, ym_p2_i,
                        theta1_p2_i, theta1_p1_i, theta1_i,
                        theta2_p2_i, theta2_p1_i, theta2_i):

    error = y_i - ym_i

    theta1_p3_o = (GAMMA*error/OMEGA**2)*(2*ZETA*OMEGA*ym_p1_i+ym_p2_i) - 2*ZETA*OMEGA*theta1_p2_i - (OMEGA**2)*theta1_p1_i
    theta1_p2_o = theta1_p2_i + ts*theta1_p3_o
    theta1_p1_o = theta1_p1_i + ts*theta1_p2_o
    theta1_o = theta1_i + ts*theta1_p1_o

    theta2_p3_o = GAMMA*error*ym_p1_i - 2*ZETA*OMEGA*theta2_p2_i - (OMEGA**2)*theta2_p1_i
    theta2_p2_o = theta2_p2_i + ts*theta2_p3_o
    theta2_p1_o = theta2_p1_i + ts*theta2_p2_o
    theta2_o = theta2_i + ts*theta2_p1_o

    result_dict = {
                   "theta1_p2": theta1_p2_o,
                   "theta1_p1": theta1_p1_o,
                   "theta1": theta1_o,
                   "theta2_p2": theta2_p2_o,
                   "theta2_p1": theta2_p1_o,
                   "theta2": theta2_o
                  }
    return result_dict

def Adaptation_Law_Normalized(y_i, ym_i, ym_p1_i, ym_p2_i,
                        theta1_p2_i, theta1_p1_i, theta1_i,
                        theta2_p2_i, theta2_p1_i, theta2_i,
                        theta1_p1_n ,theta1_n_i,
                        theta2_p1_n, theta2_n_i):

    error_val = y_i - ym_i

    if error_val !=0:
        error = error_val
    else:
        error = 1/(10**100)

    theta1_p3_o = (GAMMA*error/OMEGA**2)*(2*ZETA*OMEGA*ym_p1_i+ym_p2_i) - 2*ZETA*OMEGA*theta1_p2_i - (OMEGA**2)*theta1_p1_i
    theta1_p2_o = theta1_p2_i + ts*theta1_p3_o
    theta1_p1_o = theta1_p1_i + ts*theta1_p2_o
    theta1_o = theta1_i + ts*theta1_p1_o

    theta1_p1_n = theta1_p1_o/(alpha+(theta1_p1_o/(GAMMA*error))**2)
    theta1_n = theta1_n_i + ts*theta1_p1_n

    theta2_p3_o = GAMMA*error*ym_p1_i - 2*ZETA*OMEGA*theta2_p2_i - (OMEGA**2)*theta2_p1_i
    theta2_p2_o = theta2_p2_i + ts*theta2_p3_o
    theta2_p1_o = theta2_p1_i + ts*theta2_p2_o
    theta2_o = theta2_n_i + ts*theta2_p1_o

    theta2_p1_n = theta2_p1_o/(alpha+(theta2_p1_o/(GAMMA*error))**2)
    theta2_n = theta2_n_i + ts*theta2_p1_n

    result_dict = {
                   "theta1_p2": theta1_p2_o,
                   "theta1_p1": theta1_p1_o,
                   "theta1": theta1_o,
                   "theta1n_p1":theta1_p1_n,
                   "theta1_n":theta1_n,
                   "theta2_p2": theta2_p2_o,
                   "theta2_p1": theta2_p1_o,
                   "theta2": theta2_o,
                   "theta2n_p1":theta1_p1_n,
                   "theta2_n":theta2_n,
                  }
    return result_dict

last2_pos = []
y_p1_i = 0

def calc_speed(pos):
    global last2_pos
    global y_p1_i, _T
    last2_pos.append(pos)

    if len(last2_pos) == 2:
        y_p1_i = (last2_pos[1]-last2_pos[0])/_T

    if len(last2_pos)>2:
        y_p1_i = (last2_pos[2]-last2_pos[1])/_T
        last2_pos.pop(0)


TARGET_POSITION = 0.2
ts = 0
#pwm.ChangeDutyCycle(0)

datafile = open(OUTPUT_DATA_PATH, 'w', newline='')
data_writer = csv.writer(datafile)
data_header = ["timestep","y", "ym", "uc", "u", "theta1", "theta2"]
data_writer.writerow(data_header)
bottom = 0.38
if __name__ == "__main__":
    setup()
    #tof.open()
    #tof.start_ranging(1)
    uc_i = TARGET_POSITION

    try:
#         tof_thread = threading.Thread(target=reading_tof, daemon=True)
#         tof_thread.start()
        #user_input_pwm() # Set baseline pwm. Control/ID goes after
        #pwm.ChangeDutyCycle(0)
        _T = time.process_time()
        while True:
            
            while not ser.inWaiting():
                pass
            y_i = bottom - float(ser.readline().decode('utf-8').rstrip())*1e-3
            
            #y_i = 0.38 - tof.get_distance()*1e-3
            print("position = " +str(y_i))
            calc_speed(y_i)
            print("speed = "+ str(y_p1_i))

            model_results = system_model(uc_i, ym_p1_i, ym_i)
            ym_p2_i = model_results["ym_p2"]
            ym_p1_i = model_results["ym_p1"]
            ym_i = model_results["ym"]

            adapt_val = Adaptation_Law_Model(y_i, ym_i, ym_p1_i, ym_p2_i,
                            theta1_p2_i, theta1_p1_i, theta1_i,
                            theta2_p2_i, theta2_p1_i, theta2_i)
            theta1_p2_i, theta1_p1_i, theta1_i = adapt_val["theta1_p2"], adapt_val["theta1_p1"], adapt_val["theta1"]
            theta2_p2_i, theta2_p1_i, theta2_i = adapt_val["theta2_p2"], adapt_val["theta2_p1"], adapt_val["theta2"]

            u_i =  controller(y_i, uc_i, y_p1_i, theta1_i, theta2_i)
            vf = u_i + v_eq
            duty_cycle = lookup_cycle(vf)
            print("ui = " + str(u_i))
            print("duty_cycle = " +str(duty_cycle))
            print(" ")
            print(time.process_time() - _T)
            _T = time.process_time()

            pwm.ChangeDutyCycle(duty_cycle)

            data_row = [ts, y_i, ym_i, uc_i, u_i, theta1_i, theta2_i]
            data_writer.writerow(data_row)
            ts = ts + 1


    except KeyboardInterrupt:
        #tof.stop_ranging() # This gets run once the code is stopped
        kill_pwm()

#normalized MIT rule
# if __name__ == "__main__":
    # setup()
    # tof.open()
    # tof.start_ranging(1)
    # uc_i = TARGET_POSITION

    # try:
# #         tof_thread = threading.Thread(target=reading_tof, daemon=True)
# #         tof_thread.start()
        # #user_input_pwm() # Set baseline pwm. Control/ID goes after
        # #pwm.ChangeDutyCycle(0)
        # _T = time.process_time()
        # while True:
            # y_i = 0.38 - tof.get_distance()*1e-3
            # print("position = " +str(y_i))
            # calc_speed(y_i)
            # print("speed = "+ str(y_p1_i))

            # model_results = system_model(uc_i, ym_p1_i, ym_i)
            # ym_p2_i = model_results["ym_p2"]
            # ym_p1_i = model_results["ym_p1"]
            # ym_i = model_results["ym"]

            # adapt_val = Adaptation_Law_Normalized(y_i, ym_i, ym_p1_i, ym_p2_i,
                                    # theta1_p2_i, theta1_p1_i, theta1_i,
                                    # theta2_p2_i, theta2_p1_i, theta2_i,
                                    # theta1_p1_n, theta1_n_i,
                                    # theta2_p1_n, theta2_n_i)

            # theta1_p2_i, theta1_p1_i, theta1_i = adapt_val["theta1_p2"], adapt_val["theta1_p1"], adapt_val["theta1"]
            # theta2_p2_i, theta2_p1_i, theta2_i = adapt_val["theta2_p2"], adapt_val["theta2_p1"], adapt_val["theta2"]
            # theta1_p1_n, theta1_n_i = adapt_val["theta1n_p1"], adapt_val["theta1_n"]
            # theta2_p1_n, theta2_n_i = adapt_val["theta2n_p1"], adapt_val["theta2_n"]

            # u_i =  controller(y_i, uc_i, y_p1_i, theta1_n_i, theta2_n_i)
            # vf = u_i + v_eq
            # duty_cycle = lookup_cycle(vf)
            # print("ui = " + str(u_i))
            # print("duty_cycle = " +str(duty_cycle))
            # print(" ")
            # print(time.process_time() - _T)
            # _T = time.process_time()

            # pwm.ChangeDutyCycle(duty_cycle)

            # data_row = [ts, y_i, ym_i, uc_i, u_i, theta1_i, theta2_i]
            # writer.writerow(data_row)
            # ts = ts + 1

    # except KeyboardInterrupt:
        # tof.stop_ranging() # This gets run once the code is stopped
        # kill_pwm()
