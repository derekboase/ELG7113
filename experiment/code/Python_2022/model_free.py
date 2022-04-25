## Library import
import matplotlib.pyplot as plt
import RPi.GPIO as GPIO
import poly_fit as pf
import look_up as lu
import control as co
import pandas as pd
import serial as sp
import numpy as np 
import time

from numpy.linalg import inv

if __name__ == "__main__":
    try:
        D_MIN = 0
        D_MAX = 100
        D_EQ = 71
        PWM_PIN = 19 # Using BCM enumeration
        FINAL_TIME = 60
        TS_VAL = 0.510
        t = np.arange(0, FINAL_TIME + TS_VAL, TS_VAL)
        LOWER_BOUND = -5
        UPPER_BOUND = 5

        def reference_signal(end_time=FINAL_TIME, Ts_func=TS_VAL, lower_set=0.2, upper_set=0.15, period=60):
            uc_func = []
            time = np.arange(0, end_time + Ts_func, Ts_func)
            for _t in time:
                rat = 2*np.pi/period
                if np.sin(rat*_t) >= 0:
                    uc_func.append(upper_set)
                else:
                    uc_func.append(lower_set)
            return np.array(uc_func, float) 

        def setup():
            global pwm    
            # Setting up the motor control PWM on pin 19 (BCM)
            try:
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(PWM_PIN, GPIO.OUT)
                GPIO.output(PWM_PIN, GPIO.LOW)
                pwm = GPIO.PWM(PWM_PIN, 1000)
                pwm.start(D_EQ)
            except RuntimeWarning:
                kill_pwm()
                print('PWM killed')
                time.sleep(1)
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(PWM_PIN, GPIO.OUT)
                GPIO.output(PWM_PIN, GPIO.LOW)
                pwm = GPIO.PWM(PWM_PIN, 1000)
                pwm.start(D_EQ)

        def generate_SPD_matrix(n):
            """
            Returns
            :param n: Number of dimensions for the symmetric positive definite matrix
            :return: np array of dimensions nxn
            """
            A = np.random.rand(n, n)
            return 1/2.0 * A@A.T
                    
        def bound(low, high, val):
            return np.array([max(low, min(high, val))])
        
        def baseline():
            ser.reset_input_buffer()
            baseline = []
            for j in range(50):
                while not ser.inWaiting():
                    pass # NOP
                height = np.array([float(ser.readline().decode('utf-8').rstrip())*1e-3])
                baseline.append(height)
            return np.array(baseline).mean()
            
        def changing_pwm(new_val):
            try:
                pwm.ChangeDutyCycle(new_val)
            except ValueError:
                if new_val > 100.0:
                    print("[Err]\tVal too large")  
                    pwm.ChangeDutyCycle(100.0)
                elif new_val < 0.0:
                    print("[Err]\tVal too small")
                    pwm.ChangeDutyCycle(0.0)
                else:
                    print(f"[Err]\tUnexpected, pwm_val={new_val}")
            
        def kill_pwm():
            pwm.stop()
            GPIO.output(PWM_PIN, GPIO.LOW)
            GPIO.cleanup()

        ## Instantiating the serial object and clearing the PWM channel

        port_name = '/dev/ttyACM0'
        ser = sp.Serial(port_name, 115200, timeout=0.5)        
        
        ## Setup, baseline and model definition

        setup()
        input('Place the ball in the bottom. Press ENTER to continue...')
        bottom = baseline()
        print(f'The baseline reading is {bottom}')
        changing_pwm(D_EQ)
        ym = reference_signal()

        ## Control Algorithm

        N = FINAL_TIME / TS_VAL
        Z_ACTOR = 0.01
        Z_CRITIC = 0.05
        Q_1 = generate_SPD_matrix(3)
        R_1 = generate_SPD_matrix(1)

        # Q_1 = np.array([[0.51502986, 0.25789362, 0.06580822],
                    # [0.25789362, 0.19214249, 0.0747135],
                    # [0.06580822, 0.0747135, 0.0378436]])
        # R_1 = np.array([[0.07450969]])
        
        Q_init = Q_1
        R_init = R_1

        k, converge = 1, False  # Index and convergence flag
        E_k_1 = np.zeros((3, 1))  # Current quadratic error vector shape(3, 1)
        E_k1_1 = np.zeros((3, 1))  # Future quadratic error vector shape(3, 1)
        
        Wc_1 = generate_SPD_matrix(4)  # Initial critic matrix shape(4, 4)
        Wc_init = Wc_1

        # Wc_1 = np.array([[ 0.80349833,  0.30936819,  0.84494049,  0.71454207],
                         # [ 0.30936819,  0.21330422,  0.31156708,  0.36979277],
                         # [ 0.84494049,  0.31156708,  1.09927468,  0.53434843],
                         # [ 0.71454207,  0.36979277,  0.53434843,  0.9285541 ]])

        Wa_1 = (1/Wc_1[3][3]*Wc_1[3][0:3]).reshape(1, 3)  # Initial actor matrix shape(1, 3) 
        Wa_1[0, 1] *= -1 # Negating the middle element (to add stability "pole" in the actor vector)
        Wa_1_1, Wa_1_2, Wa_1_3 = [], [], []
        u_hat_1_arr = []
        pwm_val = D_EQ
        y_measure = np.array([0])
        
        print('*******************************************************')
        print('\t\tSTARTING CODE')
        print('*******************************************************')
        time.sleep(5)
        time_arr = []
        time_ns = time.time_ns()
        while k < len(t) and not converge:
            Wa_1_1.append(Wa_1[0, 0])
            Wa_1_2.append(Wa_1[0, 1])
            Wa_1_3.append(Wa_1[0, 2])
            
            u_hat_1 = bound(LOWER_BOUND, UPPER_BOUND, (Wa_1@E_k_1).reshape(1, ))  # shape(1,) REPLACE WITH (Wa_1@E_k_1).reshape(1, )
            u_hat_1_arr.append(u_hat_1)
            pwm_val += u_hat_1
            changing_pwm(pwm_val)
            
            # Find V and U: Step 8 
            Eu_concat_1 = np.concatenate((E_k_1, u_hat_1.reshape(-1, 1)), axis=0)
            V_k_1 = 1/2.0 * (Eu_concat_1.T@Wc_1)@Eu_concat_1
            eqe_1 = (E_k_1.T@Q_1)@E_k_1
            uru_1 = u_hat_1*R_1*u_hat_1
            U_k_1 = 1 / 2.0 * (eqe_1 + uru_1)

                # Measurement
            ser.reset_input_buffer()
            while not ser.inWaiting():
                pass
            height = bottom - np.array([float(ser.readline().decode('utf-8').rstrip())*1e-3])
            ser.reset_input_buffer()
            y_measure = np.concatenate((y_measure,
                                        (height).reshape(-1,))) 

            # Get E(k + 1), u_hat and V(k_1): Step 9 
            E_k1_1[2] = E_k_1[1]
            E_k1_1[1] = E_k_1[0]
            E_k1_1[0] = ym[k] - y_measure[k]

            u_hat_k1_1 = bound(LOWER_BOUND, UPPER_BOUND, (Wa_1@E_k1_1).reshape(1, ))  # shape(1,)
            Z_1 = np.concatenate((E_k1_1, u_hat_k1_1.reshape(-1, 1)), axis=0)
            V_k1_1 = 1/2.0*(Z_1.T@Wc_1)@Z_1

            # Update critic weights: Step 11 
            temp = Z_CRITIC*(V_k_1 - (U_k_1 + V_k1_1))
            Wc_1 -= temp*(Z_1@Z_1.T)
            Wa_1 -= Z_ACTOR*(Wa_1@E_k_1 - (-1/Wc_1[3][3]*(Wc_1[3][0:3]@E_k_1)))*E_k_1.T

            # Updates
            E_k_1 = E_k1_1            
            k += 1
            time_delta = (time.time_ns() - time_ns)*1e-9
            time_arr.append(time_delta)
            time_ns = time.time_ns()
            print(f'y={y_measure[-1]}\t,u_hat={u_hat_1}')
        
        print(Q_init)
        print(R_init)
        print(Wc_1)
        print(Wa_1)

        input('Press ENTER to see graphs')

        et = len(t)
        plt.plot(t[0:et], y_measure[0:et])
        plt.plot(t[0:et], ym[0:et])
        plt.show()
        
        plt.plot(t[0:et], u_hat_1_arr[1:et])
        plt.show()
        
        plt.plot(t[0:et], Wa_1_1[0:et])
        plt.plot(t[0:et], Wa_1_2[0:et])
        plt.plot(t[0:et], Wa_1_3[0:et])

        input('Press ENTER to kill.')
        kill_pwm()
        
    except KeyboardInterrupt:
        kill_pwm()
