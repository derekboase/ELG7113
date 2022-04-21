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
        veq = 2.78 # m/s
        
        # filepath = '../fan_charac/datasets/v_look_up_table.csv'
        # VL = lu.v_lut(filepath, v_eq=veq, v_min=2.5, v_max = 2.85)
        
        VL = pf.v_polyfit(v_eq=veq, v_min=2.5, v_max = 2.9)

        ## Functions

        PWM_PIN = 19 # Can use 12, 13, 19, 24
        D_EQ = VL.vel2duty(0)
        print(VL.vel2duty(0))

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

        def baseline():
            time.sleep(2)
            for i in range(20):
                ser.readline()
            print('Taken out trash')
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
                if pwm_val > 100.0:
                    print("[Err]\tVal too large")  
                    pwm.ChangeDutyCycle(100.0)
                elif pwm_val < 0.0:
                    print("[Err]\tVal too small")
                    pwm.ChangeDutyCycle(0.0)
                else:
                    print(f"[Err]\tUnexpected, pwm_val={pwm_val}")
            
        def kill_pwm():
            pwm.stop()
            GPIO.output(PWM_PIN, GPIO.LOW)
            GPIO.cleanup()

        ## Instantiating the serial object and clearing the PWM channel

        port_name = '/dev/ttyACM0'
        ser = sp.Serial(port_name, 115200, timeout=0.5)

        ## Constants from the nominal model 

        radius = 38/2000  # 38 mm diameter to radius in m
        mass = 4/7000  # grams per ball
        area_ball = np.pi * radius**2
        volume_ball = 4/3*np.pi*radius**3
        density = 1.2  # kg/m^3

        Ts_val = 0.4190
        b_nom = 2*9.81*(mass-density*volume_ball)/(mass*veq)

        omega_n = 0.7
        zeta = 1.25

        ## Coefficients from the nominal pulse function

        B, A = co.tfdata(co.sample_system(co.tf([b_nom], [1, b_nom, 0]), method='zoh', Ts=Ts_val))
        pulse_coeffs = A[0][0][1:].tolist()
        for Bi in B[0][0]:
            pulse_coeffs.append(Bi)

        ## Coefficients from the nominal model pulse function

        Bmz_tf, Amz_tf = co.tfdata(co.sample_system(co.tf([1], [1, 2*zeta*omega_n, omega_n**2]), method='zoh', Ts=Ts_val))
        AM1 = Amz_tf[0][0][1]
        AM2 = Amz_tf[0][0][2]
        A0 = 0.5
        T0_num = AM1 + AM2 + 1
        T1_num = A0*(T0_num)
        lam = 1
        # initial_P_weights = [1000]*4
        initial_P_weights = [100, 100, 0.1, 0.1]
        # theta = np.array(pulse_coeffs, float).reshape(4, -1) ## ONLY FOR SIM

        ## Reference signal information

        final_time = 20
        t = np.arange(0, final_time + Ts_val, Ts_val)
        def reference_signal(end_time=final_time, Ts_func=Ts_val, lower_set=0.1, upper_set=0.2, period=40):
            uc_func = []
            time = np.arange(0, end_time + Ts_func, Ts_func)
            for _t in time:
                rat = 2*np.pi/period
                if np.sin(rat*_t) >= 0:
                    uc_func.append(upper_set)
                else:
                    uc_func.append(lower_set)
            return np.array(uc_func, float) 
        uc_val = reference_signal()

        ## Beginning of the control algorithm

        setup()
        input('Place the ball in the bottom. Press ENTER to continue...')
        bottom = baseline()
        print(f'The baseline reading is {bottom}')
        changing_pwm(72.0)
        time.sleep(2)

            # Estimates k = 0
        # pulse_coeffs = [] 
        theta_hat = np.array(pulse_coeffs, float).reshape(4, -1)
        theta_arr = theta_hat
        P = np.diag(initial_P_weights)
        phi = np.zeros((4,1))  ## CONSIDER MOVING UP

            # Measurements and control parameters k = 0
        print('*******************************************************')
        print('\t\tSTARTING CODE')
        print(f'\t\tCoeffs={pulse_coeffs}')
        print('*******************************************************')
        time.sleep(2.5)
        ser.reset_input_buffer()
        # Measurement
        while not ser.inWaiting():
            pass
        height = bottom - np.array([float(ser.readline().decode('utf-8').rstrip())*1e-3])
        y_measure = height.reshape(-1, )

        # Control parameters
        a1, a2, b0, b1 = theta_hat[0], theta_hat[1], theta_hat[2], theta_hat[3]
        den_rs = ((-a1*b0*b1) + (a2*b0**2) + b1**2)   
        den_t = b0 + b1
        r0_val = 1/den_rs*((A0*AM2)*b0**2 + (-a1 + A0 + AM1)*b1**2 + (a2 - A0*AM1 - AM2)*b0*b1)
        s0_val = 1/den_rs*((-a1*a2 + a2*(A0 + AM1) - A0*AM2)*b0 + (a1**2 - a1*(A0 + AM1) - a2 + A0*AM1 + AM2)*b1)
        s1_val = 1/den_rs*((-a2**2 + A0*(a2*AM1 - a1*AM2) + a2*AM2)*b0 + (a2*(a1 - A0 - AM1) + A0*AM2)*b1)
        t0_val = T0_num/den_t
        t1_val = T1_num/den_t

        M = np.array([r0_val, s0_val, s1_val, t0_val, t1_val], float).reshape(-1, 1)
        N = np.array([0, -y_measure[0], 0, uc_val[0], 0], float).reshape(M.shape)
        u_val = (N.T@M).reshape(-1, )

        ## HERE IS WHERE WE NEED THE LOOKUP TABLE AND THE PWM CALL
        pwm_val = VL.vel2duty(u_val[0])
        # pwm_val = 71.5 + u_val[0] # For testing without lookup table
        changing_pwm(pwm_val)


            # Estimates k = 1
        phi = np.array([-y_measure[0], 0, u_val[0], 0], float).reshape(-1,1) # phi of 0
        K = P@phi@inv(lam + phi.T@P@phi)
        theta_hat = theta_hat + K@(-y_measure[0] - phi.T@theta_hat)
        theta_arr = np.concatenate((theta_arr, 
                                    theta_hat.reshape(-1, 1)), axis=1)
        P = (np.eye(len(phi)) - K@phi.T) @P/lam

            # Measurements and control parameters k = 1
        # Measurement
        while not ser.inWaiting():
            pass
        height = bottom - np.array([float(ser.readline().decode('utf-8').rstrip())*1e-3])
        y_measure = np.concatenate((y_measure,
                                    (height).reshape(-1,)))   

        # Control parameters
        a1, a2, b0, b1 = theta_hat[0], theta_hat[1], theta_hat[2], theta_hat[3]
        den_rs = ((-a1*b0*b1) + (a2*b0**2) + b1**2)   
        den_t = b0 + b1
        r0_val = 1/den_rs*((A0*AM2)*b0**2 + (-a1 + A0 + AM1)*b1**2 + (a2 - A0*AM1 - AM2)*b0*b1)
        s0_val = 1/den_rs*((-a1*a2 + a2*(A0 + AM1) - A0*AM2)*b0 + (a1**2 - a1*(A0 + AM1) - a2 + A0*AM1 + AM2)*b1)
        s1_val = 1/den_rs*((-a2**2 + A0*(a2*AM1 - a1*AM2) + a2*AM2)*b0 + (a2*(a1 - A0 - AM1) + A0*AM2)*b1)
        t0_val = T0_num/den_t
        t1_val = T1_num/den_t

        M = np.array([r0_val, s0_val, s1_val, t0_val, t1_val], float).reshape(-1, 1)
        N = np.array([-u_val[0], -y_measure[1], -y_measure[0], uc_val[1], uc_val[0]],float).reshape(M.shape)
        u_val = np.concatenate((u_val, 
                                (N.T@M).reshape(-1,)))
                                
        ## HERE IS WHERE WE NEED THE LOOKUP TABLE AND THE PWM CALL
        pwm_val = VL.vel2duty(u_val[1])
        # pwm_val += u_val[1] # For testing without lookup table
        changing_pwm(pwm_val)

            # Measurements and control parameters k

        time_arr = []
        time_ns = time.time_ns()
        for k in range(2, len(t)):
            phi = np.array([-y_measure[k-1], -y_measure[k-2], u_val[k-1], u_val[k-2]], float).reshape(-1,1)
            K = P@phi@inv(lam + phi.T@P@phi)
            theta_hat = theta_hat + K@(-y_measure[k-1] - phi.T@theta_hat)
            theta_arr = np.concatenate((theta_arr, 
                                        theta_hat.reshape(-1, 1)), axis=1)
            P = (np.eye(len(phi)) - K@phi.T)@P/lam

                # Measurements and control parameters k = 2
            # Measurement
            while not ser.inWaiting():
                pass
            height = bottom - np.array([float(ser.readline().decode('utf-8').rstrip())*1e-3])
            y_measure = np.concatenate((y_measure,
                                (height).reshape(-1,)))      
            
            # Control parameters
            a1, a2, b0, b1 = theta_hat[0], theta_hat[1], theta_hat[2], theta_hat[3]
            den_rs = ((-a1*b0*b1) + (a2*b0**2) + b1**2)   
            den_t = b0 + b1
            r0_val = 1/den_rs*((A0*AM2)*b0**2 + (-a1 + A0 + AM1)*b1**2 + (a2 - A0*AM1 - AM2)*b0*b1)
            s0_val = 1/den_rs*((-a1*a2 + a2*(A0 + AM1) - A0*AM2)*b0 + (a1**2 - a1*(A0 + AM1) - a2 + A0*AM1 + AM2)*b1)
            s1_val = 1/den_rs*((-a2**2 + A0*(a2*AM1 - a1*AM2) + a2*AM2)*b0 + (a2*(a1 - A0 - AM1) + A0*AM2)*b1)
            t0_val = T0_num/den_t
            t1_val = T1_num/den_t

            M = np.array([r0_val, s0_val, s1_val, t0_val, t1_val], float).reshape(-1, 1)
            N = np.array([-u_val[k-1], -y_measure[k], -y_measure[k-1], uc_val[k], uc_val[k-1]]).reshape(M.shape)
            u_val = np.concatenate((u_val, 
                                    (N.T@M).reshape(-1,))) 

            ## HERE IS WHERE WE NEED THE LOOKUP TABLE AND THE PWM CALL
            pwm_val = VL.vel2duty(u_val[k])
            # pwm_val += u_val[k] # For testing without lookup table
            changing_pwm(pwm_val)
            
            time_delta = (time.time_ns() - time_ns)*1e-9
            time_arr.append(time_delta)
            time_ns = time.time_ns()
            
            # print(f'For k={k},\tM={M},\tden_rs={den_rs}')
            print(theta_hat)

        time_np_arr = np.array(time_arr)
        print(f'mean={time_np_arr.mean()},\tvar={time_np_arr.var()}')

        # print(f'For k={k},\tM={M},\tden_rs={den_rs}')
        # print(theta_hat)

        input('Press ENTER to see graphs')
        for row in range(len(theta_arr)):
            plt.plot(t, theta_arr[row,:])
        plt.show()

        et = len(t)
        plt.plot(t[0:et], y_measure[0:et])
        plt.plot(t[0:et], uc_val[0:et])
        plt.show()
        plt.step(t[0:et], u_val[0:et])
        plt.show()

        input('Press ENTER to kill.')
        kill_pwm()
        
    except KeyboardInterrupt:
        kill_pwm()
