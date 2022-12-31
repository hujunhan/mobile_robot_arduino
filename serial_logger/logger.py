import serial
import numpy as np
ser=serial.Serial('/dev/tty.usbmodem11301',115200)
rpm_list=[]
pwm_list=[]
while True:
    try:
        ser_bytes = ser.readline()
        try:
            data_str=str(ser_bytes)
            print(data_str)
            data=data_str.split(',')
            rpm=float(data[0].split('\'')[-1])
            rpm_list.append(rpm)
            pwm=int(data[1].split('\\')[0])
            pwm_list.append(pwm)
            # print(f'rpm:{rpm}, pwm:{pwm}')
            if(pwm==254):
                np.savez('./serial_logger/linear.npz',pwm=pwm_list,rpm=rpm_list)
                break
        except:
            continue
    except:
        break