import serial
import time

with serial.Serial('/dev/ttyACM0', 19200, timeout=65) as ser:
    last_esp = None
    last_pc = None
    while True:
        line = ser.readline().decode().strip()
        print(line)
        if line == '':
            continue
        esp_time = int(line.replace('time: ', ''))
        pc_time = time.time()
        if last_esp != None:
            print('----------------')
            delta_esp = (esp_time - last_esp) / 1000
            delta_pc = pc_time - last_pc
            print(f'delta_esp: {delta_esp}')
            print(f'delta_pc:  {delta_pc}')
            print(f'{delta_esp / delta_pc}')
        last_esp = esp_time
        last_pc = pc_time
