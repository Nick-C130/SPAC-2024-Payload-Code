import serial
import time
import serial.tools.list_ports

timeout = 5
StartMessage = b'TF\n\r'

burnAccel = 200
burnTime = 2.4
gravAccel = -150
drougeSpeed = -26
mainSpeed = -10
mainAlt = 1000

vel = 0
alt = 0
lastTime = 0
deltaSet = False
expWait = True
startTime = 0
deltaPressure = 0
mode = 0

def split_int_to_bytes(data):
    if isinstance(data, int):
        num = data
    elif isinstance(data, bytes):
        num = int.from_bytes(data, byteorder='big')
    else:
        raise TypeError("Input must be either an integer or bytes")

    byte1 = (num >> 24) & 0xFF
    byte2 = (num >> 16) & 0xFF
    byte3 = (num >> 8) & 0xFF
    byte4 = num & 0xFF
    return (byte1, byte2, byte3, byte4)

def select_com_port():
    available_ports = list(serial.tools.list_ports.comports())
    
    if not available_ports:
        print("No COM ports available.")
        return None
    
    if len(available_ports) == 1:
        return available_ports[0].device
    
    print("Available COM ports:")
    for index, port in enumerate(available_ports, 1):
        print(f"{index}. {port.device}")
    
    while True:
        try:
            selection = int(input("Choose a port (1, 2, ...): "))
            if 1 <= selection <= len(available_ports):
                return available_ports[selection - 1].device
            else:
                print("Invalid selection. Please choose a valid port.")
        except ValueError:
            print("Invalid input. Please enter a number.")

def send_message(ser, message):
    for i in message:
        ser.write(split_int_to_bytes(i))
        #print(f"Sent message: {message.decode('utf-8')}")

def sendData(ser,message):
    for packed_value in message:
        ser.write(packed_value)

def read_serial_data(ser):
    data = []
    start_time = time.time()
    while True:
        try:
            while ser.in_waiting > 0:
                data.append(ser.read(1))
                if b'\n' in data[-1]:
                    return data        
            if time.time() - start_time > timeout:
                return None
        except serial.SerialException as e:
            print("Serial port error:", e)
            return None

def toStr(data):
    if(data == None):
        return None
    return b''.join(data).decode('utf-8')   

def altitude_to_pressure(altitude):
    if not isinstance(altitude, (int, float)):
        raise TypeError("Altitude must be an integer or float")
    P0 = 1013.25  # Atmospheric pressure at sea level in hPa
    L = 0.0065  # Temperature lapse rate in K/m
    T0 = 288.15  # Temperature at sea level in K
    g = 9.80665  # Acceleration due to gravity in m/s^2
    M = 0.02896  # Molar mass of Earth's air in kg/mol
    R = 8.31447  # Universal gas constant in J/(mol·K)

    P = P0 * (1 - (L * altitude) / T0) ** ((g * M) / (R * L))
    return float(P)

selected_port = select_com_port()

with serial.Serial(selected_port,115200, timeout=timeout) as ser:
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    send_message(ser, StartMessage)

    while True:
        message = read_serial_data(ser)
        print(toStr(message))
        if(message == None):
            continue
        if (message[0] == b'R'):
            if(message[1] == b'D'):
                if(expWait):
                    expWait = False
                    startTime = time.time()
                    lastTime = startTime
                if(not(expWait)):
                    pressChambReading = float(toStr(message[3:8]))
                    tempChambReading = float(toStr(message[8:13]))
                    altReading = float(12.3)
                    pressAtmosReading = float(pressChambReading)
                    tempAtmosReading = float(tempChambReading)
                    
                    if((time.time() - startTime) < burnTime and (mode == 0 or mode == 1)):#burnAccel
                        vel = vel + burnAccel * (time.time()-lastTime)
                        mode = 1
                    elif(vel > 0 and (mode == 1 or mode == 2)):#coast
                        vel = vel + gravAccel * (time.time()-lastTime)
                        mode = 2
                    elif(alt > mainAlt and (mode == 2 or mode == 3)):#Drouge decent
                        vel = drougeSpeed
                        mode = 3
                        if(not(deltaSet)):
                            deltaSet = True
                            deltaPressure = pressAtmosReading - pressAtmos
                    elif(alt < mainAlt and (mode == 3 or mode == 4)):#Main decent
                        vel = mainSpeed
                        mode = 4
                    elif(alt <= 0 and (mode == 4 or mode == 5)):#Landed
                        vel = 0
                        mode = 5
                        print("Done")
                        while True:
                            time.sleep(1)

                    alt = alt + vel * (time.time()-lastTime)
                    pressAtmos = altitude_to_pressure(alt)
                    pressChamb = pressChambReading - deltaPressure

                    lastTime = time.time()
                    print("Velocity: {} Altitude: {} Pressure: {} Chamber pressure: {} Fake chamber Pressure: {} Mode: {}".format(vel,alt,pressAtmos,pressChambReading,pressChamb,mode))
                    data = [int(alt*1000),int(pressAtmos*1000),int(pressChamb*1000),int(tempAtmosReading*1000),int(tempChambReading*1000)]
                    send_message(ser,data)

