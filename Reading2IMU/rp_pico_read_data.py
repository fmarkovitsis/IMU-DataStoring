import serial
import time
import struct
import threading
import sys
from serial.tools import list_ports
from datetime import datetime
import os
import time

# detect devices connected to serial
ports = list(list_ports.comports())
print ("Detected ports:", [port.device for port in ports])

terminate = 0
recal = ""

# thread function
def readData (name):
    dataStore = open(f"{os.path.basename(name)}.txt", "w")
    
    ser = serial.Serial(name, 460800, timeout=30) 

    ser.write((f"ser est\n").encode("utf-8"))

    numGyros = ""
    deadline = time.time() + 222.0  # wait a few seconds for a reply
    while time.time() < deadline:
        numGyros = ser.readline().decode(errors="ignore").rstrip()  # waits up to timeout
        if numGyros:
            print(f"{name}: {numGyros} IMUs")

            break

    if not numGyros:
        print ("Port ", name, " timed out")
        return

    if recal == "y":
        print(f"{name}: Waiting for callibration:")
        ser.write((f"y\n").encode("utf-8"))

        for i in range (0, int(numGyros)):
            line = ""
            deadline = time.time() + 222.0  # wait a few seconds for a reply
            while time.time() < deadline:
                line = ser.readline().decode(errors="ignore").rstrip()  # waits up to timeout
                if line:
                    break

            if not line:
                print (f"Port {name} timed out")
            
            biasesDoc = open(f"biases{name}"+f"_{i}.json", "w")
            biasesDoc.write(line)
            biasesDoc.close()

    else:
        print(f"{name}: Reading biases from JSON")
        ser.write((f"n\n").encode("utf-8"))

        for i in range (0, int(numGyros)):
            biasesDoc = open(f"biases{name}"+f"_{i}.json", "r")
            jsonData = biasesDoc.read()
            jsonData += "\n"
            biasesDoc.close()

            ser.write(jsonData.encode("utf-8"))

            #debugging
            line = ""
            deadline = time.time() + 222.0  # wait a few seconds for a reply
            while time.time() < deadline:
                line = ser.readline().decode(errors="ignore").rstrip()  # waits up to timeout
                if line:
                    # print (line)
                    break
            
            if not line:
                print ("Port ", name, " timed out")
                return
        
    line = ""
    deadline = time.time() + 222.0  # wait a few seconds for a reply
    while time.time() < deadline:
        line = ser.readline().decode(errors="ignore").rstrip()  # waits up to timeout
        if line == "1":
            print(f"{name}: Done Callibrating")

            break

    if not line:
        print ("Port ", name, " timed out")
        return

    while (1):
        tstr = datetime.now().strftime("%H%M%S%f")[:-3]
        mil = tstr[-3:]

        if int(mil) % 5 == 0:
            ser.write(("start\n").encode("utf-8"))
            break
    
    dataStore.write(f"{name}: Started at {tstr}\n\n")
    print (f"{name}: Started at {tstr}\n")

    form = "<I4f" # < is for little endian, 1 int 4 floats 
    bytes_expected = struct.calcsize(form)

    while (1):
        if terminate == 1:
            break
        
        for i in range (0, int(numGyros)):
            bytes_read = ser.read(bytes_expected)
            if len(bytes_read) != bytes_expected:
                continue

            data = struct.unpack(form, bytes_read)  
            dataStore.write(f"{data} \n")
        
        dataStore.write("\n")

    dataStore.close()
    ser.close()

while (1):
    UserIn = input("Start? [y/n]: ")
    if UserIn == "n":
        sys.exit()
    elif UserIn == "y":
        break
    else:
        continue

recal = input("Want to Re-Calibrate? [y/n]: ")

# create worker threads, each reading from one communication port
threads = []
for port in ports:
    t = threading.Thread(target=readData, args=[port.device])
    t.start()
    threads.append(t)

print ("Press [q] to terminate.")

while (1):
    UserIn = input()

    if UserIn == "q":
        break

terminate = 1

for thread in threads:
    thread.join()   # wait for it to finish