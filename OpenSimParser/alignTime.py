import re
import sys
import numpy as np

def getTime(timeString):
    time = []

    time.append(timeString[0:2])
    time.append(timeString[2:4])
    time.append(timeString[4:6])
    time.append(timeString[6:])

    return time

imuFilesS = []
imuFilesD = []

# imuFilesS is a list of files containing data recorded from a single IMU per MCU
# imuFilesD is a list of files containing data recorded from two IMUs per MCU
# sample files provided in the "data" folder 
# apend files using the given format

########################## DATA PATHS ##########################
# for one imu per mcu
imufile0 = open (r"C:\Users\sol77\Desktop\Motion-Capture\OpenSimParser\dataExp\COM5_data.txt", "r")
imuFilesS.append(imufile0.readlines())
imufile1 = open (r"C:\Users\sol77\Desktop\Motion-Capture\OpenSimParser\dataExp\COM18_data.txt", "r")
imuFilesS.append(imufile1.readlines())


# for two imus per mcu
imufile2 = open (r"C:\Users\sol77\Desktop\Motion-Capture\OpenSimParser\dataExp\COM9_data.txt", "r")
imuFilesD.append(imufile2.readlines())
########################## DATA PATHS ##########################


# align time
# for now, it is considered that the imu that started last is the last "double" file attached
numbers = re.findall(r'-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?', imuFilesD[len(imuFilesD)-1][0]) 
startTime = getTime(numbers[1])

timeOffsetsS = []
for file in imuFilesS:
    numbers = re.findall(r'-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?', file[0]) 
    time = getTime(numbers[1])

    counter = 0
    while 1:
        if ((int(time[0]) == int(startTime[0])) & (int(time[1]) == int(startTime[1])) & (int(time[2]) == int(startTime[2])) & (int(time[3]) == int(startTime[3]))):
            break

        counter = counter + 1

        time[3] = str(int(time[3]) + 20)

        if time[3] == "1000":
            time[3] = "000"
            time[2] = str(int(time[2]) + 1) 

        if time[2] == "60":
            time[2] = "00"
            time[1] = str(int(time[1]) + 1) 


        if time[1] == "60":
            time[1] = "00"
            time[0] = str(int(time[0]) + 1) 

        if time[0] == "24":
            time[0] = "00"



    timeOffsetsS.append(counter)

timeOffsetsD = []
for file in imuFilesD:
    numbers = re.findall(r'-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?', file[0]) 
    time = getTime(numbers[1])

    counter = 0
    while 1:
        if ((int(time[0]) == int(startTime[0])) & (int(time[1]) == int(startTime[1])) & (int(time[2]) == int(startTime[2])) & (int(time[3]) == int(startTime[3]))):
            break

        counter = counter + 1

        time[3] = str(int(time[3]) + 20)

        if time[3] == "1000":
            time[3] = "000"
            time[2] = str(int(time[2]) + 1) 

        if time[2] == "60":
            time[2] = "00"
            time[1] = str(int(time[1]) + 1) 


        if time[1] == "60":
            time[1] = "00"
            time[0] = str(int(time[0]) + 1) 

        if time[0] == "24":
            time[0] = "00"

    timeOffsetsD.append(counter)

for i in range (0, len(imuFilesS)):
    realOffset = timeOffsetsS[i] * 20

    for j in range (0, len(imuFilesS[i])):
        numbers = re.findall(r'-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?', imuFilesS[i][j])

        if (len(numbers) == 0):
            continue
        
        if int(numbers[0]) == realOffset:
            break


    del imuFilesS[i][2:(j-1)]

    for j in range (2, len(imuFilesS[i])):
        numbers = re.findall(r'-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?', imuFilesS[i][j])
        if len(numbers) < 5:
            continue

        num = int(numbers[0]) - realOffset
        imuFilesS[i][j] = f"{str(num)} {numbers[1]} {numbers[2]} {numbers[3]} {numbers[4]}\n"

for i in range (0, len(imuFilesD)):
    realOffset = timeOffsetsD[i] * 20

    for j in range (0, len(imuFilesD[i])):
        numbers = re.findall(r'-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?', imuFilesD[i][j])

        if (len(numbers) == 0):
            continue
        
        if int(numbers[0]) == realOffset:
            break


    del imuFilesD[i][2:(j-1)]

    for j in range (2, len(imuFilesD[i])):
        numbers = re.findall(r'-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?', imuFilesD[i][j])
        if len(numbers) < 5:
            continue

        num = int(numbers[0]) - realOffset
        imuFilesD[i][j] = f"{str(num)} {numbers[1]} {numbers[2]} {numbers[3]} {numbers[4]}\n"



ts = 0

terminate = 0

singleIdx = []
imuFilesSnu = []
for i in range (0, len(imuFilesS)):
    singleIdx.append(3)
    imuFilesSnu.append([])

doubleIdx = []
imuFilesDnu = []
for i in range (0, len(imuFilesD)):
    doubleIdx.append(2)
    imuFilesDnu.append([])

while 1:
    counter = 0

    for i in range (0, len(imuFilesS)):
        if singleIdx[i] >= len(imuFilesS[i]):
            terminate = 1
            break

        numbers = re.findall(r'-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?', imuFilesS[i][singleIdx[i]])

        if int(numbers[0]) == ts:
            counter = counter + 1
            singleIdx[i] = singleIdx[i] + 2
    
    for i in range (0, len(imuFilesD)):
        if doubleIdx[i] >= len(imuFilesD[i]):
            terminate = 1
            break

        numbers = re.findall(r'-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?', imuFilesD[i][doubleIdx[i]])

        if int(numbers[0]) == ts:
            counter = counter + 1
            doubleIdx[i] = doubleIdx[i] + 3
    
    if terminate == 1:
        break
    
    if counter == len(imuFilesS) + len(imuFilesD):
        for i in range (0, len(imuFilesS)):
            numbers = re.findall(r'-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?', imuFilesS[i][singleIdx[i]-2])
            imuFilesSnu[i].append(f"{numbers[0]} {numbers[1]} {numbers[2]} {numbers[3]} {numbers[4]}\n\n")

        
        for i in range (0, len(imuFilesD)):
            numbers = re.findall(r'-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?', imuFilesD[i][doubleIdx[i]-3])
            imuFilesDnu[i].append(f"{numbers[0]} {numbers[1]} {numbers[2]} {numbers[3]} {numbers[4]}\n")
            numbers = re.findall(r'-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?', imuFilesD[i][doubleIdx[i]-2])
            imuFilesDnu[i].append(f"{numbers[0]} {numbers[1]} {numbers[2]} {numbers[3]} {numbers[4]}\n\n")

    ts = ts + 20

for i in range (0, len(imuFilesSnu)):
    with open(f"outputs\\file_{i}.txt", "w") as f:
        f.writelines(imuFilesSnu[i])


for i in range (10, len(imuFilesDnu)+10):
    with open(f"outputs\\file_{i}.txt", "w") as f:
        f.writelines(imuFilesDnu[i-10])