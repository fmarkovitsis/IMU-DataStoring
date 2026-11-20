import re

osfile = open(f"outputs\\opensimData.sto", "w")

osfile.write("Datarate=50.000000\nDataType=Quaternion\nversion=3\nOpenSimVersion=4.1\nendheader\ntime	pelvis_imu	femur_r_imu	tibia_r_imu	calcn_r_imu\n")

imuFilesS = []
imuFilesD = []

# imuFilesS is a list of files containing data recorded from a single IMU per MCU
# imuFilesD is a list of files containing data recorded from two IMUs per MCU
# sample files provided in the "data" folder 
# apend files using the given format

########################## DATA PATHS ##########################
# for one imu per mcu
imufile0 = open ("outputs\\file_0.txt", "r")
imuFilesS.append(imufile0.readlines())
imufile1 = open ("outputs\\file_1.txt", "r")
imuFilesS.append(imufile1.readlines())


# for two imus per mcu
imufile2 = open ("outputs\\file_10.txt", "r")
imuFilesD.append(imufile2.readlines())
########################## DATA PATHS ##########################


doublecounter = 0
flag1 = 0
flag2 = 0

for i in range (0, len(imuFilesS[0])):
    for file in imuFilesS:
        line = file[i]

        numbers = re.findall(r'-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?', line) 

        if len(numbers) == 5:
            flag2 = 1

            if flag1 == 0:
                flag1 = 1

                time = int(numbers[0]) / 1000.0
                time = str(time)

                osfile.write(f"{time}	")

            osfile.write(f"{numbers[1]},{numbers[2]},{numbers[3]},{numbers[4]}	")
    flag1 = 0

    for file in imuFilesD:
        for j in range (0, 2):
            if len(file)-1 < doublecounter:
                break

            line = file[doublecounter]


            numbers = re.findall(r'-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?', line) 
            doublecounter = doublecounter + 1

            if len(numbers) == 5:
                osfile.write(f"{numbers[1]},{numbers[2]},{numbers[3]},{numbers[4]}	")
            else: 
                break

    if flag2 == 1:
        osfile.write("\n")
        flag2 = 0


osfile.close()