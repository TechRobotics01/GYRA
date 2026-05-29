# GYRA 
So "GYRA" is a 3-axis stabilization gimbal system i built for phones and lighteight cameras more specifically point and shoot cameras<br>
GYRA is fully custom built from the 3D Design to the motherboard PCB all made by ME <br>
The name "GYRA" i derived from gyroscope which is a part of the MPU6050 sensor used in this project and GYRA also sounds pretty cool too <br>

# PURPOSE
So why exatly does this project exist <br>
This project exists because of a few reasons
- cheaper solution to ronin gymbals 
- extremely good learning project 
- fully custom built so i can easily add my own features

# FEATURED
- based on ESP32 S3 WROOM-1
- Fully custom 3D designed parts
- MPU6050 and HMC5883L used for motion tracking
- Portable design
- Custom PCB
- Designed for phones / lightweight cameras (upto ~ 200g)

# HOW DOES IT WORK
The mpu6050 and HMC5883L countinuously track the relative motion of the gimbal and then send the signals to the motherboard the ESP32 based motherboard then processes the signals to understand the orientation and run 3 seperate PID loops for 3 seperate axes namely 
- YAW
- ROLL
- PITCH
After the signals are passed through an Alpha filter for clearning the NOISE then are send to their respective PID loop the values i found which work for this system are
- For roll and Pitch axis
- Kp->1.2
- Ki->0
- Kd->0.1

I havent't really found the perfect values for the yaw axis as the yaw axis is the most unstable axis of the 3 axes and needs propery callibration  even more precise PID tuning for proper axis stability 

These 3 PID loops are the heart of the program these loops need to be very precise for proper theoretical working of the system <br>
I have a very good reason why i am using the word Theoretical
because in reality the project suffered from bracket instability joint wobble and print arm fles during the prototype phase which i worked hard to eliminate in MARK-1

# HARDWRE
## 3D PRINTABLE ASSEMBLY 
here is a render of the 3D printable design 
<img width="1920" height="1080" alt="GYRA marl-1 with camera" src="https://github.com/user-attachments/assets/dce59ac7-e183-42e2-8ad5-0a4ff757f0f9" />
<img width="1920" height="1080" alt="GYRA EXPLODED PNG" src="https://github.com/user-attachments/assets/cb24fc0f-3608-404c-8c5e-85e0b66a1462" />
The entire assembly consists of a total of 6 files which are 
- HEXALEGGED BASE (PCB housing)
- PCB HOUSING LID
- BASE HANDEL
- YAW ARM
- YAW--ROLL ARM
- ROLL--PITCH ARM
All of these are connected by mostly standard M3 screws whereas M2 screws may be required at some places such as the last 2 screws at ever arm of the 25T Aluminium CNC servo spline

## PCB / elecronics 
The pcb consists of the following majour parts 
- ESP32 S3 WROOM-1 module
- JST Connectors for
  - MPU6050 (4-pinJST)
  - HMC5883L (4-pinJST)
  - SERVOS(incuding GND) (4-pinJST)
  - Battery connector (2Pin JST)
- Decoupling capacitors
- IO0 and EN switched (SMD)
- M3 mounting hole
<img width="577" height="712" alt="image" src="https://github.com/user-attachments/assets/cc860c7f-f0ba-4dd9-90ae-8b7d2230a207" />
<img width="593" height="629" alt="image" src="https://github.com/user-attachments/assets/658e4e0d-07cd-4d4e-a24a-787e9be7c67f" />
<img width="695" height="708" alt="image" src="https://github.com/user-attachments/assets/9f687f2d-f694-4912-9130-0a69bc26c912" />

Now here comes the important part for the connnections so no-1 you need to first of all connect the 6-li-ion battries enclosed in the base handle to a 2S BMS module and to a DC femlae jack for charging and then you need to diectly connect the VCCs and GNDs of the servos to the BMS module and now you need to make a 4 pin JST connector which is basically 3 pins for servos yaw-roll-pitch respectively and final pin GND.

Now after this you need to  take the battry 7.4V and then connect it directly to the PCB as the PCB has a internal Step-Down voltage circuit but you need to be careful with the polarity. 

Please refer to the PCB schematics while hotplate soldering / assembling your PCB because the BARE PCB does not have much marking as to such where which pin should be conected 


