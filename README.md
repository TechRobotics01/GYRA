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

# FEATURES
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

# HARDWARE
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

# ZINE PAGE 

<img width="871" height="1246" alt="GYRA_ZINE_page-0001" src="https://github.com/user-attachments/assets/ce9c08a6-fd64-4687-b739-1c3d9a38d381" />

# HOW TO BUILD IT FOR YOURSELF 
So you want to make this project for yourself too <br>
I have included each and every file and component that you will require in this repository and BOM 
STEP-1
- Buy all the parts in the BOM
- Print the parts inside the Mark-1--->>STEP FILE folder (total of 6 parts)
- Get the PCB fabricated and solder all the components neatly using the HOTPLATE
- Test the PCB for any imporperly soldered parts/components
- Assemble the pcb inside the 3D printed hexalegged base using M2 screws
- Now connect the battries in a 2S3P manner
- Cut and strip the wires and make it into a JST connector
  - 1 FOR MPU6050
  - 1 FOR HMC5883L
  - 1 FOR SERVOS AND GND PIN
  - 1 FOR THE BATTERY connector
  - Solder the battery connections as stated under the "PCB / electronics part" and then house the battires inside the base handle
  - Join the hexalegged base to the pcb lid using M3 and M2 screws whereever needed
  - now assemble the above appembly consisting of
    - Servos
    - IMUs
    - YAW arm
    - YAW---PITCH arm
    - PITCH----ROLL arm
    - And then finally connect the 1/4th inch thread to the print
    - Connect your camera or phone adapter
    - Finally please use the code to TUNE your gimbal different tuning values are needed for different cameras because center of mass and orientation
    - Affect the PID constants a lot
    - After tuning your gimbal please hardcode the values into the gimbal (more info on that in a few days)
    - And finally enjoy a gimbal you made!!!!  
    WARNING: you may need to heat the 1/4th inch screw to 100*C so it can be properly housed inside the PITCH arm

# PROBLEMS FACED
there were a lot of problems i faced during the development of this project i have all the prototypes saved in subfolders in the CAD filder this project had a total of 5 iterated prototypes before i made the actual MARK-1 <br>
so here is a list of probelms i faced <br>

So first of all i had no inspiration and was designing the structure i had in my head when i built and tested the first printed prototype i found out many many fatal flaws such as the biggest one being that the roll arm was colliding with the base of the YAW arm where the sensor was seated and also that the motor constrians were misjudged second of all the arms were flexing because i made then longer than they needed to be and the arm used to jitter and shake because the motor could not take the load of the heavy arms and assembly <br>

after like 3 iterations i finally took a bit of inspiration from ronin's gimbal and finally understood how the gimabl's standard structure should actually be first of all i was veru skeptical about if the motors could move the long arms or not as the length of the arms were almost excedding  the torue the motors could produced but once it was assembeled nd i tested it....IT worked just fine <br>

Now here comes the biggest challenge i faced almost throughtout the project so first of alli was using MG995 motors's plastic head and attachments to connect the 3D printed arm with the motor which was an bad idea because it already had a bit of play which was only made worse by the motor repeatedly moving and wearking out the joint's teeth during Prottype's PID tuning and i found out the solution to this when i was sitting in my room at 12 AM and the idea of using CNC Aluminium 25T servo spline arm <br>
<img width="216" height="162" alt="image" src="https://github.com/user-attachments/assets/4ebf853e-cf60-48cf-8eb0-661cf463f28c" />
 This thing and now i designed the nre joint in accordance to the dimension's of the spline and i used properl M3 and M2 screws to mount the print on this isntead of using improper sized M2.2 screws everywhere and hoping it would work 

 Overall this project was one of the best learning project i had made because it taught me many things related to 3D printing CAD and print tolerances
 and also sharpened my skills of PCB and Embedded design design and overall project design and workflow 
 The multiple failed prototypes challenged the appumptions i made and i developed a new kind of more efficient workflow than before 
 
### I SPENT A TOTAL OF ~55 HOURS ON THIS PROJECT ###

# CREDITS 
- ME
- google.com
- chatgpt (FOR DEBUGGING AND GUIDANCE ONLY)
- [randomnerdtutorial.com](https://randomnerdtutorials.com/)
