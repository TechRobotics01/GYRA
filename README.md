# GYRA
GYRA is a 3 axis gimbal system onto which i can connect my point and shoot camera. This system will automatically stabelize pitch roll and yaw axes of the gimbal via a high frequency PID loop runnong on an esp32 which controls the 3 mg995 servo motors of the gimbal 

# details 
so basically this systemn consists of these main parts 
1. the 3d printed gimbal assembly
2. the custom made pcb based on esp32 
3. 3x mg995 metal gear servo motors
4. 4x Li-ion battries with their custom charging board based on IP5328P IC 

# Prototype 
so before making the actual project i will be working on making the prototype 
basically i will design an extremely basic gymbal mount for mg995 servo motors and print it in PLA with my 3D printer after that i'll add the motors on the gymbal mount and then i will connect it to an esp32 and a charget and then i will control the motors via the esp32 after that i will add the PID control system and try to tune the motors also the camera will fit on the mount with a 1/4inch thread 
after that i will rigirously tune the Kp Ki and Kd values untill the prototype mount is working as intended with minimum shakes after that i'll measure and note down the values such as the current the motors are consuming the minimum frequency of the loop and then i'll be keeping these values in my mind while designing the electronics and also i will amke the 3d printable design in many iterations and with lesser infill about so that i can save filament and also the entire mount assembly will be in 3 sub-parts first one is the base handel yaw axis and then is the upper roll axis and then the cmaera connector piece pitch axis and the imu sensor will be mounted inside the upper part of the base handel just below the camera so i can measure all the required gyroscopic values precisely 
# i will make thos prototype work and not focus on any aestehtics because i can always upgrade them in the later V2 # 
(UPDATE): The prototype V1 will be 2 axis only because it is easier to debug and develop the code unto  
(UPDATE): i will combine hmc5883l magnetometer with mpu6050 because the mpu6050 does not have any magnetic rotary encoder which means it will give unrelaible yaw values which will drift over time 
(UPDATE) so i ordered a MPU9250 because it has a magnetometer and is much more precisre and is 9Dof 

# Stages of prototype 
1. gimbal 3D design and iteration
2. printing the design
3. adding servo motors and other components
4. writing the code
5. rigrous tuning of PID controller
6. noting down errors and other values like current used in mAh which will be the base values i will keep in mind while designing the controller
7. remaking a better design of the gimabl i'll call it gimbal prototype V2
