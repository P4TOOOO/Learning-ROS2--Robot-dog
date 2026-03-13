# Robotic-dog-simulation
I'm currently learning ros2 and making a robot dog simulation  
I want to share all my docs, current steps and future updates I want to make to this simulation, I'm using ROS2 Jazzy running in Ubuntu 24.04.04 with gazebo harmonics, I have some files already done and a basic test model of my robot, so i will show and try to explain it for you.  
# Robot Model
This is my test model:  

  
![Alt text](Robot_model.png)  

  
It has 12 joints, 3 in each leg, hip, thigh and shin, (he is a little ugly but no problem) I build it with onshape cad and translated to URDF by onshape-to-robot commands.  
## Joint limits  
The limits of the joints, is essentially a hardware limit imposed by the motors that I will use (when I build it), with will be the MG995 servo motors because they are cheap and affordable for me, or they are limits of the structure of the robot design that not allow the motors to continue rotating. Anyway, in my URDF model I believe these limits are wrong or they are rotating in the opposite direction to what I would like. I'll check this and I'll probably have to redo the model correctly.
# Inverse kinematics  
# ROS2 Pkg structure  
# Gazebo Simulation   
# Lauch Files  
# ROS2 node-code  
# Path Motion  
# Walking  
# Controlling by the keyboard  


