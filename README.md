# Robotic-dog-simulation
I'm currently learning ros2 and making a robot dog simulation  
I want to share all my docs, current steps and future updates I want to make to this simulation, I'm using ROS2 Jazzy running in Ubuntu 24.04.04 with gazebo harmonics, In this repository is all of my package files including the IK code, the URDF model to the robot and the launch files for the gazebo simulation. I have some files already done and a basic test model of my robot, so i will show and try to explain it for you.  
# My package  
My package is used for two things at the same time:  
- Starting the gazebo  
- Rotating the node that makes it walk (He still can't walk)
  
He starts the gazebo with a launch file, and runs the node normaly like other packages, I will put the commands in later topics.    
# Installation  
First you have to make sure the ROS2 jazzy is installed in your system by running  
```bash
ls /opt/ros
```
If returns jazzy it is installed, if not you have to install it by going to the ROS2 jazzy documentation.  

  
After making sure of this you have to create a worksapce  
```bash
cd ~
mkdir -p program_ws/src
cd program_ws/src
```
  
Clone the repository  
```bash
git clone https://github.com/P4TOOOO/Learning-ROS2--Robot-dog.git
```

Install dependencies  
```bash
cd /program_ws
rosdep update #only once
rosdep install --from-paths src --ignore-src -r -y
```

Build the package  
```bash
colcon build
```

The package is installed in your workspace, now you need to source it to run the package, first source the ROS2 itself
```bash
cd ~
source /opt/ros/jazzy/setup.bash
```

Source the package  
```bash
source program_ws/install/setup.bash
```

Now you are ready to run the node for the gazebo sim of my robot.  
# Robot Model
This is my test model:  

  
![Alt text](Imagens/Robot_model.png)  

  
It has 12 joints, 3 in each leg, hip, thigh and shin, (he is a little ugly but no problem) I build it with onshape cad and translated to URDF by onshape-to-robot commands.  
## Joint limits  
The limits of the joints, is essentially a hardware limit imposed by the motors that I will use (when I build it), with will be the MG995 servo motors because they are cheap and affordable for me, or they are limits of the structure of the robot design that not allow the motors to continue rotating. Anyway, in my ![URDF](robotic_dog/urdf/robot.urdf) model I believe some of these limits are wrong or they are rotating in the opposite direction to what I would like. I'll check this and I'll probably have to redo the model correctly.
# Inverse kinematics  
In this section I'm going to talk about the IK to this robot, initially, I had created an inverse kinematics model that only considered the final position of the robot's "foot," but then I started reading a book called "Modeling and Control of Robot Manipulators" by Sciavicco ad Siciliano, and I realized that the way I had done it was probably wrong, but the book takes that into account the position and the orientation of the end-factor, "foot" of the robot, but in my project since the orientation does not change I only have to take into account the final position, I don't know how I'm going to continue doing this part. Anyway this is my IK model and formulas only taking into accout the final position:  
## Side view:  
  
![imagem](Imagens/)  
  

## Front view:  

![Imagem](Imagens/)  
  
# Lauch Files  
In this section I'm going to explain the code of my ![gazebo simulation](robotic_dog/launch/gazebo.launch.py) launch file and show how to run it  
```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
```
This part is the imports of some libraries that I use for making the launch file
```python
def generate_launch_description():
    pkg_dir = get_package_share_directory('robotic_dog')
    urdf_path = os.path.join(pkg_dir, 'urdf', 'robot.urdf')
```
Here I'm creating the function of the launch file and especifing the paths for the URDF and the package name  
```python
return LaunchDescription([
```
This line return all the actions make by the launch  
```python
IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    ),
    launch_arguments={'gz_args': '-r empty.sdf'}.items()
),
```
This opens the gazebo using the ros_gz_sim package  
```python
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_path, 'r').read()},{'use_sim_time': True}],
            output='screen',
```
Here we publish the URDF model to the gazebo and after it spawns the robot  
```python
 Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', '/robot_description',
                '-name', 'robotic_dog',
                '-allow_renaming', 'false',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '1.5',
                '-Y', '0.0'
            ],
            output='screen'
        ),
```
And finaly spawn the controlls for the robot using controller_manager package
```python
        TimerAction(
            period=8.0,  # 8 segundos de delay
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                    output='screen',
                    parameters=[{'use_sim_time': True}]
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['position_controllers', '--controller-manager', '/controller_manager'],
                    output='screen',
                    parameters=[{'use_sim_time': True}]
                ),
            ]
        ),
```
## Running the simulation  
First make sure to build and source the pkg if everything works out run the following command to initiate the gazebo simulation  
```bash
ros2 launch robotic_dog gazebo.launch.py
```
The gazebo will apear like this:    

![Alt text](Imagens/gazebo_sim.png)  

The joints state publisher doesn't work properly but i will try to fix it, if you want to try the model to see if it moves you can publish the value of the motors by running  
```bash
ros2 topic pub /position_controllers/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]" --once
```
With each of these values being the value of a specific motor of the robot where the order of the values correspond respectively to  
* [hip1, thig4, shin1, hip2, thig2, shin3, hip3, thig1, shin4, hip4, thig3, shin2]  
# Principal node  
I will explain my principal code but note that it does not work properly I'm triyng to fix it and make a walking motion but it will take a time to finish  
The imports look like this  
```python
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
```
The initialization looks like this it only initiate the node  
```python
class RoboticDogNode(Node):
    def __init__(self):
        super().__init__('robotic_dog_node')
```
Make a publisher class to publish the value to the simulation motors and the "lado" variable set the link lenth both are the same  
```python
        self.lado = 10.0  # tamanho do lado

        # Publisher para comandos de posição no Gazebo
        self.command_publisher = self.create_publisher(
            Float64MultiArray,
            '/position_controllers/commands',
            10
        )
```
Set the num_steps and call the trajectory to generate a list of positions  
```python
        self.num_steps = 50

        # Gera a trajetória
        self.trajectory = self.generate_trajectory_forward(self.num_steps)

        self.positions = []
        self.all_angles = []
        self.current_step = 0
```
Make the timers and sincronize with the gazebo timers  
```python
self.timer = self.create_timer(0.1, self.movement_callback)

        # Para sincronizar com Gazebo (tempo simulado)
        self.get_logger().info('Node rodando: calculando IK e publicando comandos para o Gazebo!')
```
Generate the y function for the motion  
```python
def func_y(self, i):
        if 0 <= i <= 0.5:
            return -(80 * (i - 0.25) ** 2 + 5)
        else:
            return 10
```
Make the IK calculus and send they to the correct motors (now it only send to 3 motors becouse I'm trying to make it simple to test)
```python
 def calculate_ik(self, x_input, y_input, z_input):
        yn = math.sqrt(y_input ** 2 + x_input ** 2)
        dist = math.sqrt(yn ** 2 + z_input ** 2)
        if dist > 2 * self.lado or dist < 0:
            self.get_logger().warn(
                f"Posição inalcançável: ({x_input}, {y_input}, {z_input}), dist={dist}"
            )
            return None

        knee = math.degrees(
            math.acos((2 * self.lado ** 2 - dist ** 2) / (2 * self.lado ** 2))
        )
        hip1 = math.degrees(math.acos(dist / (2 * self.lado)))
        hip1_correction = math.degrees(math.asin(x_input / dist)) if dist != 0 else 0
        hip2 = math.degrees(math.asin(z_input / dist)) if dist != 0 else 0

        motor_angle1 = -hip2
        motor_angle2 = 0
        motor_angle3 = 180 - knee 
        motor_angle4 = 0
        motor_angle5 = 0
        motor_angle6 = 0
        motor_angle7 = 0
        motor_angle8 = -hip1 + hip1_correction 
        motor_angle9 = 0
        motor_angle10 = 0
        motor_angle11 = 0
        motor_angle12 = 0

        return (
            motor_angle1, motor_angle2, motor_angle3,
            motor_angle4, motor_angle5, motor_angle6,
            motor_angle7, motor_angle8, motor_angle9,
            motor_angle10, motor_angle11, motor_angle12,
        )
```
Make the trajectory list based on the x, y and z functions (it's in that place that I think I need to make changes)  
```python
 def generate_trajectory_forward(num_steps=50):
        trajectory = []
        for step in range(num_steps + 1):
            amplitude = 5.0
            t = step / num_steps
            x = -(amplitude * math.sin((2 * math.pi * t) - 1.5))
            y = self.func_y(t) 
            z = 0
            trajectory.append((x, y, z))
        return trajectory
```
Callback function to run the node and publish the values  
```python
 def movement_callback(self):
        if self.current_step >= len(self.trajectory):
            self.get_logger().info('Cálculo concluído.')
            self.timer.cancel()
            self.plot_results()
            return

        pos = self.trajectory[self.current_step]
        angles_deg = self.calculate_ik(*pos)
        if angles_deg is not None:
            self.positions.append(pos)
            self.all_angles.append(angles_deg)

            # Converte graus para radianos (já 12 valores)
            angles_rad = [math.degrees(angle) for angle in angles_deg]

            # Publica os 12 valores diretamente (sem duplicar)
            msg = Float64MultiArray()
            msg.data = angles_rad
            self.command_publisher.publish(msg)

            self.get_logger().info(
                f"Passo {self.current_step}: Posição ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}) | "
                f"Ângulos (graus): {tuple(round(a, 2) for a in angles_deg)} | Publicados: {msg.data}"
            )

        self.current_step += 1
```
Shows the graph of the function and the graph of the motors angles   
```python
    def plot_results(self):
        if not self.positions:
            return
        xs, ys, zs = zip(*self.positions)
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(xs, ys, zs, marker='o')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('Trajetória Simulada')
        plt.show()

        steps = range(len(self.all_angles))
        plt.figure()
        for i in range(12):  # Agora para 12 motors
            plt.plot(steps, [a[i] for a in self.all_angles], label=f'Motor {i+1}')
        plt.xlabel('Passo')
        plt.ylabel('Ângulo (graus)')
        plt.title('Ângulos ao longo da trajetória')
        plt.legend()
        plt.show()
```
Start ROS2, create the node, and run it  
```python
def main(args=None):
    rclpy.init(args=args)
    node = RoboticDogNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
# Path Motion  
# Walking  
# Controlling by the keyboard  


