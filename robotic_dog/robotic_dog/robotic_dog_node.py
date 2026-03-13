import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt

class RoboticDogNode(Node):
    def __init__(self):
        super().__init__('robotic_dog_node')
        self.lado = 10.0  # tamanho do lado

        # Publisher para comandos de posição no Gazebo
        self.command_publisher = self.create_publisher(
            Float64MultiArray,
            '/position_controllers/commands',
            10
        )

        self.start_pos = (-5.0, 0.0, 0.0)
        self.end_pos = (0.0, 0.0, 0.0)
        self.num_steps = 50

        # Gera a trajetória
        self.trajectory = self.generate_trajectory_forward(self.start_pos, self.end_pos, self.num_steps)

        self.positions = []
        self.all_angles = []
        self.current_step = 0

        # Timer para calcular e publicar a cada 0.1s
        self.timer = self.create_timer(0.1, self.movement_callback)

        # Para sincronizar com Gazebo (tempo simulado)
        self.get_logger().info('Node rodando: calculando IK e publicando comandos para o Gazebo!')

    def func_y(self, i):
        if 0 <= i <= 0.5:
            return -(80 * (i - 0.25) ** 2 + 5)
        else:
            return 10

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

    def generate_trajectory_forward(self, start_pos, end_pos, num_steps=50):
        x0, y0, z0 = start_pos
        trajectory = []
        for step in range(num_steps + 1):
            amplitude = 5.0
            t = step / num_steps
            x = -(amplitude * math.sin((2 * math.pi * t) - 1.5))
            y = self.func_y(t) 
            z = 0
            trajectory.append((x, y, z))
        return trajectory

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

def main(args=None):
    rclpy.init(args=args)
    node = RoboticDogNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
