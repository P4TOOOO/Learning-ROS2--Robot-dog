import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class RoboticDogNode(Node):
    def __init__(self):
        super().__init__('robotic_dog_node')
        self.lado = 10.0  # tamanho do lado (a2)

        # Publisher para comandos de posição no Gazebo
        self.command_publisher = self.create_publisher(
            JointTrajectory,
            '/position_controllers/joint_trajectory',
            10
        )
        self.joint_names = ['hip1', 'thigh1', 'shin1', 'hip2', 'thigh2', 'shin2', 'hip3', 'thigh3', 'shin3', 'hip4', 'thigh4', 'shin4']

        self.start_pos = (10.0, 12.0, 5.0)
        self.end_pos = (10.0, 12.0, 5.0)
        self.path_points = [ (10.0, 7.0, 3.0), (10.0, 14.0, 7.0), (10.0, 14.0, -7.0), (10.0, 7.0, -3.0), (10.0, 5.0, 0.0)]
        self.path_times = (0.0, 0.5, 1.0, 1.5, 2.0)
        self.delay_steps = 90
        self.leg_delays = [0, self.delay_steps, 60, self.delay_steps+60]
        self.steps_per_segment = 30
        

        # Gera a trajetória
        self.trajectory = self.generate_trajectory_forward(self.path_points, self.path_times, self.steps_per_segment)
        self.trajectory_length = len(self.trajectory)

        self.positions = []
        self.all_angles = []
        self.current_step = 0

        # Timer para calcular e publicar a cada 0.1s
        self.timer = self.create_timer(0.1, self.movement_callback)

        self.get_logger().info('Node rodando: calculando IK em radianos e publicando comandos para o Gazebo!')

    def func_y(self, i):
        if 0 <= i <= 0.5:
            return -(80 * (i - 0.25) ** 2 + 5)
        else:
            return 10

    # === ADICIONE ESSAS DUAS FUNÇÕES NOVAS (logo após o func_y, por exemplo) ===

    def calculate_ik_left(self, Pwx, Pwy, Pwz):
        """IK apenas para as pernas ESQUERDAS (1 e 2)"""
        a2 = self.lado
        r = math.sqrt((Pwx**2) + (Pwy**2))
        D_sq = (Pwx**2) + (Pwy**2) + (Pwz**2)
        D = math.sqrt(D_sq)

        if D > 2 * a2 or D == 0:
            self.get_logger().warn(f"Posição inalcançável (left): ({Pwx}, {Pwy}, {Pwz})")
            return None

        v1 = 0.0                                      # mantendo fixo como você queria
        cosv3 = (D_sq - 2*a2**2) / (2 * a2**2)
        cosv3 = max(-1.0, min(1.0, cosv3))
        senv3 = math.sqrt(1 - cosv3**2)
        v3 = math.atan2(senv3, cosv3)

        senv2 = ((a2 + a2*cosv3)*Pwz - a2*senv3*r) / D
        cosv2 = ((a2 + a2*cosv3)*r + a2*senv3*Pwz) / D
        v2 = math.atan2(senv2, cosv2)

        return v1, v2, v3

    def calculate_ik_right(self, Pwx, Pwy, Pwz):
        """IK apenas para as pernas DIREITAS (3 e 4) - com sinais corrigidos"""
        a2 = self.lado
        r = math.sqrt((Pwx**2) + (Pwy**2))
        D_sq = (Pwx**2) + (Pwy**2) + (Pwz**2)
        D = math.sqrt(D_sq)

        if D > 2 * a2 or D == 0:
            self.get_logger().warn(f"Posição inalcançável (right): ({Pwx}, {Pwy}, {Pwz})")
            return None

        v1r = 0.0                                     # mantendo fixo (igual ao esquerdo)
        cosv3 = (D_sq - 2*a2**2) / (2 * a2**2)
        cosv3 = max(-1.0, min(1.0, cosv3))
        senv3 = math.sqrt(1 - cosv3**2)
        v3 = math.atan2(senv3, cosv3)

        # === AQUI ESTÁ A CORREÇÃO PRINCIPAL ===
        # Usamos o MESMO v2 do lado esquerdo (não a fórmula antiga v2r)
        senv2 = ((a2 + a2*cosv3)*(-Pwz) + a2*senv3*r) / D
        cosv2 = ((a2 + a2*cosv3)*r - a2*senv3*(-Pwz)) / D
        v2r = math.atan2(senv2, cosv2)                # ← mesmo v2 das pernas esquerdas

        v3r = -v3                                     # canela invertida (padrão)

        return v1r, v2r, v3r
    def calculate_ik(self, Pwx, Pwy, Pwz):
        # Calcula separadamente para cada lado (exatamente como você pediu)
        left  = self.calculate_ik_left(Pwx, Pwy, Pwz)
        right = self.calculate_ik_right(Pwx, Pwy, Pwz)

        if left is None or right is None:
            return None
        
        perna1 = list(left)   # [v1, v2, v3]
        perna2 = list(left)   # [v1, v2, v3]
        perna3 = list(right)  # [v1r, v2r, v3r]
        perna4 = list(right)  # [v1r, v2r, v3r]

        return perna1, perna2, perna3, perna4

    def generate_trajectory_forward(self, path_points, path_times, steps_per_segment):
        trajectory = []
    
        for seg in range(len(path_points) - 1):                    # 2 segmentos para 3 pontos
        # Pontos e tempos do segmento atual
            q0 = path_points[seg]
            qf = path_points[seg+1]
            t0 = path_times[seg]
            tf = path_times[seg+1]
            T  = tf - t0
        
        # Velocidades iniciais e finais do segmento (0 no começo e fim do movimento)
            v0 = (0.0, 0.0, 0.0) if seg == 0 else None   # será calculado no próximo passo
            vf = (0.0, 0.0, 0.0) if seg == 1 else None
        
        # (Aqui você pode calcular v1 automaticamente se quiser, mas para simplicidade usamos 0)
        
        # Coeficientes (exatamente como no livro)
            a0 = q0
            a1 = (0.0, 0.0, 0.0)                     # v0 = 0
            a2 = (
                3*(qf[0]-q0[0])/T**2 - 2*a1[0]/T,
                3*(qf[1]-q0[1])/T**2 - 2*a1[1]/T,
                3*(qf[2]-q0[2])/T**2 - 2*a1[2]/T
            )
            a3 = (
                -2*(qf[0]-q0[0])/T**3 + a1[0]/T**2,
                -2*(qf[1]-q0[1])/T**3 + a1[1]/T**2,
                -2*(qf[2]-q0[2])/T**3 + a1[2]/T**2
            )
        
        # Gera os pontos do segmento
            for i in range(steps_per_segment + 1):
                t = t0 + i * (T / steps_per_segment)
                tau = t - t0
                pos = (
                    a0[0] + a1[0]*tau + a2[0]*tau**2 + a3[0]*tau**3,
                    a0[1] + a1[1]*tau + a2[1]*tau**2 + a3[1]*tau**3,
                    a0[2] + a1[2]*tau + a2[2]*tau**2 + a3[2]*tau**3
                )
                trajectory.append(pos)
    
        return trajectory

    def movement_callback(self):
        if not self.trajectory:
            return

        joint_positions = [0.0] * 12

        # Calcula posição de cada perna com delay + loop infinito
        for leg in range(4):
            delay = self.leg_delays[leg]
            effective_step = self.current_step - delay

            if effective_step < 0:
                pos = self.trajectory[0]                    # ainda não começou
            else:
                # LOOP INFINITO: volta pro início quando chega no fim
                idx = effective_step % self.trajectory_length
                pos = self.trajectory[idx]

            # IK por lado (esquerda ou direita)
            if leg < 2:   # pernas 1 e 2 = esquerda
                ik_result = self.calculate_ik_left(*pos)
            else:         # pernas 3 e 4 = direita
                ik_result = self.calculate_ik_right(*pos)

            if ik_result:
                start_idx = leg * 3
                joint_positions[start_idx:start_idx + 3] = list(ik_result)

        # === PUBLICAÇÃO NO GAZEBO ===
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.velocities = [0.0] * 12
        point.time_from_start = Duration(sec=0, nanosec=100_000_000)
        msg.points = [point]
        self.command_publisher.publish(msg)

        # Log (para você acompanhar)
        ref_idx = self.current_step % self.trajectory_length
        ref_pos = self.trajectory[ref_idx]
        self.get_logger().info(
            f"Loop {self.current_step:4d} | Ref ({ref_pos[0]:.2f},{ref_pos[1]:.2f},{ref_pos[2]:.2f}) | "
            f"P1+P3: {tuple(round(x,4) for x in joint_positions[0:3])} | "
            f"P2+P4: {tuple(round(x,4) for x in joint_positions[3:6])}"
        )

        # (Opcional) Coleta só da perna 1 para possível plot futuro
        leg1_pos = self.trajectory[ref_idx]
        self.positions.append(leg1_pos)
        self.all_angles.append((
            list(joint_positions[0:3]),
            list(joint_positions[0:3]),
            list(joint_positions[0:3]),
            list(joint_positions[0:3])
        ))

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
        for i in range(3):  
            plt.plot(steps, [a[i] for a in self.all_angles], label=f'{self.joint_names[i]}')
        plt.xlabel('Passo')
        plt.ylabel('Ângulo (radianos)') # Atualizado para radianos
        plt.title('Ângulos da Perna 1 ao longo da trajetória')
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
