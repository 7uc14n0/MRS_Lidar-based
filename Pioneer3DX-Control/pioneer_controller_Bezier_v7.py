import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
import math
import random
import time
import csv
import os

class PioneerFinalController(Node):
    def __init__(self):
        super().__init__('pioneer_final_controller')
        
        # --- Configurações ROS ---
        qos_cmd = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 20)
        
        qos_odom = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.subscription = self.create_subscription(Odometry, '/pose', self.pose_callback, 20)

        self.msgReceiver = self.create_subscription(String, '/string', self.start_callback, 10)
        self.volume_request_pub = self.create_publisher(String, '/volume_request', 10)  

        # --- Variáveis de Controle ---
        self.started = False
        self.start_seq = 0
        self.start_seq_processed = 0
        self.volume_requested = False

        # --- Pontos ---
        self.points = {
            'A': {'x': -1.15, 'y': -1.15},
            'B': {'x': -0.70, 'y': +1.55},
            'C': {'x': +2.05, 'y': +1.10},
            'D': {'x': +1.60, 'y': -1.55},
            'X': {'x': -3.15, 'y': -0.15}
        }
        
        
        self.remaining_points = ['A', 'B', 'C', 'D']
        
        self.current_pose = None
        self.current_yaw = 0.0
        self.target_point = None
        self.state = 'IDLE'

        # --- CONFIGURAÇÃO DA MANOBRA ---
        # Distância antes do X para começar a girar
        self.maneuver_distance = 0.7 
        
        # LADO DO GIRO: 
        # 'LEFT' = Anti-Horário (Gira para esquerda)
        # 'RIGHT' = Horário (Gira para direita)
        # Escolha o lado oposto ao braço ou o lado seguro.
        self.preferred_turn_side = 'CENTER' 

        # Variáveis Bézier
        self.bezier_path = []
        self.bezier_index = 0
        self.bezier_step = 0.20

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info(f"Iniciado. Lado de giro forçado: {self.preferred_turn_side}")

        # --- LOG DE DADOS (CSV) ---
        log_path = os.path.expanduser("~/trajetoria_bezier_vs_odometria.csv")
        self.csv_file = open(log_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        # Cabeçalho
        self.csv_writer.writerow([
            "time",
            "x_odom", "y_odom", "yaw_odom",
            "x_bezier", "y_bezier",
            "state"
        ])

        self.start_time = time.time()
        self.get_logger().info(f"Log CSV salvo em: {log_path}")


    # ==================== UTILITÁRIOS ===========================

    def log_data(self):
        if self.current_pose is None:
            return

        # Tempo relativo
        t = time.time() - self.start_time

        # Odometria
        x_odom = self.current_pose.position.x
        y_odom = self.current_pose.position.y
        yaw_odom = self.current_yaw

        # Referência Bézier (se existir)
        if self.bezier_path and self.bezier_index < len(self.bezier_path):
            x_ref, y_ref = self.bezier_path[self.bezier_index]
        else:
            x_ref, y_ref = None, None

        self.csv_writer.writerow([
            t,
            x_odom, y_odom, yaw_odom,
            x_ref, y_ref,
            self.state
        ])


    def normalize_angle(self, angle):
        """Mantém entre -pi e pi"""
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose
        q = self.current_pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y*q.y + q.z*q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def start_callback(self, msg):
        if msg.data == "start":
            self.start_seq += 1
            self.started = True

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def get_distance_to_point(self, p_dict):
        if not self.current_pose: return 99.9
        dx = p_dict['x'] - self.current_pose.position.x
        dy = p_dict['y'] - self.current_pose.position.y
        return math.sqrt(dx*dx + dy*dy)

    # ==================== BÉZIER ===========================
    def bezier_point(self, t, p0, p1, p2, p3):
        x = (1-t)**3 * p0[0] + 3*(1-t)**2 * t * p1[0] + 3*(1-t)*t**2 * p2[0] + t**3 * p3[0]
        y = (1-t)**3 * p0[1] + 3*(1-t)**2 * t * p1[1] + 3*(1-t)*t**2 * p2[1] + t**3 * p3[1]
        return (x, y)

    def generate_bezier_curve(self, start, end, start_yaw=None, backward_start=False):
        p0 = start
        p3 = end
        dist = math.sqrt((end[0]-start[0])**2 + (end[1]-start[1])**2)
        offset = dist * 0.5

        if start_yaw is not None:
            angle = start_yaw + math.pi if backward_start else start_yaw
            p1 = (start[0] + offset * math.cos(angle), start[1] + offset * math.sin(angle))
        else:
            p1 = (start[0] + offset, start[1] + offset)

        p2 = (end[0] - (end[0]-start[0])*0.2, end[1] - (end[1]-start[1])*0.2)

        curve = []
        t = 0.0
        while t <= 1.0:
            curve.append(self.bezier_point(t, p0, p1, p2, p3))
            t += self.bezier_step
        return curve

    # ==================== MOVIMENTOS ===========================
    def follow_bezier(self, reverse=False):
        if self.bezier_index >= len(self.bezier_path):
            return True

        target = self.bezier_path[self.bezier_index]
        dx = target[0] - self.current_pose.position.x
        dy = target[1] - self.current_pose.position.y
        dist = math.sqrt(dx*dx + dy*dy)

        if dist < 0.3:
            self.bezier_index += 1
            return False

        angle_target = math.atan2(dy, dx)
        
        if reverse:
            desired_yaw = angle_target + math.pi
            linear_factor = -1.0
        else:
            desired_yaw = angle_target
            linear_factor = 1.0

        # Erro angular padrão para seguir linha
        angle_error = self.normalize_angle(desired_yaw - self.current_yaw)

        twist = Twist()
        #v_linear = min(0.4 * dist, 0.4) * linear_factor

        base_speed = 0.4 * dist
        # Se a velocidade calculada for menor que 0.1, força ser 0.1
        if abs(base_speed) < 0.15:
            base_speed = 0.15

        v_linear = min(base_speed, 0.4) * linear_factor
        
        # Se erro for grande, reduz linear
        if abs(angle_error) > 0.5:
            v_linear *= 0.15

        twist.linear.x = v_linear
        twist.angular.z = 0.5 * angle_error
        self.cmd_pub.publish(twist)
        return False

    def perform_forced_turn(self):
        """
        Gira até a traseira apontar para X, FORÇANDO o lado da rotação.
        """
        # 1. Ângulo ideal da TRASEIRA para o ponto X
        dx = self.points['X']['x'] - self.current_pose.position.x
        dy = self.points['X']['y'] - self.current_pose.position.y
        angle_front_to_target = math.atan2(dy, dx)
        target_yaw_rear = angle_front_to_target + math.pi
        
        # 2. Calcula erro simples (-pi a pi)
        raw_error = target_yaw_rear - self.current_yaw
        normalized_error = math.atan2(math.sin(raw_error), math.cos(raw_error))
        
        # 3. FORÇA O SENTIDO DO GIRO
        # Se queremos virar para ESQUERDA, o erro deve ser POSITIVO (0 a 2pi)
        if self.preferred_turn_side == 'LEFT':
            if normalized_error < 0:
                normalized_error += 2 * math.pi 
        
        # Se queremos virar para DIREITA, o erro deve ser NEGATIVO (0 a -2pi)
        elif self.preferred_turn_side == 'RIGHT':
            if normalized_error > 0:
                normalized_error -= 2 * math.pi

        # 4. Verifica se chegou (tolerância)
        # Verifica a versão normalizada curta para saber se alinhou
        short_error = self.normalize_angle(normalized_error)
        
        if abs(short_error) < 0.08: # ~5 graus de tolerância
            self.stop_robot()
            return True

        # 5. Aplica comando
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.8 * normalized_error # Proporcional ao erro forçado
        
        # Trava velocidade máxima para não girar loucamente
        twist.angular.z = max(min(twist.angular.z, 0.6), -0.6)
        
        self.cmd_pub.publish(twist)
        return False

    # ==================== MÁQUINA DE ESTADOS ===========================
    def control_loop(self):
        if self.current_pose is None: return

        # 1. IDLE: Escolhe caixa aleatória (A, B, C, D) e cria rota de ida
        if self.state == 'IDLE':
            if not self.remaining_points:
                self.state = 'FINISHED'
                self.get_logger().info("Missão Completa!")
                return

            key = random.choice(self.remaining_points)
            self.remaining_points.remove(key)
            self.target_point = self.points[key]
            
            # Rota FRENTE até a caixa
            start = (self.current_pose.position.x, self.current_pose.position.y)
            self.bezier_path = self.generate_bezier_curve(start, (self.target_point['x'], self.target_point['y']), self.current_yaw, False)
            self.bezier_index = 0
            self.state = 'GOING_BOX'
            self.get_logger().info(f"--- Indo para caixa: {key}---")

        # 2. GOING_BOX: Navega até a caixa
        elif self.state == 'GOING_BOX':
            if self.follow_bezier(reverse=False):
                self.stop_robot()
                #time.sleep(0.5) 
                #time.sleep(5.0) #Aguarda 5 segundos
                
                # Rota FRENTE de volta para X
                start = (self.current_pose.position.x, self.current_pose.position.y)
                self.bezier_path = self.generate_bezier_curve(start, (self.points['X']['x'], self.points['X']['y']), self.current_yaw, False)
                self.bezier_index = 0
                self.state = 'RETURNING_FRONT'
                self.get_logger().info("Voltando para X")

        # 3. RETURNING_FRONT: Volta até chegar perto de X (maneuver_distance)
        elif self.state == 'RETURNING_FRONT':
            dist_x = self.get_distance_to_point(self.points['X'])
            
            # Se chegou na distância de manobra (ex: 0.7m do alvo)
            if dist_x < self.maneuver_distance:
                self.stop_robot()
                self.state = 'MANEUVERING'
                self.get_logger().info(f"Distância {dist_x:.2f}m. Iniciando MANOBRA...")
            else:
                # Continua andando
                if self.follow_bezier(reverse=False):
                    # Fallback caso a curva acabe antes da distância (raro)
                    self.state = 'MANEUVERING'

        # 4. MANEUVERING: Gira no próprio eixo (Lado Forçado)
        elif self.state == 'MANEUVERING':
            if self.perform_forced_turn():
                # Terminou giro. Agora calcula a Ré final
                start = (self.current_pose.position.x, self.current_pose.position.y)
                end = (self.points['X']['x'], self.points['X']['y'])
                
                # Gera curva saindo de RÉ (backward_start=True)
                self.bezier_path = self.generate_bezier_curve(start, end, self.current_yaw, backward_start=True)
                self.bezier_index = 0
                self.state = 'DOCKING'
                self.get_logger().info("Alinhado! Dando ré final...")

        # 5. DOCKING: Ré final até o ponto exato X
        elif self.state == 'DOCKING':
            if self.follow_bezier(reverse=True):
                self.stop_robot()
                self.state = 'WAITING'
                self.get_logger().info("Chegada em X concluída.")

        # 6. WAITING: Aguarda Start e Mede Volume
        elif self.state == 'WAITING':
            # Solicita medição 1 vez
            if (self.start_seq == self.start_seq_processed) and (not self.volume_requested):
                msg = String()
                msg.data = "measure"
                self.volume_request_pub.publish(msg)
                self.volume_requested = True 
                self.get_logger().info("Solicitando medição...")

            # Espera comando externo
            if ((rclpy.ok() and self.started) and (self.start_seq > self.start_seq_processed)):
                self.start_seq_processed = self.start_seq
                self.state = 'IDLE'
                self.started = False
                self.volume_requested = False
                self.get_logger().info("Reiniciando ciclo...")

        elif self.state == 'FINISHED':
            self.stop_robot()

        self.log_data()


def main(args=None):
    rclpy.init(args=args)
    controller = PioneerFinalController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt: pass
    finally:
        controller.stop_robot()
        controller.csv_file.close()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
