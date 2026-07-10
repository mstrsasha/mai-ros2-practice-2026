import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


class DrawFigureNode(Node):
    def __init__(self):
        super().__init__('draw_figure_node')
        
        #публкую команды скорости
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        #подписан на одометрию
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        #вызывается 20 раз в секунду
        self.timer = self.create_timer(0.05, self.control_loop)
    
        self.linear_speed = 0.15
        self.angular_speed = 0.4  
        self.pos_tolerance = 0.02
        self.ang_tolerance = 0.02
        
        #текущее состояние из одометрии
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False
        
        #квадрат
        self.actions = [
            ('fwd', 1.0),
            ('turn', -math.pi/2),
            ('fwd', 1.0),
            ('turn', -math.pi/2),
            ('fwd', 1.0),
            ('turn', -math.pi/2),
            ('fwd', 1.0),
            ('turn', -math.pi/2),
        ]
        
        self.current_action_idx = 0
        self.phase = 'idle'
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0
        self.target_dist = 0.0
        self.target_angle = 0.0
        self.direction = 1.0
        
        self.get_logger().info('Node started. Waiting for odom...')
    
    def odom_callback(self, msg):
        """Получаем текущую позицию и ориентацию робота"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        #yaw из кватерниона
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.odom_received = True
    
    def normalize_angle(self, angle):
        """приводим углы в диапазон [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def control_loop(self):
        """цикл управления"""
        if not self.odom_received:
            return
        
        if self.phase == 'done':
            return
        
        if self.phase == 'idle':
            if self.current_action_idx >= len(self.actions):
                self.stop_robot()
                self.get_logger().info('Figure completed!')
                self.phase = 'done'
                return
            
            action_type, value = self.actions[self.current_action_idx]
            self.get_logger().info(
                f'Action {self.current_action_idx}: {action_type} {value:.3f}')
            
            if action_type == 'fwd':
                self.phase = 'fwd'
                self.start_x = self.current_x
                self.start_y = self.current_y
                self.target_dist = abs(value)
                self.direction = 1.0 if value >= 0 else -1.0
            else:  # turn
                self.phase = 'turn'
                self.start_yaw = self.current_yaw
                self.target_angle = abs(value)
                self.direction = 1.0 if value >= 0 else -1.0
            return
        
        #едем вперёд
        if self.phase == 'fwd':
            #считаем сколько уже проехали после старта
            dist = math.sqrt(
                (self.current_x - self.start_x)**2 + 
                (self.current_y - self.start_y)**2)
            #если проехали достаточно >= проехали - погрешность ==> стоп
            if dist >= self.target_dist - self.pos_tolerance:
                self.stop_robot()
                self.current_action_idx += 1
                self.phase = 'idle'
                self.get_logger().info(f'  -> moved {dist:.3f} m')
                return
            #продолжаем ехать
            msg = Twist()
            msg.linear.x = self.linear_speed * self.direction
            self.publisher.publish(msg)
            return
        
        #поворот
        if self.phase == 'turn':
            angle_diff = self.normalize_angle(self.current_yaw - self.start_yaw)
            angle_done = abs(angle_diff)
            
            #проверка + учитываем направление!!
            if self.direction > 0:
                done = angle_diff >= self.target_angle - self.ang_tolerance
            else:
                done = angle_diff <= -self.target_angle + self.ang_tolerance
            
            if done:
                self.stop_robot()
                self.current_action_idx += 1
                self.phase = 'idle'
                self.get_logger().info(f'  -> turned {angle_done:.3f} rad')
                return
            
            msg = Twist()
            msg.angular.z = self.angular_speed * self.direction
            self.publisher.publish(msg)
            return
    
    def stop_robot(self):
        """останавливаем робота"""
        msg = Twist()
        for _ in range(10):
            self.publisher.publish(msg)


def main():
    rclpy.init()
    node = DrawFigureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()