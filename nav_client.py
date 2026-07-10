import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math


class NavClientNode(Node):
    def __init__(self):
        super().__init__('nav_client_node')
        
        self.action_client = ActionClient(
            self,   
            NavigateToPose,
            '/navigate_to_pose'
        )
        
        self.goals = [
            {'x': 2.3, 'y': -0.5, 'yaw': 0.0, 'name': 'верхний угол'},
            {'x': -2.5, 'y': 0.0, 'yaw': 0.0, 'name': 'нижний угол'},
            {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'name': 'вплотную к препятствию'},
        ]
        self.current_goal_idx = 0
        
        self.get_logger().info('Nav client initialized. Waiting for action server...')
        
        #ждем пока Nav2 запустится
        while not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Action server not available, waiting...')
        
        self.get_logger().info('Action server ready. Starting navigation')
        
        #отправляем первую цель
        self.send_next_goal()
    
    def create_pose_message(self, x, y, yaw):
        """создаёт сообщение PoseStamped из координат"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        
        #координаты
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        
        #кватернион из yaw
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        return pose
    
    def send_next_goal(self):
        """отправляет следующую цель из списка"""
        if self.current_goal_idx >= len(self.goals):
            self.get_logger().info('All goals completed!')
            rclpy.shutdown()
            return
        
        goal_data = self.goals[self.current_goal_idx]
        self.get_logger().info(
            f'\n=== Goal {self.current_goal_idx + 1}/{len(self.goals)}: '
            f'{goal_data["name"]} ({goal_data["x"]}, {goal_data["y"]}) ==='
        )
        
        #сообщение
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_pose_message(
            goal_data['x'], 
            goal_data['y'], 
            goal_data['yaw']
        )
        
        #отправляем цель
        self.get_logger().info('Sending goal...')
        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg,
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """вызывается, когда Nav2 принял или отклонил цель"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal REJECTED by Nav2')
            self.current_goal_idx += 1
            self.send_next_goal()
            return
        
        self.get_logger().info('Goal ACCEPTED. Robot is moving...')
        
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)
    
    
    def get_result_callback(self, future):
        """вызывается, когда робот доехал или не доехал."""
        result = future.result()
        status = result.status
        
        #статусы из action_msgs/GoalStatus
        STATUS_SUCCEEDED = 4
        STATUS_CANCELED = 5
        STATUS_ABORTED = 6
        
        goal_data = self.goals[self.current_goal_idx]
        
        if status == STATUS_SUCCEEDED:
            self.get_logger().info(
                f' Goal {self.current_goal_idx + 1} SUCCEEDED: '
                f'reached {goal_data["name"]}'
            )

        elif status == STATUS_CANCELED:
            self.get_logger().warn(
                f'  Goal {self.current_goal_idx + 1} CANCELED: '
                f'{goal_data["name"]}'
            )

        elif status == STATUS_ABORTED:
            self.get_logger().error(
                f' Goal {self.current_goal_idx + 1} ABORTED: '
                f'{goal_data["name"]} (не удалось доехать)'
            )

        else:
            self.get_logger().error(
                f' Goal {self.current_goal_idx + 1} unknown status: {status}'
            )
        
        #переходим к следующей
        self.current_goal_idx += 1
        self.send_next_goal()


def main():
    rclpy.init()
    node = NavClientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()