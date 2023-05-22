import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class heliogabalus(Node):
    def __init__(self, time_count=0.1):
        super().__init__('sun')
        
        self.get_logger().info("teste")

        self.rot = [7.0,-8.0, 4.0]
        self.rotIndex = 0
        self.rotEnd = len(self.rot)
        self.destination = self.rot[self.rotIndex]
        self.target = None

        self.velocity_publisher = self.create_publisher(
            msg_type=Twist,
            topic='/cmd_vel',
            qos_profile=10)
        
        self.twist = Twist()
        
        self.pose_subscription = self.create_subscription(
            msg_type=Odometry,
            topic='/odom',
            callback=self.pose_callback,
            qos_profile=10
        )

        self.odomData = None

        self.timer_count = self.create_timer(time_count, self.movement)
       
    def pose_callback(self, msg):
        self.odomData = msg

    def movement(self):
        if self.odomData is not None:
            x = self.odomData.pose.pose.position.x
            y = self.odomData.pose.pose.position.y
            z = self.odomData.pose.pose.position.z
            ang = self.odomData.pose.pose.orientation
            _, _, theta = euler_from_quaternion([ang.x, ang.y, ang.z, ang.w])
            self.get_logger().info("x={:.3f}, y={:.3f}, theta={:.3f}".format(x,y,theta))

            if self.target is None:
                self.target = self.destination + x

            if abs(x - self.target) > 1.0:
                self.twist.linear.x = 3.0 if self.target > x else -3.0
                self.velocity_publisher.publish(self.twist)
            else:
                self.twist.linear.x = 0.0
                self.velocity_publisher.publish(self.twist)
                self.rotIndex += 1
                self.destination = self.rot[self.rotIndex]
                self.target = None

            
def main(args=None):
    rclpy.init(args=args)
    move = heliogabalus()

    time.sleep(1.0)
    rclpy.spin(move)

    move.destroy_node()
    rclpy.shutdown()