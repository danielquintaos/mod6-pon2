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


def main(args=None):
    rclpy.init(args=args)
    move = heliogabalus()

    time.sleep(2.0)

    move.destroy_node()
    rclpy.shutdown()