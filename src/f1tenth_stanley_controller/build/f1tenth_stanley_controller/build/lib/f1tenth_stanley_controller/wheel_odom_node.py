#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64,Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState,Imu
import math
import tf_transformations
from tf_transformations import euler_from_quaternion, normalize_angle

class WheelOdomNode(Node):
    def __init__(self):
        super().__init__('wheel_odom_node')
        assert self.wheel_diameter > 0, "Invalid wheel diameter"
        assert self.ticks_per_rev > 0, "Invalid encoder resolution"
        # Initialize encoder counters and pose
        self.left_ticks = 0
        self.right_ticks = 0
        self.last_left_ticks = 0
        self.last_right_ticks = 0
        self.steering_angle = 0
        # self.x = 0.
        self.x = 0.74
        self.y = 2.8418
        self.theta = 1.57
        
        # Robot physical parameters (modify these for your robot)
        self.wheel_separation = 0.2360   # Distance between wheels in meters
        self.wheel_diameter = 0.0590*2     # Wheel diameter in meters
        self.wheelbase  = 0.3240
        self.ticks_per_rev = 16     # Encoder ticks per full revolution
        self.imu_data = None
        # Calculate distance per tick
        self.distance_per_tick = (math.pi * self.wheel_diameter) / self.ticks_per_rev
        
        # Create publisher and subscribers
        self.odom_pub = self.create_publisher(Odometry, 'wheel_odometry', 10)
        self.create_subscription(JointState, '/autodrive/f1tenth_1/left_encoder', self.left_callback, 10)
        self.create_subscription(JointState, '/autodrive/f1tenth_1/right_encoder', self.right_callback, 10)
        self.create_subscription(Float32, '/autodrive/f1tenth_1/steering', self.steering_callback, 10)
        self.create_subscription(Imu,'/autodrive/f1tenth_1/imu',self.imu_callback,10)
        # Initialize time tracking
        self.last_time = self.get_clock().now()
        #self.get_logger().info("hellooo")
        #print("yes")

        # Create timer for periodic updates (50Hz)
        self.create_timer(0.02, self.update_odometry)

    def left_callback(self, msg):
        if len(msg.position) > 0:
            self.left_ticks = msg.position[0]
        else:
            self.get_logger().warn('Empty encoder message received')    

    def right_callback(self, msg):
        if len(msg.position) > 0:
            self.right_ticks = msg.position[0]
        else:
            self.get_logger().warn('Empty encoder message received')  

    def steering_callback(self, msg):

        self.steering_angle = msg.data  # Expecting radians
        print(self.steering_angle)
    
    def imu_callback(self,msg):
        orientation_q = msg.orientation
        angular_velocity = msg.angular_velocity.z
        linear_acceleration_x = msg.linear_acceleration.x

        # Convert quaternion to yaw (orientation about z-axis)
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        
        # Store IMU data in a format usable by the EKF
        self.imu_data = {
            'yaw': yaw,
            'angular_velocity': angular_velocity,
            'linear_acceleration': linear_acceleration_x,
            'covariance': msg.orientation_covariance,
        }

    def update_odometry(self):
        if not(self.imu_data):
            return
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt == 0:
            return

        # Calculate rear wheel displacements
        delta_left = self.left_ticks - self.last_left_ticks
        delta_right = self.right_ticks - self.last_right_ticks
        left_dist = delta_left * self.distance_per_tick
        right_dist = delta_right * self.distance_per_tick
        
        #
        # Replace linear displacement calculation with:
        angular_velocity = (linear_velocity * math.tan(self.steering_angle)) / self.wheelbase
        self.theta += angular_velocity * dt  # Integrate heading change


        # Average rear wheel movement for linear displacement
        linear_displacement = (left_dist + right_dist) / 2.0
        linear_velocity = linear_displacement / dt

        # Use complementary filter for pose
        imu_yaw = self.imu_data['yaw']
        wheel_yaw = self.theta + angular_velocity * dt
        self.theta = 0.98*wheel_yaw + 0.02*imu_yaw  # Adjust filter ratio as needed

        self.theta = normalize_angle(self.theta)

        self.x += linear_velocity * math.cos(self.theta) * dt
        self.y += linear_velocity * math.sin(self.theta) * dt

        # Create and populate odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'f1tenth_1'
        
        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        
        # Orientation (convert to quaternion)
        q = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
        # Velocity
        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.angular.z = self.imu_data["angular_velocity"]
        
        # Covariance matrix (adjust values based on your system's uncertainty)
        odom_msg.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05
        ]

        # Publish and update state
        self.odom_pub.publish(odom_msg)
        self.last_left_ticks = self.left_ticks
        self.last_right_ticks = self.right_ticks
        self.last_time = now

def main(args=None):
    rclpy.init(args=args)
    node = WheelOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # Handle Ctrl+C gracefully
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()