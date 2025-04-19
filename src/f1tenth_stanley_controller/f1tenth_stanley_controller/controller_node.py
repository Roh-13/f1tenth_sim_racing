import rclpy
import csv
import time
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import PointStamped,Point32
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Float32

class StanleyController(Node):
    def __init__(self):
        super().__init__('stanley_controller')

        # Hyperparameters
        self.declare_parameter('k', 1.0)
        self.declare_parameter('ks', 2.0)
        self.declare_parameter('vel_max', 5.0)
        self.declare_parameter('min_vel_ratio', 0.25)
        self.declare_parameter('time_interval',0.01)
        # Constants
        self.angle_limit = np.pi / 6
        self.goal = 0
        self.count = 0
        self.flw = 'front_left_wheel'
        self.frw = 'front_right_wheel'
        self.blw = 'rear_left_wheel'
        self.brw = 'rear_right_wheel'
        self.ff = 'map'
        self.wayfile = '/home/autodrive_devkit/src/f1tenth_stanley_controller/f1tenth_stanley_controller/centerline_f1tenth.csv'
        # Parameters
        self.k = float(self.get_parameter('k').get_parameter_value().double_value)
        self.ks = float(self.get_parameter('ks').get_parameter_value().double_value)
        self.vel_max = float(self.get_parameter('vel_max').get_parameter_value().double_value)
        self.vel = 4.0
        self.min_vel_ratio = float(self.get_parameter('min_vel_ratio').get_parameter_value().double_value)
        self.time_interval = float(self.get_parameter('time_interval').get_parameter_value().double_value)

        # Transform Listener initialization
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)	


        # PID variables
        self.target_velocity = self.vel
        self.current_velocity = 0.0
        self.integral = 0.0
        self.prev_error = 0.0
        self.kp = 0.5
        self.ki = 0.05
        self.kd = 0.1

        # Load waypoints
        self.waypoints = []
        self.vec = []
        self.getWaypoints()
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints.')

        # Publishers
        qos = QoSProfile(depth=5)
        self.throttle_pub = self.create_publisher(Float32, '/autodrive/f1tenth_1/throttle_command', qos_profile=qos)
        self.steering_pub = self.create_publisher(Float32, '/autodrive/f1tenth_1/steering_command', qos_profile=qos)
        self.goal_pub = self.create_publisher(PointStamped, '/local_goal_point', qos_profile=qos)
        self.cloud_pub = self.create_publisher(PointCloud, '/waypoint_cloud', qos_profile=qos)

        # Subscribers
        self.create_subscription(Float32, '/autodrive/f1tenth_1/speed', self.speed_callback, qos)

        # Timer for control loop
        time.sleep(1)  # Wait for frames to  initialize
        self.timer = self.create_timer(self.time_interval, self.compute)

    def getWaypoints(self):
        with open(self.wayfile, mode='r') as file:
            reader = csv.reader(file)
            next(reader)
            #as origin and world differ in location
            #self.waypoints = [[(float(row[0])-3.63), -float(row[1])+8.52] for row in reader]
            self.waypoints = [[(float(row[0])-7.2), -float(row[1])+15.5] for row in reader]
            #self.waypoints = [[(float(row[0])), float(row[1]),float(row[2])] for row in reader]

    def speed_callback(self, msg: Float32):
        self.current_velocity = msg.data

    def getNearestWaypoint(self, x, y):
        min_dist = float("inf")
        nearest_idx = 0
        for i in range(len(self.waypoints) - 1):
            dist = np.sqrt((x - float(self.waypoints[i][0]))**2 + (y - float(self.waypoints[i][1]))**2)
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i
        self.publish_waypoints()
        return nearest_idx, float(self.waypoints[nearest_idx][0]), float(self.waypoints[nearest_idx][1])

    def publish_waypoints(self):
        point_array = PointCloud()
        point_array.header.frame_id = "map"
        point_array.header.stamp = self.get_clock().now().to_msg()
        for i in range(len(self.waypoints)-1):
            point=Point32()
            point.x=float(self.waypoints[i][0])
            point.y = float(self.waypoints[i][1])
            point.z = 0.0
            #self.goal_pub.publish(point)
            point_array.points.append(point)
        
        self.cloud_pub.publish(point_array)
    def compute(self):
        try:
            t_flw = self.tf_buffer.lookup_transform(self.ff, self.flw, rclpy.time.Time())
            t_frw = self.tf_buffer.lookup_transform(self.ff, self.frw, rclpy.time.Time())
            t_blw = self.tf_buffer.lookup_transform(self.ff, self.blw, rclpy.time.Time())
            t_brw = self.tf_buffer.lookup_transform(self.ff, self.brw, rclpy.time.Time())
        except TransformException as e:
            # Check the buffer length in code
            self.get_logger().warn(f'Transform error: {e}')
            return
        # Calculate front axle midpoint
        x0 = (t_flw.transform.translation.x + t_frw.transform.translation.x) / 2
        y0 = (t_flw.transform.translation.y + t_frw.transform.translation.y) / 2
        z0 = (t_flw.transform.translation.z + t_frw.transform.translation.z) / 2

        # Calculate back axle midpoint
        xb0 = (t_blw.transform.translation.x + t_brw.transform.translation.x) / 2
        yb0 = (t_blw.transform.translation.y + t_brw.transform.translation.y) / 2

        # Heading vector
        self.vec = [x0 - xb0, y0 - yb0, 0.0]

        # Determine goal point
        self.goal, x1, y1 = self.getNearestWaypoint(x0, y0)
        num = int(self.goal - np.power(1.05, self.vel / (self.vel_max * self.min_vel_ratio)))
        if num<0:
            num=num+len(self.waypoints)
        x2, y2 = float(self.waypoints[num][0]), float(self.waypoints[num][1])

        #self.get_logger().info(f"({x0,y0}) to ({x2,y2})")

        # Crosstrack and heading errors
        a, b, c = y2 - y1, x1 - x2, y1 * (x2 - x1) - x1 * (y2 - y1)
        e_t = abs(a * x0 + b * y0 + c) / np.sqrt(a**2 + b**2)
        dp = np.dot([x2 - x1, y2 - y1, 0.0], self.vec)
        psi_t = -np.sign(np.cross([x2 - x1, y2 - y1, 0.0], self.vec)[2]) * np.arccos(dp / (np.linalg.norm([x2 - x1, y2 - y1]) * np.linalg.norm(self.vec)))

        # Steering control
        delta_t = psi_t + np.sign(a * x0 + b * y0 + c) * np.arctan(self.k * (e_t / (self.ks + self.vel)))
        delta_t = max(min(delta_t, self.angle_limit), -self.angle_limit)
        self.steering_pub.publish(Float32(data=delta_t))
        
        # After calculating delta_t
        self.get_logger().info(f"Crosstrack error: {e_t},Steering error: {psi_t}")
        self.get_logger().info(f"Steering angle (delta_t): {delta_t}")

        self.target_velocity=self.vel*(1-0.75*abs(delta_t/self.angle_limit))

        # PID for throttle control
        error = self.target_velocity - self.current_velocity
        self.integral += error * self.time_interval
        derivative = (error - self.prev_error) / self.time_interval
        throttle = self.kp * error + self.ki * self.integral + self.kd * derivative
        throttle = max(0.0, min(2.0, throttle))
        self.prev_error = error

        # Publish throttle
        self.throttle_pub.publish(Float32(data=throttle))
        self.get_logger().info(f"Throttle command: {throttle}, velocity {self.current_velocity}")


def main(args=None):
    rclpy.init(args=args)
    node = StanleyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()




