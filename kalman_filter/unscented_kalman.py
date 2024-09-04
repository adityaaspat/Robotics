import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math
import numpy as np
# from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose2D
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints

class Unscented_Kalman(Node):
    def __init__(self):
        super().__init__('unscented_kalman')
        
        self.imu_subscription = self.create_subscription(Imu,'imu',self.imu_callback,10)
        self.cmd_vel_subscription = self.create_subscription(Twist,'cmd_vel',self.cmd_vel_callback,10)
        self.pose_publisher = self.create_publisher(Pose2D, 'ukf_poses', 10)
        self.timer = self.create_timer(0.1, self.publish_pose)

        self.measurement= np.array([0.0,0.0,0.0])
        
  
        
        self.points = MerweScaledSigmaPoints(n=8, alpha=0.1, beta=2, kappa=0.0)
        self.ukf = UKF(dim_x=8, dim_z=3, dt=0.1, fx=self.f, hx=self.h, points=self.points)
        
        self.ukf.x = np.zeros((8,)) #initial state
        self.ukf.P = np.eye(8) * 0.1   # Initial covariance
        
        self.ukf.R = np.eye(3)*0.1   # Measurement noise covariance
        self.ukf.Q = np.eye(8) * 0.1  # Process noise covariance
        
  
        self.cmd_flag=0
        self.imu_flag=1
        
        
    def imu_callback(self, msg):
            self.measurement=np.array([msg.angular_velocity.z,msg.linear_acceleration.x,msg.linear_acceleration.y])
            self.imu_flag=1
    
    def cmd_vel_callback(self, msg):
        # Handle the cmd_vel data here
        if self.imu_flag ==1:
            v= msg.linear.x
            w=msg.angular.z
            uin=np.array([v,w])
            self.ukf.predict(u=uin)
            self.ukf.update(self.measurement)
            
            print(f"\n X hat= {self.ukf.x} \n Measurement={self.measurement} \n P={self.ukf.P} \n",)
            
            self.imu_flag=0
            self.cmd_flag=1
        
    def f(self,x,dt,u):
        v=u[0]
        w=u[1]
        # Predicted state (non-linear motion model)
        theta=x[2]
        x[0]+= v * np.cos(theta) * dt + 0.5 * x[6] * dt**2
        x[1] += v * np.sin(theta) * dt + 0.5 * x[7] * dt**2
        x[2] += w * dt
        x[3] += v * np.cos(theta)
        x[4] += v * np.sin(theta)
        x[5] = w
        x[6] = x[6]
        x[7] = x[7]
        return x
        

    def h(self,x):
        return np.array([x[5],x[6],x[7]])
    
    def publish_pose(self):
        if self.cmd_flag ==1:
            msg = Pose2D()
            msg.x = self.ukf.x[0]
            msg.y = self.ukf.x[1]
            msg.theta = self.ukf.x[2]
            self.pose_publisher.publish(msg)
            # self.get_logger().info(f'Publishing: x={msg.x}, y={msg.y}, theta={msg.theta}')
            self.cmd_flag=0
        
        
def main(args=None):
    rclpy.init(args=args)
    kalman_node = Unscented_Kalman()
    rclpy.spin(kalman_node)
    kalman_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()