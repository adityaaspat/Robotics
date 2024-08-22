import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose2D


class Kalman(Node):
    def __init__(self):
        super().__init__('kalman')
        
        self.imu_subscription = self.create_subscription(Imu,'imu',self.imu_callback,10)
        self.cmd_vel_subscription = self.create_subscription(Twist,'cmd_vel',self.cmd_vel_callback,10)
        self.subscription = self.create_subscription(Odometry,'odom',self.odom_callback,10)
        self.pose_publisher = self.create_publisher(Pose2D, 'pose_topic', 10)
        self.timer = self.create_timer(0.1, self.publish_pose)

        self.cmd_flag=0
        self.state=None
        self.measurement= np.array([0.0,0.0,0.0,0.0,0.0]).reshape(5,1)
        self.x_hat = np.zeros((8, 1)) #initial state
        # Initial covariance matrix
        sigma_x_0 = 100  # Adjust based on your estimation of the initial uncertainty
        sigma_y_0 = 100
        sigma_theta_0 = 100
        sigma_dot_x_0 = 100
        sigma_dot_y_0 = 100
        sigma_dot_theta_0 = 0.1
        self.P = np.diag([sigma_x_0**2, sigma_y_0**2, sigma_theta_0**2, 
                    sigma_dot_x_0**2, sigma_dot_y_0**2, sigma_dot_theta_0**2
                    ,sigma_dot_y_0**2, sigma_dot_y_0**2])
        self.odom_flag=1
        self.imu_flag=1
        self.poses=[]
        
        
    def quaternion_to_euler(self,q):
        r = R.from_quat(q)
        return r.as_euler('xyz', degrees=True)
    
    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose.position
        self.robot_x=self.robot_pose.x
        self.robot_y=self.robot_pose.y
        orientation= msg.pose.pose.orientation
        self.odom_quaternions=[orientation.x, orientation.y, orientation.z ,orientation.w]
        _,_,self.robot_theta=self.quaternion_to_euler(self.odom_quaternions)
        self.robot_theta=self.robot_theta*np.pi/180
        self.state=[self.robot_x,self.robot_y,self.robot_theta]
        self.measurement[0,0]=self.robot_x
        self.measurement[1,0]=self.robot_y
        self.odom_flag=1
        
    def imu_callback(self, msg):
        # Handle the imu data here
            self.measurement[2:,0]=msg.angular_velocity.z,msg.linear_acceleration.x,msg.linear_acceleration.y
            # print(f'X={self.robot_x} Y={self.robot_y} Theta={self.robot_theta}')
            self.imu_flag=1
    
    def cmd_vel_callback(self, msg):
        # Handle the cmd_vel data here
        if self.imu_flag==1 and self.odom_flag==1:
            self.cmd_flag=1
            v= msg.linear.x
            w=msg.angular.z
            self.poses.append([self.x_hat[0,0],self.x_hat[1,0],self.x_hat[2,0]])
            print(f"\n X hat= {self.x_hat} \n P={self.P} \n",)
            x_hat_, P_ = self.prediction(self.x_hat, self.P,v,w)

            # Perform correction step using measurements and sensor flags (gps, imu)
            x_hat, K, P = self.correction(x_hat_, P_, self.measurement)
            self.x_hat=x_hat
            self.P=P
            self.imu_flag=0
            self.odom_flag=0
            # print(f'Cmd_vel linear: {v}, angular: {w}')
        
    def prediction(self,x_hat, P,v,w ,dt=0.1):

        # State transition matrix
        F = np.array(
        [[1.0, 0.0, 0.0, dt, 0.0, 0.0, 0.5*dt**2, 0.0],     # x
        [0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0, 0.5*dt**2],      # y
        [0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0],            # θ
        [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],            # v_x
        [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],            # v_y
        [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],           # theta_dot
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],           # a_x
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])          # a_y

        # Control input matrix (assuming no control input in this example)
        B = np.array([
            [1,0], #x+ vx*t
            [0,0], #y +vy*t
            [0,1], #theta +w*t
            [1,0], #vx
            [0,0], #vy
            [0,1], #w
            [0,0], #ax
            [0,0], #ay
        ])

        # Control input (assuming no control input in this example)
        u_k = np.array([[v],
                        [w]])

        # Prediction step
        x_hat_ = np.dot(F, x_hat)  + np.dot(B, u_k) 

        # Process noise covariance matrix
        Q = np.diag([0.1]*8)

        # Covariance prediction
        P_ = np.dot(np.dot(F, P), F.T) + Q

        return x_hat_, P_
    
    def correction(self,x_hat_minus, P_minus, measurement):
        """
        Correction step of the Kalman Filter.'

        Parameters:
        - x_hat_minus: Predicted state estimate at time k (numpy array)
        - K: Kalman Gain (numpy array)
        - measurement: Measurement at time k (numpy array)

        Returns:
        - Corrected state estimate at time k (numpy array)"""
        H = np.array([
        [1, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 1]]) # Measurement matrix, assuming direct observation of x,y,w,ax,ay)

        # Measurement noise covariance matrix
        std_dev_x = 0.1  # Example value for x
        std_dev_y = 0.1  # Example value for y
        std_dev_w = 1  # Example value for theta
        std_dev_ax = 1 # Example value for a_x
        std_dev_ay =1  # Example value for a_y
    
        R = np.array([
            [std_dev_x**2, 0, 0, 0, 0],
            [0, std_dev_y**2, 0, 0, 0],
            [0, 0, std_dev_w**2, 0, 0],
            [0, 0, 0, std_dev_ax**2, 0],
            [0, 0, 0, 0, std_dev_ay**2] # Adjust if needed based on actual observations
        ])
            
        # Calculate the innovation (residual)
        innovation = measurement - np.dot(H, x_hat_minus)
        
        # Calculate the innovation covariance
        S = np.dot(np.dot(H, P_minus), H.T) + R
        
        # Calculate the Kalman Gain
        K = np.dot(np.dot(P_minus, H.T), np.linalg.inv(S))
        
        # Correct the state estimate
        x_hat = x_hat_minus + np.dot(K, innovation)
        
        # Correct the covariance matrix
        P = P_minus - np.dot(np.dot(K, H), P_minus)
    
        return x_hat, K, P 
    
    def publish_pose(self):
        if self.cmd_flag ==1:
            msg = Pose2D()
            msg.x = self.x_hat[0,0]
            msg.y = self.x_hat[1,0]
            msg.theta = self.x_hat[2,0]
            self.pose_publisher.publish(msg)
            # self.get_logger().info(f'Publishing: x={msg.x}, y={msg.y}, theta={msg.theta}')
            self.cmd_flag=0
        
        
def main(args=None):
    rclpy.init(args=args)
    kalman_node = Kalman()
    rclpy.spin(kalman_node)
    kalman_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()