import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math
import numpy as np
# from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose2D


class Extended_Kalman(Node):
    def __init__(self):
        super().__init__('extended_kalman')
        
        self.imu_subscription = self.create_subscription(Imu,'imu',self.imu_callback,10)
        self.cmd_vel_subscription = self.create_subscription(Twist,'cmd_vel',self.cmd_vel_callback,10)
        self.subscription = self.create_subscription(Odometry,'odom',self.odom_callback,10)
        self.pose_publisher = self.create_publisher(Pose2D, 'ekf_poses', 10)
        self.timer = self.create_timer(0.1, self.publish_pose)

        self.cmd_flag=0
        self.state=None
        self.measurement= np.array([0.0,0.0,0.0]).reshape(3,1)
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
        
        
    
    def odom_callback(self, msg):
        self.odom_flag=1
        
    def imu_callback(self, msg):
            self.measurement=np.array([msg.angular_velocity.z,msg.linear_acceleration.x,msg.linear_acceleration.y]).reshape(3,1)
            self.imu_flag=1
    
    def cmd_vel_callback(self, msg):
        # Handle the cmd_vel data here
        if self.imu_flag ==1:
            v= msg.linear.x
            w=msg.angular.z
            self.poses.append([self.x_hat[0,0],self.x_hat[1,0],self.x_hat[2,0]])
            
            print(f"\n X hat= {self.x_hat} \n Measurement={self.measurement} \n P={self.P} \n",)
            
            x_hat_, P_ = self.prediction(self.x_hat, self.P,v,w)

            # Perform correction step using measurements and sensor flags (gps, imu)
            x_hat, P = self.correction(x_hat_, P_, self.measurement)
            self.x_hat=x_hat
            self.P=P
            self.imu_flag=0
            self.cmd_flag=1
        
    def prediction(self,x_hat, P,v,w ,dt=0.1):
        

        # Predicted state (non-linear motion model)
        
        theta=x_hat[2,0]
                
        ay = x_hat[7, 0]
        x_hat[0,0]+= v * np.cos(theta) * dt + 0.5 * x_hat[6, 0] * dt**2
        x_hat[1,0] += v * np.sin(theta) * dt + 0.5 * x_hat[7, 0] * dt**2
        x_hat[2,0] += w * dt
        x_hat[3, 0] += v * np.cos(theta)
        x_hat[4, 0] += v * np.sin(theta)
        x_hat[5,0] = w
  
        
        # Jacobian of the motion model (F)
        F = np.eye(8)
        F[0, 2] = -v * np.sin(theta) * dt
        F[0, 6] = 0.5 * dt**2  #0.5 * dt**2 if you wanna add bias uncomment
        F[1, 2] = v * np.cos(theta) * dt
        F[1, 7] = 0.5 * dt**2  #0.5 * dt**2
        F[2, 5] = dt
        F[3, 2] = -v * np.sin(theta)
        F[4, 2] = v * np.cos(theta)
        F[5, 5] = 1
        F[6, 6] = 1
        F[7, 7] = 1
        
        Q = np.diag([0.1]*8)

        # Predicted state covariance
        P = F @ P @ F.T + Q
        return x_hat, P
    
    def correction(self,x_hat_minus, P_minus, measurement):
        
        # Initialize Jacobian matrix H with zeros
        H = np.zeros((3, 8))
        
        # Measurement vector is [w, ax, ay]
        H[0, 5] = 1  # ∂w/∂w = 1
        H[1, 6] = 1  # ∂ax/∂ax = 1
        H[2, 7] = 1  # ∂ay/∂ay = 1        
        # Compute the innovation/residual
        y = measurement - H @ x_hat_minus
        
        R= np.eye(3)*0.1
        
        # Compute the innovation covariance
        S = H @ P_minus @ H.T + R
        
        # Compute the Kalman gain
        K = P_minus @ H.T @ np.linalg.inv(S)
        
        # Update the state vector
        x_hat = x_hat_minus + K @ y
        
        # Update the state covariance matrix
        P = (np.eye(8)- K @ H) @ P_minus
        return x_hat,P
    
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
    kalman_node = Extended_Kalman()
    rclpy.spin(kalman_node)
    kalman_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()