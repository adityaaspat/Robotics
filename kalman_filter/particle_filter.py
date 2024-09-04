import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math
import numpy as np
# from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose2D
from filterpy.monte_carlo import systematic_resample
from numpy.random import uniform, randn
from scipy.stats import norm


class Particle_Filter(Node):
    def __init__(self):
        super().__init__('particle_filter')
        
        self.imu_subscription = self.create_subscription(Imu,'imu',self.imu_callback,10)
        self.cmd_vel_subscription = self.create_subscription(Twist,'cmd_vel',self.cmd_vel_callback,10)
        self.pose_publisher = self.create_publisher(Pose2D, 'pf_poses', 10)
        self.timer = self.create_timer(0.1, self.publish_pose)
        num_particles=1500
        x_range = [-0.05, 0.05]
        y_range = [-0.05, 0.05]
        theta_range = [-0.01, 0.10]
        v_range = [-0.01, 0.01]
        a_range = [-0.00001, -0.00001]
        
        self.particles = self.create_particles(num_particles, x_range, y_range, theta_range, v_range, a_range)
        self.weights = np.ones(num_particles) / num_particles
        
        self.measurement= np.array([0.0,0.0,0.0])
        self.state= np.zeros(8)
        
        self.cmd_flag=0
        self.imu_flag=0
        
        
    def imu_callback(self, msg):
            self.measurement=np.array([msg.angular_velocity.z,msg.linear_acceleration.x,msg.linear_acceleration.y])
            self.imu_flag=1
    
    def cmd_vel_callback(self, msg):
        # Handle the cmd_vel data here
        if self.imu_flag ==1:
            v= msg.linear.x
            w=msg.angular.z
            
            self.predict(v,w)
            self.update()
            self.resample()
            self.state = np.average(self.particles, weights=self.weights, axis=0)
            print(f"\n States:{self.state} \n Measurement={self.measurement} \n")
            self.imu_flag=0
            self.cmd_flag=1
    
    def predict(self, v,w):
        self.motion_model(v,w)
        
        
    def motion_model(self, v,w ,dt=0.1):
        # x = self.particles[:,0]
        # y= self.particles[:,1]
        # theta=self.particles[:,2]
        # vx=self.particles[:,3]
        # vy=self.particles[:,4]
        # omega=self.particles[:,5]
        # ax=self.particles[:,6]
        # ay=self.particles[:,7]
        self.particles[:,0] += v*np.cos(self.particles[:,2])* dt + 0.5*self.particles[:,6]*dt**2
        self.particles[:,1] += v*np.sin(self.particles[:,2])* dt + 0.5*self.particles[:,7]*dt**2
        self.particles[:,2] += w * dt
        self.particles[:,3] = v*np.cos(self.particles[:,2]) 
        self.particles[:,4] = v*np.sin(self.particles[:,2]) 
        self.particles[:,5] = w

    
    def create_particles(self,num_particles, x_range, y_range, theta_range, v_range, a_range):
        particles = np.empty((num_particles, 8))
        particles[:, 0] = uniform(x_range[0], x_range[1], num_particles)  # x
        particles[:, 1] = uniform(y_range[0], y_range[1], num_particles)  # y
        particles[:, 2] = uniform(theta_range[0], theta_range[1], num_particles)  # theta
        particles[:, 3] = uniform(v_range[0], v_range[1], num_particles)  # vx
        particles[:, 4] = uniform(v_range[0], v_range[1], num_particles)  # vx
        particles[:, 5] = uniform(-0.01, 0.01, num_particles)  # w
        particles[:, 6] = uniform(a_range[0], a_range[1], num_particles)  # vx
        particles[:, 7] = uniform(a_range[0], a_range[1], num_particles)  # ay
        
        return particles
   
    def measurement_model(self, state):
        # Measurement model outputs only w, ax, ay
        w = self.particles[:, 5]
        ax = self.particles[:, 6]
        ay = self.particles[:, 7]
        return np.stack([w, ax, ay], axis=1)
    
    def update(self):
        
        pred_measurements = self.measurement_model(self.particles)
    
        # Compute the probability of each predicted measurement given the actual measurement
        # Using broadcasting, we can calculate the pdf for each particle across all measurements
        probs = norm(pred_measurements, 0.9).pdf(self.measurement)
        
        # Compute the product of probabilities across the measurements for each particle
        self.weights = probs.prod(axis=1)
        
        # Avoid rounding errors by adding a small value to weights
        self.weights += 1.e-300
        
        # Normalize the weights
        self.weights /= self.weights.sum()
        
    def resample(self):
        indices = systematic_resample(self.weights)
        self.particles[:] = self.particles[indices]
        self.weights[:] = self.weights[indices]
        self.weights.fill(1.0 / len(self.weights))
        
    def publish_pose(self):
        if self.cmd_flag ==1:
            msg = Pose2D()
            msg.x = self.state[0]
            msg.y = self.state[1]
            msg.theta = self.state[2]
            self.pose_publisher.publish(msg)
            # self.get_logger().info(f'Publishing: x={msg.x}, y={msg.y}, theta={msg.theta}')
            self.cmd_flag=0
        
        
def main(args=None):
    rclpy.init(args=args)
    kalman_node = Particle_Filter()
    rclpy.spin(kalman_node)
    kalman_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()