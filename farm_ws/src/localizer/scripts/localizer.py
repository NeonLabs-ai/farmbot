#!/usr/bin/env python3
import numpy as np
from numpy.linalg import inv
import rospy
from std_msgs.msg import Float64,Time
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3,Pose
from farm_msgs.msg import GlobalState

class Localizer:

    def __init__(self):

        self.pub = rospy.Publisher('/global_state', GlobalState, queue_size=1)
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_cb)
        self.gps_sub = rospy.Subscriber('/gps', Pose, self.gps_cb)

        self.ax = 0.0
        self.ay = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.yaw_dot = 0.0
        self.noise_ax = 1
        self.noise_ay = 1
        self.x_m = 0.0
        self.y_m = 0.0
        self.gps_pose = Pose()

        self.I = np.identity(4)
        self.X = np.array[[0],[0],[0],[0]]
        self.Y = np.array[[self.x_m],[self.y_m]]
        self.Q = np.zeros([4,4])
        self.R = np.array([[self.noise_ax,0],[0,self.noise_ay]])
        self.H = np.array([[1,0,0,0],[0,1,0,0]])

        self.dt = 0.0
        self.prev_time = rospy.Time().to_sec()
        self.curr_time = rospy.Time().to_sec()

    def imu_cb(self, msg):
        
        self.curr_time = rospy.Time().to_sec()

        self.ax = msg.linear_acceleration.x
        self.ay = msg.linear_acceleration.y
        self.wz = msg.angular_velocity.z

        self.time_stamp = msg.header.stamp
        seconds = self.time_stamp.to_sec()
        self.dt = seconds - self.prev_time
        
        self.P = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        self.A = np.array([[1,0,self.dt,0],[0,1,0,self.dt],[0,0,1,0],[0,0,0,1]])
        self.B = np.array([[0.5*self.dt**2,0],[0,0.5*self.dt**2],[self.dt,0],[0,self.dt]])
        self.U = np.array([[self.ax],[self.ay]])

        dt_2 = self.dt * self.dt
        dt_3 = dt_2 * self.dt
        dt_4 = dt_3 * self.dt

        self.Q[0][0] = dt_4/4*self.noise_ax
        self.Q[0][2] = dt_3/2*self.noise_ax
        self.Q[1][1] = dt_4/4*self.noise_ay
        self.Q[1][3] = dt_3/2*self.noise_ay
        self.Q[2][0] = dt_3/2*self.noise_ax
        self.Q[2][2] = dt_2*self.noise_ax
        self.Q[3][1] = dt_3/2*self.noise_ay
        self.Q[3][3] = dt_2*self.noise_ay
        
        self.prev_time = self.curr_time

        self.predict()
        self.update(self.Y)

    def publish_state(self):
        state = GlobalState()
        state.x = self.x
        state.y = self.y
        state.yaw = self.yaw
        state.vx = self.vx
        state.vy = self.vy
        state.yaw_dot = self.yaw_dot
        self.pub.publish(state)

    def gps_cb(self, msg):
        self.x_m = msg.position.x
        self.y_m = msg.position.y
        self.Y = np.array([[self.x_m],[self.y_m]])
        list_y = np.array(self.Y)
        print("GPS values: ", list_y) 

    def predict(self):
        self.X = np.matmul(self.A, self.X) + np.matmul(self.B, self.U)
        list_X = np.array(self.X)
        print("Predicted values: ", list_X)
        At = np.transpose(self.A)
        self.P = np.matmul(self.A, np.matmul(self.P, At),self.Q)

    def update(self,Y):
        Ht = self.H.T
        self.S = np.add(np.matmul(self.H, np.matmul(self.P, Ht)) , self.R)
        self.K = np.matmul(self.P, Ht) @ inv(self.S)
        self.K = np.matmul(self.K, inv(self.S))
        list_k = np.array(self.K)
        print("Kalman gain: ", list_k)
        self.G = np.matmul(Y, np.matmul(self.H, self.X))

        self.X = self.X + self.K @ (Y - self.H @ self.X)
        pub_state = GlobalState()
        pub_state.x = self.X[0,0]
        pub_state.y = self.X[1,0]
        self.pub.publish(pub_state)
        self.P = (self.I - self.K @ self.H) @ self.P
        list_X = np.array(self.X)
        print("Updated values: ", list_X)

def main(argc, argv):
    rospy.init_node('localizer', anonymous=True)
    localizer = Localizer()
    rospy.spin()

if __name__ == "__main__":
    main()