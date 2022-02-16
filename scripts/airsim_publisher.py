#!/usr/bin/env python
# license removed for brevity
import rospy
#from ublox.msg import PosVelEcef
#from ublox.msg import RelPos
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
import airsim

class AirsimPub():
    # Listen to AirSim information from UnrealEngine, and publish in ROS
    def __init__(self):
        # Setup message types
        self.imu = Imu()
        #self.rtk = RelPos()
        self.base = PoseStamped()
        self.rover = PoseStamped()
        # Setup publishers
        self.imuPub_ = rospy.Publisher('imu', Imu, queue_size=5, latch=True)
        self.roverPub_ = rospy.Publisher('rover_airsim', PoseStamped, queue_size=5, latch=True)
        self.basePub_ = rospy.Publisher('base_airsim', PoseStamped, queue_size=5, latch=True)
        # Airsim Client
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
    
        # Check airsim at 200? Hz and publish results if they've changed
        rate = rospy.Rate(200)
        while not rospy.is_shutdown():
            self.get_airsim()
            rate.sleep()

    def get_airsim(self):
        # Collect and publish airsim info
        time = rospy.Time.now()

        # Base
        self.base.header.stamp = time
        boat_state = self.client.simGetObjectPose('boat_2')
        self.base.pose.position.x = boat_state.position.x_val
        self.base.pose.position.y = boat_state.position.y_val
        self.base.pose.position.z = boat_state.position.z_val
        self.base.pose.orientation.x = boat_state.orientation.x_val
        self.base.pose.orientation.y = boat_state.orientation.y_val
        self.base.pose.orientation.z = boat_state.orientation.z_val
        self.base.pose.orientation.w = boat_state.orientation.w_val
    

        # Rover
        self.rover.header.stamp = time
        rover_state = self.client.getMultirotorState()
        self.rover.pose.position.x = rover_state.kinematics_estimated.position.x_val
        self.rover.pose.position.y = rover_state.kinematics_estimated.position.y_val
        self.rover.pose.position.z = rover_state.kinematics_estimated.position.z_val
        self.rover.pose.orientation.x = rover_state.kinematics_estimated.orientation.x_val
        self.rover.pose.orientation.y = rover_state.kinematics_estimated.orientation.y_val
        self.rover.pose.orientation.z = rover_state.kinematics_estimated.orientation.z_val
        self.rover.pose.orientation.w = rover_state.kinematics_estimated.orientation.w_val

        self.roverPub_.publish(self.rover)
        self.basePub_.publish(self.base)


if __name__ == '__main__':
    rospy.init_node('talker', anonymous=True)
    
    try:
        
        talker = AirsimPub()
        
    except rospy.ROSInterruptException:
        pass


