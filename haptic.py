import socket
from H3DInterface import *
from H3DUtils import *
import random
import time as python_time
import datetime
import pickle
import rospy
from std_msgs.msg import Float64MultiArray, Float64, Int8 
from geometry_msgs.msg import PoseStamped, Vector3
from sensor_msgs.msg import JointState
import math
random.seed(python_time.time())

x, = references.getValue()  # Group node on x3d

class DeviceChangeField(AutoUpdate(TypedField(SFBool, (SFBool, SFBool, SFVec3f, SFRotation, SFVec3f,SFVec3f)))):
    def __init__(self):
        AutoUpdate(TypedField(SFBool, (SFBool, SFBool, SFVec3f, SFRotation, SFVec3f,SFVec3f))
                   ).__init__(self)

        self.node, self.dn = createX3DNodeFromString("""\
            <ForceField DEF="FORCE"/>""")
        x.children.push_back(
            self.node)  # make the node visible by adding it to the parent
        self.force = Vec3f(0, 0, 0)

    def callback(self,data):
        # rospy.loginfo(str(data.x)+" "+str(data.y)+" "+str(data.z))
        try:
            self.force=Vec3f(data.x, data.y, data.z)
        except:
            pass

    def update(self, event):
        """
        Update states changes of haptic stylus's position, orientation and 2 buttons.
        Send latest data to robot arm using TCP socket.
        Receive force feedback from robot arm.
        Generate force feedback.
        """
        # Read stylus states
        button1, button2, pos, ori,joint,gimbal = self.getRoutesIn()
        
        rospy.init_node('omni_haptic_node', anonymous=True)
        
        # Publish /phantom/pose
        pose = PoseStamped()
        pose.pose.position.x = pos.getValue().x
        pose.pose.position.y = -pos.getValue().z
        pose.pose.position.z = pos.getValue().y
        pose.pose.orientation.x = -ori.getValue().y
        pose.pose.orientation.y = ori.getValue().x
        pose.pose.orientation.z = ori.getValue().z
        pose.pose.orientation.w = ori.getValue().angle
        pub = rospy.Publisher('/phantom/pose', PoseStamped, queue_size=1)
        pub.publish(pose)

        # Publish button_diff
        btn_diff = Int8()
        btn_diff = int(button1.getValue())-int(button2.getValue())
        pub = rospy.Publisher('/phantom/button_diff', Int8, queue_size=1)
        pub.publish(btn_diff)

        # Publish /phantom/joint_states
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ["waist","shoulder","elbow","yaw","pitch","roll"]
        joint_state.position = [0]*6
        joint_state.position[0] = -joint.getValue().x
        joint_state.position[1] = joint.getValue().y
        joint_state.position[2] = joint.getValue().z - joint.getValue().y
        joint_state.position[3] = -gimbal.getValue().x + math.pi;
        joint_state.position[4] = - gimbal.getValue().y - 3*math.pi/4;
        joint_state.position[5] = - gimbal.getValue().z - math.pi;
        pub = rospy.Publisher('/phantom/joint_states', JointState, queue_size=1)        
        pub.publish(joint_state)

        # Subscribe force feedback
        rospy.Subscriber("/phantom/force_feedback", Vector3, self.callback, queue_size = 1, buff_size = 1024)
        self.dn["FORCE"].force.setValue(self.force)
        # rospy.spin()
        return True


device = getHapticsDevice(0)
if not device:
    di = createX3DNodeFromString(
        """<DeviceInfo><AnyDevice/></DeviceInfo>""")[0]
    device = getHapticsDevice(0)

position_change = DeviceChangeField()
if device:
    device.mainButton.routeNoEvent(position_change)
    device.secondaryButton.routeNoEvent(position_change)
    device.devicePosition.routeNoEvent(position_change)
    device.deviceOrientation.routeNoEvent(position_change)
    device.jointAngles.routeNoEvent(position_change)
    device.gimbalAngles.routeNoEvent(position_change)
    
