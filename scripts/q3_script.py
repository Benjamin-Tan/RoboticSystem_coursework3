#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import tf2_ros
#import math
from tf.transformations import quaternion_matrix
from sensor_msgs.msg import JointState
import numpy as np
from sympy import *

class q3_dynamic():

    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('cw3_q3')

        self.define_mass()

        # Robot contains the entire state of the robot (iiwa and shadow hand)
        robot = moveit_commander.RobotCommander()
        # We can get a list of all the groups in the robot
        print('============ Robot Groups:')
        print('{}\n\n'.format(robot.get_group_names()))

        self.iiwa_group = moveit_commander.MoveGroupCommander('object_iiwa')

        print('\n\nhand_iiwa Group Information')
        print('============ Reference frame: {}'.format(self.iiwa_group.get_planning_frame()))
        print('============ End effector: {}\n\n'.format(self.iiwa_group.get_end_effector_link()))

        self.tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # subscriber to joint state
        rospy.Subscriber("/joint_states", JointState, self.cb_jointState, queue_size=1)


        checkPos = self.iiwa_group.get_current_joint_values()
        print checkPos

        self.iiwa_group.set_joint_value_target([0.3,0.6,0.1,0.3,0.2,1.5,0])
        self.iiwa_group.go()
        rospy.sleep(3)
        #print self.current_joint_state


        self.find_mass_com_object()

        # link_name.append('object_iiwa_link_ee')
        #
        # T_01 = self.forward_kinematics(link_name[0],link_name[1])
        # T_12 = self.forward_kinematics(link_name[1],link_name[2])
        #
        # T_02 = self.forward_kinematics(link_name[0],link_name[2])
        #
        # R = quaternion_matrix([T_01.transform.rotation.x,T_01.transform.rotation.y,T_01.transform.rotation.z,T_01.transform.rotation.w])
        # R2 = quaternion_matrix([T_02.transform.rotation.x,T_02.transform.rotation.y,T_02.transform.rotation.z,T_02.transform.rotation.w])
        # R1 = quaternion_matrix([T_12.transform.rotation.x,T_12.transform.rotation.y,T_12.transform.rotation.z,T_12.transform.rotation.w])
        #
        # print R
        # print R1
        # print R2 - R.dot(R1)
        #
        # print T_01
        # print T_12




        # When finished shut down moveit_commander.
        moveit_commander.roscpp_shutdown()

    def define_mass(self):
        # gravity
        self.g_z = 9.81

        # mass of link 4,5,6
        self.m = np.array([4, 4, 3, 2.7, 1.7, 1.8, 0.3])

        self.object_z = Symbol('object_z')
        self.object_mass = Symbol('object_mass')


    def find_mass_com_object(self):
        link_name = []

        for i in range (0,8):
            link_name.append('object_iiwa_link_' + str(i))

        link_name.append('object_link_0') # box link


        J4c = self.get_jacobian_center(4,link_name)
        J5c = self.get_jacobian_center(5,link_name)
        J6c = self.get_jacobian_center(6,link_name)
        J7c = self.get_jacobian_center(7,link_name)
        J8c = self.get_jacobian_center(8,link_name)

        J_vc4 = J4c[2,3]
        J_vc5 = J5c[2,3]
        J_vc6 = J6c[2,3]
        J_vc7 = J7c[2,3]
        J_vc8 = J8c[2,3]

        tau_4 = self.g_z* ( self.m[3]*J_vc4 + self.m[4]*J_vc5 +self.m[5]*J_vc6 + self.m[6]*J_vc7 + self.object_mass*J_vc8 ) - self.current_joint_state.effort[3]
        tau_6 = self.g_z* ( self.m[5]*J6c[2,5] + self.m[6]*J7c[2,5] + self.object_mass*J8c[2,5] ) - self.current_joint_state.effort[5]

        sol = solve([tau_4,tau_6], [self.object_mass,self.object_z])
        print sol[0][0]
        print sol[0][1]



    def get_jacobian_center(self,to_link_no,link_name):


        # pose
        tf_01 = self.forward_kinematics(link_name[0], link_name[1])
        tf_02 = self.forward_kinematics(link_name[0], link_name[2])
        tf_03 = self.forward_kinematics(link_name[0], link_name[3])
        tf_04 = self.forward_kinematics(link_name[0], link_name[4])
        tf_05 = self.forward_kinematics(link_name[0], link_name[5])
        tf_06 = self.forward_kinematics(link_name[0], link_name[6])
        tf_07 = self.forward_kinematics(link_name[0], link_name[7])
        tf_08 = self.forward_kinematics(link_name[0], link_name[8])

        # transformation matrix
        T_01 = self.pose_to_matrix(tf_01)
        T_02 = self.pose_to_matrix(tf_02)
        T_03 = self.pose_to_matrix(tf_03)
        T_04 = self.pose_to_matrix(tf_04)
        T_05 = self.pose_to_matrix(tf_05)
        T_06 = self.pose_to_matrix(tf_06)
        T_07 = self.pose_to_matrix(tf_07)
        T_08 = self.pose_to_matrix(tf_08)

        # computer vector z and o
        z0 = np.array([[0,0,1]]) #maybe todo
        z1 = T_01[:3,2]
        z2 = T_02[:3,2]
        z3 = T_03[:3,2]
        z4 = T_04[:3,2]
        z5 = T_05[:3,2]
        z6 = T_06[:3,2]
        z7 = T_07[:3,2]

        o0 = np.array([[0,0,0]]) #todo
        o1 = T_01[:3,3]
        o2 = T_02[:3,3]
        o3 = T_03[:3,3]
        o4 = T_04[:3,3]
        o5 = T_05[:3,3]
        o6 = T_06[:3,3]
        o7 = T_07[:3,3]
        o8 = T_08[:3,3]

        # 7 joints + 1 box
        J = np.zeros((6,7))

        if to_link_no==1:
            o2 = o2 + np.dot(T_02[0:3,0:3],np.array([0,-0.03,0.12]))
            J[:3,0] = np.cross(z1,o2-o1)

        elif to_link_no==2:
            o3 = o3 + np.dot(T_03[0:3,0:3],np.array([0.0003,0.059,0.042]))
            J[:3,0] = np.cross(z1,o3-o1)
            J[:3,1] = np.cross(z2,o3-o2)

        elif to_link_no==3:
            o4 = o4 + np.dot(T_04[0:3,0:3],np.array([0,0.03,0.13]))
            J[:3,0] = np.cross(z1,o4-o1)
            J[:3,1] = np.cross(z2,o4-o2)
            J[:3,2] = np.cross(z3,o4-o3)

        elif to_link_no==4:
            o5 = o5 + np.dot(T_05[0:3,0:3],np.array([0,0.067,0.034]))
            J[:3,0] = np.cross(z1,o5-o1)
            J[:3,1] = np.cross(z2,o5-o2)
            J[:3,2] = np.cross(z3,o5-o3)
            J[:3,3] = np.cross(z4,o5-o4)

        elif to_link_no==5:
            o6 = o6 + np.dot(T_06[0:3,0:3],np.array([0.0001,0.021,0.076]))
            J[:3,0] = np.cross(z1,o6-o1)
            J[:3,1] = np.cross(z2,o6-o2)
            J[:3,2] = np.cross(z3,o6-o3)
            J[:3,3] = np.cross(z4,o6-o4)
            J[:3,4] = np.cross(z5,o6-o5)

        elif to_link_no==6:
            o7 = o7 + np.dot(T_07[0:3,0:3],np.array([0,0.0006,0.0004]))
            J[:3,0] = np.cross(z1,o7-o1)
            J[:3,1] = np.cross(z2,o7-o2)
            J[:3,2] = np.cross(z3,o7-o3)
            J[:3,3] = np.cross(z4,o7-o4)
            J[:3,4] = np.cross(z5,o7-o5)
            J[:3,5] = np.cross(z6,o7-o6)

        elif to_link_no==7:
            o8 = o8 + np.dot(T_08[0:3,0:3],np.array([0,0,0.02]))
            J[:3,0] = np.cross(z1,o8-o1)
            J[:3,1] = np.cross(z2,o8-o2)
            J[:3,2] = np.cross(z3,o8-o3)
            J[:3,3] = np.cross(z4,o8-o4)
            J[:3,4] = np.cross(z5,o8-o5)
            J[:3,5] = np.cross(z6,o8-o6)
            J[:3,6] = np.cross(z7,o8-o7)

        else:
            o8 = o8 + np.dot(T_08[0:3,0:3],np.array([0,0,self.object_z]))
            J[:3,0] = np.cross(z1,o8-o1)
            J[:3,1] = np.cross(z2,o8-o2)
            J[:3,2] = np.cross(z3,o8-o3)
            J[:3,3] = np.cross(z4,o8-o4)
            J[:3,4] = np.cross(z5,o8-o5)
            J[:3,5] = np.cross(z6,o8-o6)
            J[:3,6] = np.cross(z7,o8-o7)


        J[3:6,0] = z1
        J[3:6,1] = z2
        J[3:6,2] = z3
        J[3:6,3] = z4
        J[3:6,4] = z5
        J[3:6,5] = z6
        J[3:6,6] = z7

        # print J
        return J

    def pose_to_matrix(self,pose):
        # convert quaternion to rotation matrix
        transformation_mat = quaternion_matrix([pose.transform.rotation.x,pose.transform.rotation.y,pose.transform.rotation.z,pose.transform.rotation.w])

        # store the translation to the matrix
        transformation_mat[0,3] = pose.transform.translation.x
        transformation_mat[1,3] = pose.transform.translation.y
        transformation_mat[2,3] = pose.transform.translation.z

        return transformation_mat


    def forward_kinematics(self,current_frame,target_frame):
        while True:
            try:
                trans = self.tf_buffer.lookup_transform(current_frame,target_frame, rospy.Time())
                rospy.sleep(1)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue
            break
        return trans

    def cb_jointState(self,msg):
        self.current_joint_state = msg


if __name__ == '__main__':
    try:
        q3_dynamic()
    except rospy.ROSInterruptException:
        pass
