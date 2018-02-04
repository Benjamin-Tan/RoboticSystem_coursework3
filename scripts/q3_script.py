#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import tf2_ros
import math
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

        self.iiwa_group.set_joint_value_target([0.1,0.2,0.3,0.3,0.2,0.2,0.5])
        self.iiwa_group.go()
        rospy.sleep(3)
        #print self.current_joint_state

        # self.forward_kinematics_sym(2)
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

        # mass and inertia tensor at centre of mass of each link
        self.m = np.array([4, 4, 3, 2.7, 1.7, 1.8, 0.3])
        self.Icm = np.array([[0.1, 0.09, 0.02], [0.05, 0.018, 0.044], [0.08, 0.075, 0.01],
                             [0.03, 0.01, 0.029], [0.02, 0.018, 0.005], [0.005, 0.0036, 0.0047],
                             [0.001, 0.001, 0.001]])

        # symbols for center of mass (z axis) and object mass
        self.object_z = Symbol('object_z')
        self.object_mass = Symbol('object_mass')

        self.com = np.array([[0,-0.03,0.12], [0.0003,0.059,0.042], [0,0.03,0.13], [0,0.067,0.034],
                             [0.0001,0.021,0.076], [0,0.0006,0.0004], [0,0,0.02]])
        self.object_com = np.array([0,0,self.object_z])


        # dh parameters => theta, d, alpha, a
        # 3rd, 5th , 7th link requires additional translation of 0.2045, 0.1845,and 0.081 respectively.
        self.dh_param = np.array([[      0, 0.1575,         0, 0],
                                  [math.pi, 0.2025, math.pi/2, 0],
                                  [math.pi,      0, math.pi/2, 0],
                                  [      0, 0.2155, math.pi/2, 0],
                                  [math.pi,      0, math.pi/2, 0],
                                  [      0, 0.2155, math.pi/2, 0],
                                  [math.pi,      0, math.pi/2, 0]])

        # symbols for theta
        self.theta1 = Symbol('theta1')
        self.theta2 = Symbol('theta2')
        self.theta3 = Symbol('theta3')
        self.theta4 = Symbol('theta4')
        self.theta5 = Symbol('theta5')
        self.theta6 = Symbol('theta6')
        self.theta7 = Symbol('theta7')


    def forward_kinematics_sym(self, to_link_no):
        additional_translation_3 = np.array([[1,0,0,0],
                                             [0,1,0,0],
                                             [0,0,1,0.2045],
                                             [0,0,0,1]])

        additional_translation_5 = np.array([[1,0,0,0],
                                             [0,1,0,0],
                                             [0,0,1,0.1845],
                                             [0,0,0,1]])

        additional_translation_7 = np.array([[1,0,0,0],
                                             [0,1,0,0],
                                             [0,0,1,0.081],
                                             [0,0,0,1]])

        self.theta1 += self.dh_param[0][0]
        self.theta2 += self.dh_param[1][0]
        self.theta3 += self.dh_param[2][0]
        self.theta4 += self.dh_param[3][0]
        self.theta5 += self.dh_param[4][0]
        self.theta6 += self.dh_param[5][0]
        self.theta7 += self.dh_param[6][0]


        T_01 = np.array([[cos(self.theta1), -sin(self.theta1)*cos(self.dh_param[0][2]), sin(self.theta1)*sin(self.dh_param[0][2]), 0],
                         [sin(self.theta1),  cos(self.theta1)*cos(self.dh_param[0][2]),-cos(self.theta1)*sin(self.dh_param[0][2]), 0],
                         [               0,                   sin(self.dh_param[0][2]),                  cos(self.dh_param[0][2]), self.dh_param[0][1]],
                         [               0,                                          0,                                         0, 1]])

        T_12 = np.array([[cos(self.theta2), -sin(self.theta2)*cos(self.dh_param[1][2]), sin(self.theta2)*sin(self.dh_param[1][2]), 0],
                         [sin(self.theta2),  cos(self.theta2)*cos(self.dh_param[1][2]),-cos(self.theta2)*sin(self.dh_param[1][2]), 0],
                         [               0,                   sin(self.dh_param[1][2]),                  cos(self.dh_param[1][2]), self.dh_param[1][1]],
                         [               0,                                          0,                                         0, 1]])

        T_23 = np.array([[cos(self.theta3), -sin(self.theta3)*cos(self.dh_param[2][2]), sin(self.theta3)*sin(self.dh_param[2][2]), 0],
                         [sin(self.theta3),  cos(self.theta3)*cos(self.dh_param[2][2]),-cos(self.theta3)*sin(self.dh_param[2][2]), 0],
                         [               0,                   sin(self.dh_param[2][2]),                  cos(self.dh_param[2][2]), self.dh_param[2][1]],
                         [               0,                                          0,                                         0, 1]])

        # print 't23\n',T_23
        T_23 = np.dot(T_23,additional_translation_3)
        # print type(T_23)
        # print 'new\n',T_23
        T_34 = np.array([[cos(self.theta4), -sin(self.theta4)*cos(self.dh_param[3][2]), sin(self.theta4)*sin(self.dh_param[3][2]), 0],
                         [sin(self.theta4),  cos(self.theta4)*cos(self.dh_param[3][2]),-cos(self.theta4)*sin(self.dh_param[3][2]), 0],
                         [               0,                   sin(self.dh_param[3][2]),                  cos(self.dh_param[3][2]), self.dh_param[3][1]],
                         [               0,                                          0,                                         0, 1]])

        T_45 = np.array([[cos(self.theta5), -sin(self.theta5)*cos(self.dh_param[4][2]), sin(self.theta5)*sin(self.dh_param[4][2]), 0],
                         [sin(self.theta5),  cos(self.theta5)*cos(self.dh_param[4][2]),-cos(self.theta5)*sin(self.dh_param[4][2]), 0],
                         [               0,                   sin(self.dh_param[4][2]),                  cos(self.dh_param[4][2]), self.dh_param[4][1]],
                         [               0,                                          0,                                         0, 1]])

        T_45 = np.dot(T_45,additional_translation_5)

        T_56 = np.array([[cos(self.theta6), -sin(self.theta6)*cos(self.dh_param[5][2]), sin(self.theta6)*sin(self.dh_param[5][2]), 0],
                         [sin(self.theta6),  cos(self.theta6)*cos(self.dh_param[5][2]),-cos(self.theta6)*sin(self.dh_param[5][2]), 0],
                         [               0,                   sin(self.dh_param[5][2]),                  cos(self.dh_param[5][2]), self.dh_param[5][1]],
                         [               0,                                          0,                                         0, 1]])

        T_67 = np.array([[cos(self.theta7), -sin(self.theta7)*cos(self.dh_param[6][2]), sin(self.theta7)*sin(self.dh_param[6][2]), 0],
                         [sin(self.theta7),  cos(self.theta7)*cos(self.dh_param[6][2]),-cos(self.theta7)*sin(self.dh_param[6][2]), 0],
                         [               0,                   sin(self.dh_param[6][2]),                  cos(self.dh_param[6][2]), self.dh_param[6][1]],
                         [               0,                                          0,                                         0, 1]])

        T_67 = np.dot(T_67,additional_translation_7)

        if to_link_no == 1:
            T = T_01
        elif to_link_no == 2:
            T = np.mat(T_01) * np.mat(T_12)
        elif to_link_no == 3:
            T = np.mat(T_01) * np.mat(T_12) * np.mat(T_23)
        elif to_link_no == 4:
            T = np.mat(T_01) * np.mat(T_12) * np.mat(T_23) * np.mat(T_34)
        elif to_link_no == 5:
            T = np.mat(T_01) * np.mat(T_12) * np.mat(T_23) * np.mat(T_34) * np.mat(T_45)
        elif to_link_no == 6:
            T = np.mat(T_01) * np.mat(T_12) * np.mat(T_23) * np.mat(T_34) * np.mat(T_45) * np.mat(T_56)
        else:
            T = np.mat(T_01) * np.mat(T_12) * np.mat(T_23) * np.mat(T_34) * np.mat(T_45) * np.mat(T_56) * np.mat(T_67)

        T = np.asarray(T)


        return T



########## Numerical methods to find mass and CoM ############
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

        self.object_mass = sol[0][0]
        #self.object_z = sol[0][1]
        print 'mass...com '
        print self.object_mass,sol[0][1]

        tau_4 = self.g_z* ( self.m[3]*J_vc4 + self.m[4]*J_vc5 +self.m[5]*J_vc6 + self.m[6]*J_vc7 + self.object_mass*J_vc8 ) - self.current_joint_state.effort[3]
        tau_6 = self.g_z* ( self.m[5]*J6c[2,5] + self.m[6]*J7c[2,5] + self.object_mass*J8c[2,5] ) - self.current_joint_state.effort[5]
        print 'tau4...tau6'
        print tau_4.subs(self.object_z,sol[0][1]),tau_6.subs(self.object_z,sol[0][1])


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
        z0 = T_01[:3,2]
        z1 = T_02[:3,2]
        z2 = T_03[:3,2]
        z3 = T_04[:3,2]
        z4 = T_05[:3,2]
        z5 = T_06[:3,2]
        z6 = T_07[:3,2]

        o0 = T_01[:3,3]
        o1 = T_02[:3,3]
        o2 = T_03[:3,3]
        o3 = T_04[:3,3]
        o4 = T_05[:3,3]
        o5 = T_06[:3,3]
        o6 = T_07[:3,3]
        o7 = T_08[:3,3]

        self.R_01 = T_01[0:3,0:3]
        self.R_02 = T_02[0:3,0:3]
        self.R_03 = T_03[0:3,0:3]
        self.R_04 = T_04[0:3,0:3]
        self.R_05 = T_05[0:3,0:3]
        self.R_06 = T_06[0:3,0:3]
        self.R_07 = T_07[0:3,0:3]

        # 7 joints + 1 box
        if to_link_no != 8:
            J = np.zeros((3,7))

        if to_link_no==1:
            o1 = o0 + np.dot(self.R_01,self.com[0])
            J[:3,0] = np.cross(z0,o1-o0)

        elif to_link_no==2:
            o2 = o1 + np.dot(self.R_02,self.com[1])
            J[:3,0] = np.cross(z0,o2-o0)
            J[:3,1] = np.cross(z1,o2-o1)

        elif to_link_no==3:
            o3 = o2 + np.dot(self.R_03,self.com[2])
            J[:3,0] = np.cross(z0,o3-o0)
            J[:3,1] = np.cross(z1,o3-o1)
            J[:3,2] = np.cross(z2,o3-o2)

        elif to_link_no==4:
            o4 = o3 + np.dot(self.R_04,self.com[3])
            J[:3,0] = np.cross(z0,o4-o0)
            J[:3,1] = np.cross(z1,o4-o1)
            J[:3,2] = np.cross(z2,o4-o2)
            J[:3,3] = np.cross(z3,o4-o3)

        elif to_link_no==5:
            o5 = o4 + np.dot(self.R_05,self.com[4])
            J[:3,0] = np.cross(z0,o5-o0)
            J[:3,1] = np.cross(z1,o5-o1)
            J[:3,2] = np.cross(z2,o5-o2)
            J[:3,3] = np.cross(z3,o5-o3)
            J[:3,4] = np.cross(z4,o5-o4)

        elif to_link_no==6:
            o6 = o5 + np.dot(self.R_06,self.com[5])
            J[:3,0] = np.cross(z0,o6-o0)
            J[:3,1] = np.cross(z1,o6-o1)
            J[:3,2] = np.cross(z2,o6-o2)
            J[:3,3] = np.cross(z3,o6-o3)
            J[:3,4] = np.cross(z4,o6-o4)
            J[:3,5] = np.cross(z5,o6-o5)

        elif to_link_no==7:
            o7 = o6 + np.dot(self.R_07,self.com[6])
            J[:3,0] = np.cross(z0,o7-o0)
            J[:3,1] = np.cross(z1,o7-o1)
            J[:3,2] = np.cross(z2,o7-o2)
            J[:3,3] = np.cross(z3,o7-o3)
            J[:3,4] = np.cross(z4,o7-o4)
            J[:3,5] = np.cross(z5,o7-o5)
            J[:3,6] = np.cross(z6,o7-o6)

        else:
            o8 = o7 + np.dot(self.R_07,self.object_com)
            J1 = np.cross(z0,o8-o0)
            J2 = np.cross(z1,o8-o1)
            J3 = np.cross(z2,o8-o2)
            J4 = np.cross(z3,o8-o3)
            J5 = np.cross(z4,o8-o4)
            J6 = np.cross(z5,o8-o5)
            J7 = np.cross(z6,o8-o6)

            comb0 = [J1[0], J2[0], J3[0], J4[0], J5[0], J6[0], J7[0]]
            comb1 = [J1[1], J2[1], J3[1], J4[1], J5[1], J6[1], J7[1]]
            comb2 = [J1[2], J2[2], J3[2], J4[2], J5[2], J6[2], J7[2]]

            J = np.matrix([comb0, comb1, comb2])

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
