#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from coursework3.srv import ChangeCollisionObject
import tf2_ros
import math
from tf.transformations import quaternion_from_euler,euler_from_quaternion
from gazebo_msgs.msg import ContactsState, ContactState
import numpy as np

class pick_and_place:
    def __init__(self):
        # rospy.sleep(10) # wait for other files to be launched successfully

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('cw3_q2')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # self.touch = [0,0,0,0]

        # rospy.Subscriber("/contacts/lh_ff/distal", ContactsState, self.call_ff)
        # rospy.Subscriber("/contacts/lh_mf/distal", ContactsState, self.call_mf)
        # rospy.Subscriber("/contacts/lh_rf/distal", ContactsState, self.call_rf)
        # rospy.Subscriber("/contacts/lh_th/distal", ContactsState, self.call_th)
        # # Robot contains the entire state of the robot (iiwa and shadow hand)
        # robot = moveit_commander.RobotCommander()
        # # We can get a list of all the groups in the robot
        # print('============ Robot Groups:')
        # print('{}\n\n'.format(robot.get_group_names()))

        self.iiwa_group = moveit_commander.MoveGroupCommander('hand_iiwa')
        self.hand_group = moveit_commander.MoveGroupCommander('sr_hand')

        rospy.sleep(1)

        N = 10 # amount of tracking to average

        combine_obj_pos = np.empty((N,9))
        combine_euler = np.empty((N,9))

        # object tracking
        for cN in range(0,N):
            [all_object_name,all_object_pose] = self.get_object_position()

            # loop through each object and store their respective position and orientation
            for m in range(0,3):
                combine_obj_pos[cN,3*m:3*m+3] = (all_object_pose[m].transform.translation.x,all_object_pose[m].transform.translation.y,all_object_pose[m].transform.translation.z)
                obj_quat = (all_object_pose[m].transform.rotation.x,all_object_pose[m].transform.rotation.y,all_object_pose[m].transform.rotation.z,all_object_pose[m].transform.rotation.w)
                combine_euler[cN,3*m:3*m+3] = euler_from_quaternion(obj_quat)

        print combine_obj_pos

        average_obj_pos = np.mean(combine_obj_pos,axis=0)
        average_euler = np.mean(combine_euler,axis=0)

        # update the object pose with the new averaged value
        for p in range(0,3):
            all_object_pose[p].transform.translation.x = average_obj_pos[3*p]
            all_object_pose[p].transform.translation.y = average_obj_pos[3*p+1]
            all_object_pose[p].transform.translation.z = average_obj_pos[3*p+2]

            new_quat = quaternion_from_euler(average_euler[3*p], average_euler[3*p+1], average_euler[3*p+2])
            all_object_pose[p].transform.rotation.x = new_quat[0]
            all_object_pose[p].transform.rotation.y = new_quat[1]
            all_object_pose[p].transform.rotation.z = new_quat[2]
            all_object_pose[p].transform.rotation.w = new_quat[3]

        print all_object_pose

        # print average_obj_pos
        # create a service proxy for add and remove objects to allowed collision matrix
        add_to_acm = rospy.ServiceProxy('/add_object_acm', ChangeCollisionObject)
        remove_from_acm = rospy.ServiceProxy('/remove_object_acm', ChangeCollisionObject)

        # open the hand
        self.hand_open()

        for i in range(0,3):
            # i = 2 # object no.

            # move the iiwa to just above the object
            self.move_iiwa_to_object(all_object_pose,i)

            # add object to allowed collision matrix
            add_to_acm(all_object_name[i])

            # close the hand (grasping)
            self.hand_close()

            # hold the grasping position for 5 seconds
            rospy.sleep(5)

            # open the hand (releasing)
            self.hand_open()

            # remove object from allowed collision matrix
            remove_from_acm(all_object_name[i])

        moveit_commander.roscpp_shutdown()
        rospy.loginfo("Task finished")

    ## object tracking method
    def get_object_position(self):
        objects = rospy.get_param('object_list')
        print(objects)
        all_object_pose = []
        no_object = len(objects)

        for i in range(0,no_object):
            while True:
                try:
                    object_pose = self.tf_buffer.lookup_transform('world', objects[i], rospy.Time.now())
                    rospy.sleep(1)
                except (tf2_ros.ExtrapolationException,tf2_ros.LookupException,tf2_ros.ConnectivityException):
                    continue
                break
            print('Found object {} [{}] at: \n{}'.format(i,objects[i],object_pose.transform))
            # store the object pose to new message
            all_object_pose.append(object_pose)

        return objects,all_object_pose

    ## open the hand method
    def hand_open(self):
        # units in radian
        ff = [0,0,0,0]
        mf = [0,0,0,0]
        rf = [0,0,0,0]
        th = [-0.6,1.2,0,0]
        self.open_joint_values = ff+mf+rf+th
        count = 0
        rospy.loginfo("Hand opening")
        result = self.hand_group.plan(self.open_joint_values)
        print self.hand_group.execute(result, wait=True)
        while self.hand_group.execute(result, wait=True) != True and count<3:
            count += 1
            rospy.logwarn("Hand opening failed and retry attempt {}".format(count))
            result = self.hand_group.plan(self.open_joint_values)
            rospy.sleep(1)


        rospy.loginfo("Hand opening SUCCEEDED")

    ## move iiwa to above object method
    def move_iiwa_to_object(self,all_object_pose,i):
        # move the iiwa just above object
        pose = self.iiwa_group.get_current_pose(self.iiwa_group.get_end_effector_link())

        obj_quat = (all_object_pose[i].transform.rotation.x,all_object_pose[i].transform.rotation.y,all_object_pose[i].transform.rotation.z,all_object_pose[i].transform.rotation.w)
        euler = euler_from_quaternion(obj_quat)
        new_quat = quaternion_from_euler(euler[0]+math.pi,euler[1],euler[2]+math.pi/2)
        # print new_quat

        pose.pose.position = all_object_pose[i].transform.translation
        if i==1:
            vector_pose = [0.0, 0.0, 0.35]
        elif i==2:
            vector_pose = [0.01, 0.03, 0.31]
        else:
            vector_pose = [0.01, 0.03, 0.30]

        pose.pose.position.x -= vector_pose[0] # 0.01 for object 0 and 2, 0 for object 1
        pose.pose.position.y += vector_pose[1] # 0.03 for object 0 and 2, 0 for object 1
        pose.pose.position.z += vector_pose[2] # 0.3 for object 0 and 2, 0.35 for object 1

        pose.pose.orientation.x = new_quat[0]
        pose.pose.orientation.y = new_quat[1]
        pose.pose.orientation.z = new_quat[2]
        pose.pose.orientation.w = new_quat[3]

        self.iiwa_group.clear_pose_targets()
        self.iiwa_group.set_pose_target(pose)
        count = 0
        rospy.loginfo("IIWA moving")
        result = self.iiwa_group.plan()

        if result.joint_trajectory.joint_names == []:
            # planning fail, re-plan
            euler = euler_from_quaternion(new_quat)
            new_quat = quaternion_from_euler(euler[0],euler[1],euler[2]+math.pi/4) #rotate 45 degrees
            pose.pose.position.x = pose.pose.position.x + vector_pose[0]
            pose.pose.position.y = pose.pose.position.y - vector_pose[1]

            pose.pose.orientation.x = new_quat[0]
            pose.pose.orientation.y = new_quat[1]
            pose.pose.orientation.z = new_quat[2]
            pose.pose.orientation.w = new_quat[3]

            self.iiwa_group.clear_pose_targets()
            self.iiwa_group.set_pose_target(pose)
            rospy.loginfo("Re-planning")
            result = self.iiwa_group.plan()
            # print result
            if result.joint_trajectory.joint_names == []:
                # planning fail, re-plan
                euler = euler_from_quaternion(new_quat)
                new_quat = quaternion_from_euler(euler[0],euler[1],euler[2]-math.pi/2) #rotate -45 degrees

                pose.pose.orientation.x = new_quat[0]
                pose.pose.orientation.y = new_quat[1]
                pose.pose.orientation.z = new_quat[2]
                pose.pose.orientation.w = new_quat[3]

                self.iiwa_group.clear_pose_targets()
                self.iiwa_group.set_pose_target(pose)
                rospy.loginfo("Re-planning 2")
                result = self.iiwa_group.plan()

        self.iiwa_group.execute(result, wait=True)

        while self.iiwa_group.execute(result, wait=True) != True:
            count += 1
            rospy.logwarn("IIWA moving failed and retry attempt {}".format(count))
            result = self.iiwa_group.plan()
            rospy.sleep(1)

        rospy.loginfo("IIWA moving SUCCEEDED")

    ## close the hand method
    def hand_close(self):
        # units in radian
        ff = [0,0.6,0.5,0.35]#[0,0.55,0.4,0.35]
        mf = [0,0.6,0.5,0.35]#[0,0.55,0.4,0.35]
        rf = [0,0.6,0.5,0.35]#[0,0.55,0.4,0.35]
        th = [0.22,1.20,0.65,1.1]
        self.closed_joint_values = ff+mf+rf+th
        count = 0
        rospy.loginfo("Hand closing")
        result = self.hand_group.plan(self.closed_joint_values)
        print self.hand_group.execute(result, wait=True)
        # while self.hand_group.execute(result, wait=True) != True:
        #     count += 1
        #     rospy.logwarn("Hand closing failed and retry attempt {}".format(count))
        #     result = self.hand_group.plan(self.closed_joint_values)
        #     rospy.sleep(5)

        # rospy.loginfo(self.touch)
        rospy.loginfo("Hand closing SUCCEEDED")

    ## callback function (ff)
    # def call_ff(self,msg):
    #     print msg.states
        #     self.touch[0] = 1
        # else:
        #     self.touch[0] = 0
        # print self.touch[0]
    # ## callback function (mf)
    # def call_mf(self,msg):
    #     self.touch[1] = len(msg.states)
    #
    # ## callback function (rf)
    # def call_rf(self,msg):
    #     self.touch[2] = len(msg.states)
    # ## callback function (th)
    # def call_th(self,msg):
    #     self.touch[3] = len(msg.states)

if __name__ == '__main__':
    try:
        pick_and_place()
    except rospy.ROSInterruptException:
        pass
