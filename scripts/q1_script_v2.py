#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from coursework3.srv import ChangeCollisionObject
import tf2_ros
import math
from tf.transformations import quaternion_from_euler,euler_from_quaternion,quaternion_matrix,quaternion_from_matrix
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes
from geometry_msgs.msg import Pose, PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class pick_and_place:
    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('cw3_q1')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Robot contains the entire state of the robot (iiwa and shadow hand)
        robot = moveit_commander.RobotCommander()
        # We can get a list of all the groups in the robot
        print('============ Robot Groups:')
        print('{}\n\n'.format(robot.get_group_names()))

        # Planning groups are used to control seperate aspects of the robot.
        # iiwa_group controls the iiwa arm from the base link to the end effector
        # hand group controls all of the joints of the hand.

        self.iiwa_group = moveit_commander.MoveGroupCommander('hand_iiwa')
        self.hand_group = moveit_commander.MoveGroupCommander('sr_hand')

        # print('\n\nhand_iiwa Group Information')
        # print('============ Reference frame: {}'.format(iiwa_group.get_planning_frame()))
        # print('============ End effector: {}\n\n'.format(iiwa_group.get_end_effector_link()))
        #
        # print('sr_hand Group Information')
        # print('============ Reference frame: {}'.format(hand_group.get_planning_frame()))
        # print('============ End effector: {}'.format(hand_group.get_end_effector_link()))

        rospy.sleep(1)

        # The hand has a very strict controller and will occasionally say it as failed even though it has moved to the right
        # position (just not in the right amount of time), ensure you check the achieved position.


        # Objects in the scene can be found using the object tracker node which publishes on the topic
        # '/recognized_objects_array' and to tf. The list of object names is found in the param '/object_list'

    ## ===================================   Object Tracking  =========================================================
        objects = rospy.get_param('object_list')
        print(objects)
        all_object_pose = []
        no_object = len(objects)

        for i in range(0,no_object):
            while True:
                try:
                    object_pose = self.tf_buffer.lookup_transform('world', objects[i], rospy.Time.now())
                    rospy.sleep(1)
                except (tf2_ros.ExtrapolationException,tf2_ros.LookupException):
                    continue
                break
            print('Found object {} [{}] at: \n{}'.format(i,objects[i],object_pose.transform))
            # store the object pose to new message
            all_object_pose.append(object_pose)

        iiwa_pose = self.tf_buffer.lookup_transform('world','hand_iiwa_link_0',rospy.Time.now())
        rospy.sleep(1)
        print('Found iiwa start Pos at: \n{}'.format(iiwa_pose.transform))
    ## ===================================== end object tracking =================================================
        # TF doesn't always return as transform even when the transform exists, try catching the execption, waiting a second
        # and looking up the transform again.

        # To grasp objects they must first be added to the allowed collision matrix so that the path planner knows to ignore
        #  the collision. You can do this using a service '/add_object_acm', they can also be removed from the acm after
        # grasping to prevent any unintended collisions.

        add_to_acm = rospy.ServiceProxy('/add_object_acm', ChangeCollisionObject)
        remove_from_acm = rospy.ServiceProxy('/remove_object_acm', ChangeCollisionObject)
        i = 1 # object no.
        success = add_to_acm(objects[i])
        print success

        ## open the hand
        # units in radian
        ff = [0,0,0,0]
        mf = [0,0,0,0]
        rf = [0,0,0,0]
        # open [-0.6,1.2,0   ,0]
        # grab [0.2 ,1.2,0.65,1]
        th = [-0.6,1.2,0,0]
        self.open_joint_values = ff+mf+rf+th
        self.hand_group.set_joint_value_target(self.open_joint_values)
        if self.hand_group.go() != True:
            rospy.logwarn("Go failed")
            # Retry
            print self.hand_group.go()

        checkPos = self.hand_group.get_current_joint_values()
        print checkPos[-4:]

        # move the iiwa just above object
        pose = self.iiwa_group.get_current_pose(self.iiwa_group.get_end_effector_link())

        obj_quat = (all_object_pose[i].transform.rotation.x,all_object_pose[i].transform.rotation.y,all_object_pose[i].transform.rotation.z,all_object_pose[i].transform.rotation.w)
        euler = euler_from_quaternion(obj_quat)
        new_quat = quaternion_from_euler(euler[0]+math.pi,euler[1],euler[2]+math.pi/2)
        # print new_quat

        pose.pose.position = all_object_pose[i].transform.translation
        pose.pose.position.x -= 0.00 # 0.02 for object 0 and 2, 0 for object 1
        pose.pose.position.y += 0.00 # 0.01 for object 0 and 2, 0 for object 1
        pose.pose.position.z += 0.35 # 0.3 for object 0 and 2, 0.35 for object 1

        pose.pose.orientation.x = new_quat[0]
        pose.pose.orientation.y = new_quat[1]
        pose.pose.orientation.z = new_quat[2]
        pose.pose.orientation.w = new_quat[3]

        self.iiwa_group.set_pose_target(pose)
        if self.iiwa_group.go != True:
            rospy.logwarn(" Arm failed")
            print self.iiwa_group.go()

        object_name = objects[i]+'__link_0'

        ff = [0,0.6,0.5,0.35]#[0,0.55,0.4,0.35]
        mf = [0,0.6,0.5,0.35]#[0,0.55,0.4,0.35]
        rf = [0,0.6,0.5,0.35]#[0,0.55,0.4,0.35]
        th = [0.22,1.20,0.65,1.1]
        self.closed_joint_values = ff+mf+rf+th


        target_pose_stamped = PoseStamped()
        target_pose_stamped.header.frame_id = 'world'
        target_pose_stamped.pose = pose.pose
        grasp_pose = target_pose_stamped
        grasps = self.make_grasps(grasp_pose, object_name)

        print grasps

        result = self.hand_group.pick(object_name,grasps)
        print result
        # # hand_group.set_named_target('open')
        # # hand_group.set_named_target('fingers_pack_thumb_open')
        #
        # # plan = hand_group.plan()
        #
        # # hand_group.set_named_target('open_grasp')
        # # hand_group.set_named_target('fingers_pack_thumb_open')
        #
        # ## grasping
        #
        # # move three fingers
        # # units in radian
        # ff = [0,0.6,0.5,0.35]#[0,0.55,0.4,0.35]
        # mf = [0,0.6,0.5,0.35]#[0,0.55,0.4,0.35]
        # rf = [0,0.6,0.5,0.35]#[0,0.55,0.4,0.35]
        # th = [0.22,1.20,0.65,1.1]
        # joint_values = ff+mf+rf+th
        # # hand_group.pick(object_name,joint_values)
        # plan = hand_group.plan(joint_values)
        # # print('Plan result: {}\n\n'.format(plan))
        # hand_group.execute(plan, wait=True)
        # # # rospy.sleep(10)
        # #
        # # checkPos = hand_group.get_current_joint_values()
        # # print checkPos[-4:]
        #
        # ## attach object
        # object_name = objects[i]+'__link_0'
        # hand_group.attach_object(object_name,'lh_palm')
        #
        # ## Lifting
        # pose = iiwa_group.get_current_pose(iiwa_group.get_end_effector_link())
        # pose.pose.position.z += 0.2
        #
        # # iiwa_group.clear_pose_targets()
        # print('iiwa planning to pose: {}'.format(pose.pose))
        # iiwa_group.set_pose_target(pose)
        # result = iiwa_group.plan()
        # # print('Plan result: {}'.format(result))
        # # rospy.sleep(5)
        # iiwa_group.execute(result, wait=True)
        # rospy.sleep(10)
        #
        # # Put it back to original position
        # pose = iiwa_group.get_current_pose(iiwa_group.get_end_effector_link())
        # pose.pose.position.z -= 0.2
        #
        # # iiwa_group.clear_pose_targets()
        # print('iiwa planning to pose: {}'.format(pose.pose))
        # iiwa_group.set_pose_target(pose)
        # result = iiwa_group.plan()
        # # # print('Plan result: {}'.format(result))
        # # # rospy.sleep(5)
        # iiwa_group.execute(result, wait=True)


        # # check the object position
        # for i in range(0,no_object):
        #     while True:
        #         try:
        #             object_pose = tf_buffer.lookup_transform('world', objects[i], rospy.Time.now())
        #             rospy.sleep(1)
        #         except (tf2_ros.ExtrapolationException,tf2_ros.LookupException):
        #             continue
        #         break
        #     print('Found object {} [{}] at: \n{}'.format(i,objects[i],object_pose.transform))
        #     # store the object pose to new message
        #     all_object_pose.append(object_pose)

        success = remove_from_acm(objects[i])

        moveit_commander.roscpp_shutdown()

    def make_grasps(self, initial_pose, allow_touch_objects):
        # initialise a grasp object
        g = Grasp()

        g.pre_grasp_posture = self.make_grab_posture(self.open_joint_values)
        g.grasp_posture = self.make_grab_posture(self.closed_joint_values)

        g.pre_grasp_approach = self.make_grab_translation(0.01, 0.1, [0.0, 0.0, 1.0])
        g.post_grasp_retreat = self.make_grab_translation(0.1, 0.15, [0.0, 0.0, 1.0])

        g.grasp_pose = initial_pose

        # pitch_vals = [0, 0.1, -0.1, 0.2, -0.2, 0.4, -0.4]
        #
        # target_pose_arm_ref = self.tf_buffer.transform(initial_pose,'hand_iiwa_link_0')
        # x = target_pose_arm_ref.pose.position.x
        # y = target_pose_arm_ref.pose.position.y
        # yaw_vals = [math.atan2(y,x) + inc for inc in [0, 0.1, -0.1]]
        #
        grasps = []
        #
        # for yaw in yaw_vals:
        #     for pitch in pitch_vals:
        #         q = quaternion_from_euler(0,pitch,yaw)
        #
        #         g.grasp_pose.pose.orientation.x = q[0]
        #         g.grasp_pose.pose.orientation.y = q[1]
        #         g.grasp_pose.pose.orientation.z = q[2]
        #         g.grasp_pose.pose.orientation.w = q[3]

        g.id = str(len(grasps))

        g.allowed_touch_objects = allow_touch_objects
        g.max_contact_force = 0
        g.grasp_quality = 1.0 #- abs(pitch)

        grasps.append(g)

        return grasps


    def make_grab_posture(self, joint_positions):
        t = JointTrajectory()

        t.header.stamp = rospy.get_rostime()
        t.joint_names = self.hand_group.get_joints()

        tp= JointTrajectoryPoint()

        tp.positions = joint_positions
        tp.effort = [1.0]
        tp.time_from_start = rospy.Duration(0.0)

        t.points.append(tp)
        return t

    def make_grab_translation(self, min_dist, desired, vector):
        g = GripperTranslation()

        g.direction.vector.x = vector[0]
        g.direction.vector.y = vector[1]
        g.direction.vector.z = vector[2]

        g.direction.header.frame_id = 'lh_palm'
        g.min_distance = min_dist
        g.desired_distance = desired
        return g

if __name__ == '__main__':
    try:
        pick_and_place()
    except rospy.ROSInterruptException:
        pass
