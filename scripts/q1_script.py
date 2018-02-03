#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from coursework3.srv import ChangeCollisionObject
import tf2_ros
import math,tf,moveit_msgs.msg

def q1_script():

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('cw3_q1')

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Robot contains the entire state of the robot (iiwa and shadow hand)
    robot = moveit_commander.RobotCommander()
    # We can get a list of all the groups in the robot
    print('============ Robot Groups:')
    print('{}\n\n'.format(robot.get_group_names()))

    # Planning groups are used to control seperate aspects of the robot.
    # iiwa_group controls the iiwa arm from the base link to the end effector
    # hand group controls all of the joints of the hand.

    iiwa_group = moveit_commander.MoveGroupCommander('hand_iiwa')
    hand_group = moveit_commander.MoveGroupCommander('sr_hand')

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

## ===================================   new  =========================================================
    objects = rospy.get_param('object_list')
    print(objects)
    all_object_pose = []
    no_object = len(objects)

    for i in range(0,no_object):
        while True:
            try:
                object_pose = tf_buffer.lookup_transform('world', objects[i], rospy.Time.now())
                rospy.sleep(1)
            except (tf2_ros.ExtrapolationException,tf2_ros.LookupException):
                continue
            break
        print('Found object {} [{}] at: \n{}'.format(i,objects[i],object_pose.transform))
        # store the object pose to new message
        all_object_pose.append(object_pose)

    iiwa_pose = tf_buffer.lookup_transform('world','hand_iiwa_link_0',rospy.Time.now())
    rospy.sleep(1)
    print('Found iiwa start Pos at: \n{}'.format(iiwa_pose.transform))
## ===================================== end new =================================================
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

    ## open
    # units in radian
    ff = [0,0,0,0]
    mf = [0,0,0,0]
    rf = [0,0,0,0]
    # open [-0.6,1.2,0   ,0]
    # grab [0.2 ,1.2,0.65,1]
    th = [-0.6,1.2,0,0]
    joint_values = ff+mf+rf+th
    plan = hand_group.plan(joint_values)
    # print('Plan result: {}\n\n'.format(plan))
    hand_group.execute(plan, wait=True)
    rospy.sleep(5)

    checkPos = hand_group.get_current_joint_values()
    print checkPos[-4:]

    # Now you can plan a motion to grasp the object
    pose = iiwa_group.get_current_pose(iiwa_group.get_end_effector_link())

    obj_quat = (all_object_pose[i].transform.rotation.x,all_object_pose[i].transform.rotation.y,all_object_pose[i].transform.rotation.z,all_object_pose[i].transform.rotation.w)
    euler = tf.transformations.euler_from_quaternion(obj_quat)
    new_quat = tf.transformations.quaternion_from_euler(euler[0]+math.pi,euler[1],euler[2]+math.pi/2)
    print new_quat

    pose.pose.position = all_object_pose[i].transform.translation
    pose.pose.position.x -= 0.00 # 0.02 for object 0 and 2, 0 for object 1
    pose.pose.position.y += 0.00 # 0.01 for object 0 and 2, 0 for object 1
    pose.pose.position.z += 0.35 # 0.3 for object 0 and 2, 0.35 for object 1

    pose.pose.orientation.x = new_quat[0]
    pose.pose.orientation.y = new_quat[1]
    pose.pose.orientation.z = new_quat[2]
    pose.pose.orientation.w = new_quat[3]

    print('iiwa planning to pose: {}'.format(pose.pose))
    iiwa_group.set_pose_target(pose)
    result = iiwa_group.plan()
    # print('Plan result: {}'.format(result))
    # rospy.sleep(5)
    iiwa_group.execute(result, wait=True)
    # rospy.sleep(10)
    final_pose = iiwa_group.get_current_pose(iiwa_group.get_end_effector_link())
    print final_pose

    # hand_group.set_named_target('open')
    # hand_group.set_named_target('fingers_pack_thumb_open')

    # plan = hand_group.plan()

    # hand_group.set_named_target('open_grasp')
    # hand_group.set_named_target('fingers_pack_thumb_open')

    ## grasping

    # move three fingers
    object_name = objects[i]+'__link_0'
    # units in radian
    ff = [0,0.6,0.5,0.35]#[0,0.55,0.4,0.35]
    mf = [0,0.6,0.5,0.35]#[0,0.55,0.4,0.35]
    rf = [0,0.6,0.5,0.35]#[0,0.55,0.4,0.35]
    th = [0.22,1.20,0.65,1.1]
    joint_values = ff+mf+rf+th
    # hand_group.pick(object_name,joint_values)
    plan = hand_group.plan(joint_values)
    # print('Plan result: {}\n\n'.format(plan))
    hand_group.execute(plan, wait=True)
    # # rospy.sleep(10)
    #
    # checkPos = hand_group.get_current_joint_values()
    # print checkPos[-4:]

    ## attach object
    hand_group.attach_object(object_name,'lh_palm')

    ## Lifting
    pose = iiwa_group.get_current_pose(iiwa_group.get_end_effector_link())
    pose.pose.position.z += 0.2

    # iiwa_group.clear_pose_targets()
    print('iiwa planning to pose: {}'.format(pose.pose))
    iiwa_group.set_pose_target(pose)
    result = iiwa_group.plan()
    # print('Plan result: {}'.format(result))
    # rospy.sleep(5)
    iiwa_group.execute(result, wait=True)
    rospy.sleep(10)

    # Put it back to original position
    pose = iiwa_group.get_current_pose(iiwa_group.get_end_effector_link())
    pose.pose.position.z -= 0.2

    # iiwa_group.clear_pose_targets()
    print('iiwa planning to pose: {}'.format(pose.pose))
    iiwa_group.set_pose_target(pose)
    result = iiwa_group.plan()
    # # print('Plan result: {}'.format(result))
    # # rospy.sleep(5)
    iiwa_group.execute(result, wait=True)
    #
    hand_group.detach_object(object_name)
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

    # When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    try:
        q1_script()
    except rospy.ROSInterruptException:
        pass
