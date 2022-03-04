#!/usr/bin/env python3

import math
import argparse as ap

import rospy
from std_srvs.srv import Trigger, TriggerRequest
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

import hello_helpers.hello_misc as hm


class KeyboardTeleopNode(hm.HelloNode):

    def __init__(self, mapping_on=False, hello_world_on=False, open_drawer_on=False, clean_surface_on=False, grasp_object_on=False, deliver_object_on=False):
        hm.HelloNode.__init__(self)
        # self.keys = GetKeyboardCommands(mapping_on, hello_world_on, open_drawer_on, clean_surface_on, grasp_object_on, deliver_object_on)
        self.rate = 10.0
        self.joint_state = None
        self.mapping_on = mapping_on
        self.hello_world_on = hello_world_on
        self.open_drawer_on = open_drawer_on
        self.clean_surface_on = clean_surface_on
        self.grasp_object_on = grasp_object_on
        self.deliver_object_on = deliver_object_on

    def joint_states_callback(self, joint_state):
        self.joint_state = joint_state

    def send_command(self, command):
        joint_state = self.joint_state
        if (joint_state is not None) and (command is not None):
            point = JointTrajectoryPoint()
            point.time_from_start = rospy.Duration(0.0)
            trajectory_goal = FollowJointTrajectoryGoal()
            trajectory_goal.goal_time_tolerance = rospy.Time(1.0)
            
            joint_name = command['joint']
            trajectory_goal.trajectory.joint_names = [joint_name]
            if 'inc' in command:
                inc = command['inc']
                new_value = inc
            elif 'delta' in command:
                joint_index = joint_state.name.index(joint_name)
                joint_value = joint_state.position[joint_index]
                delta = command['delta']
                new_value = joint_value + delta
            point.positions = [new_value]
            trajectory_goal.trajectory.points = [point]
            trajectory_goal.trajectory.header.stamp = rospy.Time.now()
            self.trajectory_client.send_goal(trajectory_goal)

    def main(self):
        hm.HelloNode.main(self, 'keyboard_teleop', 'keyboard_teleop', wait_for_first_pointcloud=False)

        if self.mapping_on: 
            rospy.loginfo('Node ' + self.node_name + ' waiting to connect to /funmap/trigger_head_scan.')

            rospy.wait_for_service('/funmap/trigger_head_scan')
            rospy.loginfo('Node ' + self.node_name + ' connected to /funmap/trigger_head_scan.')
            self.trigger_head_scan_service = rospy.ServiceProxy('/funmap/trigger_head_scan', Trigger)

            rospy.wait_for_service('/funmap/trigger_drive_to_scan')
            rospy.loginfo('Node ' + self.node_name + ' connected to /funmap/trigger_drive_to_scan.')
            self.trigger_drive_to_scan_service = rospy.ServiceProxy('/funmap/trigger_drive_to_scan', Trigger)

            rospy.wait_for_service('/funmap/trigger_global_localization')
            rospy.loginfo('Node ' + self.node_name + ' connected to /funmap/trigger_global_localization.')
            self.trigger_global_localization_service = rospy.ServiceProxy('/funmap/trigger_global_localization', Trigger)

            rospy.wait_for_service('/funmap/trigger_local_localization')
            rospy.loginfo('Node ' + self.node_name + ' connected to /funmap/trigger_local_localization.')
            self.trigger_local_localization_service = rospy.ServiceProxy('/funmap/trigger_local_localization', Trigger)

            rospy.wait_for_service('/funmap/trigger_align_with_nearest_cliff')
            rospy.loginfo('Node ' + self.node_name + ' connected to /funmap/trigger_align_with_nearest_cliff.')
            self.trigger_align_with_nearest_cliff_service = rospy.ServiceProxy('/funmap/trigger_align_with_nearest_cliff', Trigger)

            rospy.wait_for_service('/funmap/trigger_reach_until_contact')
            rospy.loginfo('Node ' + self.node_name + ' connected to /funmap/trigger_reach_until_contact.')
            self.trigger_reach_until_contact_service = rospy.ServiceProxy('/funmap/trigger_reach_until_contact', Trigger)

            rospy.wait_for_service('/funmap/trigger_lower_until_contact')
            rospy.loginfo('Node ' + self.node_name + ' connected to /funmap/trigger_lower_until_contact.')
            self.trigger_lower_until_contact_service = rospy.ServiceProxy('/funmap/trigger_lower_until_contact', Trigger)

        if self.hello_world_on: 
            rospy.wait_for_service('/hello_world/trigger_write_hello')
            rospy.loginfo('Node ' + self.node_name + ' connected to /hello_world/trigger_write_hello.')
            self.trigger_write_hello_service = rospy.ServiceProxy('/hello_world/trigger_write_hello', Trigger)

        if self.open_drawer_on:
            rospy.wait_for_service('/open_drawer/trigger_open_drawer_down')
            rospy.loginfo('Node ' + self.node_name + ' connected to /open_drawer/trigger_open_drawer_down.')
            self.trigger_open_drawer_down_service = rospy.ServiceProxy('/open_drawer/trigger_open_drawer_down', Trigger)

            rospy.wait_for_service('/open_drawer/trigger_open_drawer_up')
            rospy.loginfo('Node ' + self.node_name + ' connected to /open_drawer/trigger_open_drawer_up.')
            self.trigger_open_drawer_up_service = rospy.ServiceProxy('/open_drawer/trigger_open_drawer_up', Trigger)

            
        if self.clean_surface_on:
            rospy.wait_for_service('/clean_surface/trigger_clean_surface')
            rospy.loginfo('Node ' + self.node_name + ' connected to /clean_surface/trigger_clean_surface.')
            self.trigger_clean_surface_service = rospy.ServiceProxy('/clean_surface/trigger_clean_surface', Trigger)

        if self.grasp_object_on:
            rospy.wait_for_service('/grasp_object/trigger_grasp_object')
            rospy.loginfo('Node ' + self.node_name + ' connected to /grasp_object/trigger_grasp_object.')
            self.trigger_grasp_object_service = rospy.ServiceProxy('/grasp_object/trigger_grasp_object', Trigger)

        if self.deliver_object_on:
            rospy.wait_for_service('/deliver_object/trigger_deliver_object')
            rospy.loginfo('Node ' + self.node_name + ' connected to /deliver_object/trigger_deliver_object.')
            self.trigger_deliver_object_service = rospy.ServiceProxy('/deliver_object/trigger_deliver_object', Trigger)

        rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)

        rate = rospy.Rate(self.rate)

        command = 1 # set equal to not None, so menu is printed out on first loop
        while not rospy.is_shutdown():
            if self.joint_state is not None:
                # self.keys.print_commands(self.joint_state, command)
                # command = self.keys.get_command(self)
                deltas = {'rad': self.small_rad, 'translate': self.small_translate}
                command = {'joint': 'translate_mobile_base', 'inc': deltas['translate']}
                self.send_command(command)
            rate.sleep()


if __name__ == '__main__':
    try:
        parser = ap.ArgumentParser(description='Keyboard teleoperation for stretch.')
        parser.add_argument('--mapping_on', action='store_true', help='Turn on mapping control. For example, the space bar will trigger a head scan. This requires that the mapping node be run (funmap).')
        parser.add_argument('--hello_world_on', action='store_true', help='Enable Hello World writing trigger, which requires connection to the appropriate hello_world service.')
        parser.add_argument('--open_drawer_on', action='store_true', help='Enable Open Drawer trigger, which requires connection to the appropriate open_drawer service.')
        parser.add_argument('--clean_surface_on', action='store_true', help='Enable Clean Surface trigger, which requires connection to the appropriate clean_surface service.')
        parser.add_argument('--grasp_object_on', action='store_true', help='Enable Grasp Object trigger, which requires connection to the appropriate grasp_object service.')
        parser.add_argument('--deliver_object_on', action='store_true', help='Enable Deliver Object trigger, which requires connection to the appropriate deliver_object service.')

        args, unknown = parser.parse_known_args()
        mapping_on = args.mapping_on
        hello_world_on = args.hello_world_on
        open_drawer_on = args.open_drawer_on
        clean_surface_on = args.clean_surface_on
        grasp_object_on = args.grasp_object_on
        deliver_object_on = args.deliver_object_on

        node = KeyboardTeleopNode(mapping_on, hello_world_on, open_drawer_on, clean_surface_on, grasp_object_on, deliver_object_on)
        node.main()
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')
