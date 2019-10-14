
import os
import time

import rospy
from qsrlib_io.world_trace import Object_State, World_Trace

import actionlib
from rasberry_hri.msg import Action
from topological_navigation.msg import GotoNodeAction, GotoNodeGoal

from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped

from bdi_system import BDISystem
from bdi_system import INF
from utils import suppress, wp2sym, sym2wp



class Scheduler:


    def __init__(self, robot_id):
        rospy.loginfo("SCH: Initializing Scheduler")
        self.robot_id = robot_id
        self.robot_control = actionlib.SimpleActionClient('topological_navigation', GotoNodeAction)
        self.robot_control.wait_for_server()
        rospy.loginfo("SCH: Found Robot Countrol Server")
        self.latest_robot_node = None
        self.bdi = BDISystem(self.robot_id)
        self.bdi.world_state.add_belief("is_a({:},Robot)".format(self.robot_id.capitalize()))
        self.robot_sub = rospy.Subscriber('robot_pose', Pose, self.robot_position_coordinate_callback)
        self.robot_sub = rospy.Subscriber('current_node', String, self.robot_position_node_callback)
        #TODO: move to multiple pickers
        self.people_sub = rospy.Subscriber("/picker01/posestamped", PoseStamped, lambda msg: self.people_tracker_callback(msg, "picker01") )
        self.people_sub = rospy.Subscriber("/picker02/posestamped", PoseStamped, lambda msg: self.people_tracker_callback(msg, "picker02") )
        rospy.Subscriber('human_actions', Action, self.human_intention_callback)



    def spin(self):
        """
        Blocks until ROS node is shutdown. Yields activity to other threads.
        @raise ROSInitException: if node is not in a properly initialized state
        """

        if not rospy.core.is_initialized():
            raise rospy.exceptions.ROSInitException("client code must call rospy.init_node() first")
        rospy.logdebug("node[%s, %s] entering spin(), pid[%s]", rospy.core.get_caller_id(), rospy.core.get_node_uri(), os.getpid())
        try:
            while not rospy.core.is_shutdown():
                rospy.loginfo("Looping")
                self.bdi.loop()
                rospy.rostime.wallsleep(1)
        except KeyboardInterrupt:
            rospy.logdebug("keyboard interrupt, shutting down")
            rospy.core.signal_shutdown('keyboard interrupt')


    def robot_position_coordinate_callback(self, msg):
        self.bdi.robot_track.append(Object_State(name=id, timestamp=time.time(), x=msg.position.x, y=msg.position.y, width=1.35584, length=1.5))


    def robot_position_node_callback(self, msg):
        if not self.latest_robot_node is None:
            # rospy.loginfo("retracting is_at({:},{:})".format(self.robot_id, self.latest_robot_node))
            self.bdi.world_state.abandon_belief("is_at({:},{:})".format(self.robot_id.capitalize(), self.latest_robot_node.capitalize()))
            rospy.logdebug("retracted is_at({:},{:})".format(self.robot_id.capitalize(), self.latest_robot_node.capitalize()))
        if msg.data != "none":
            self.latest_robot_node = wp2sym(msg.data)
            self.bdi.world_state.add_belief("is_at({:},{:})".format(self.robot_id.capitalize(), self.latest_robot_node.capitalize()))
            rospy.logdebug("added is_at({:},{:})".format(self.robot_id.capitalize(), self.latest_robot_node.capitalize()))
        else:
            self.latest_robot_node = None


    def human_intention_callback(self, msg):
        # TODO: match detected person to symbol
        if msg.action == "has crate":
            self.bdi.world_state.add_belief("has_crate({:})".format(msg.id.capitalize()))
            rospy.logdebug("added has_crate({:})".format(msg.id.capitalize()))
        elif msg.action == "picking berries left" or msg.action == "picking berries right":
            self.bdi.world_state.add_belief("seen_picking({:})".format(msg.id.capitalize()))
            rospy.logdebug("added seen_picking({:})".format(msg.id.capitalize()))
        # self.bdi.world_state.add_belief("{:}({:})".format(msg.action, msg.id.capitalize()))
        # rospy.logdebug("added {:}({:})".format(msg.action, msg.id.capitalize()))


    def people_tracker_callback(self, msg, id):
        self.bdi.latest_people_msgs[id] = msg

        # if current_node != "none":
        #     current_node = wp2sym(current_node)
        #     self.latest_people_node[id] = ("is_at", current_node)
        #     self.bdi.world_state.add_belief("is_at({:},{:})".format(id, current_node))
        #
        #
        # else:
        #     closest_node = wp2sym(closest_node)
        #     self.latest_people_node[id] = ("is_near", closest_node)
        #     self.bdi.world_state.add_belief("is_near({:},{:})".format(id, closest_node))


    def save(self):
        self.bdi.save()
