
import os
import time

import rospy
from qsrlib_io.world_trace import Object_State, World_Trace

import actionlib
from rasberry_hri.msg import Action
from topological_navigation.msg import GotoNodeAction, GotoNodeGoal
from rasberry_people_perception.topological_localiser import TopologicalNavLoc
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped

from bdi_system import BDISystem
from bdi_system import INF
from utils import suppress



class Scheduler:


    def __init__(self, robot_id):

        self.robot_id = robot_id
        self.robot_control = actionlib.SimpleActionClient('topological_navigation', GotoNodeAction)
        self.robot_control.wait_for_server()
        self.latest_people_node = {}
        self.bdi = BDISystem(self.robot_id)

        self.bdi.world_state.db.add("is_a({:},Robot)".format(self.robot_id), 1)

        self.locator = TopologicalNavLoc()
        time.sleep(1)
        for node in self.locator.tmap.nodes:
            self.bdi.world_state.db.add("is_a({:},Place)".format(node.name), 1)

        self.robot_sub = rospy.Subscriber('robot_pose', Pose, self.robot_position_coordinate_callback)
        self.robot_sub = rospy.Subscriber('current_node', String, self.robot_position_node_callback)
        #TODO: move to multiple pickers
        self.people_sub = rospy.Subscriber("/picker01/posestamped", PoseStamped, lambda msg: self.people_tracker_callback(msg, "picker01") )
        self.people_sub = rospy.Subscriber("/picker02/posestamped", PoseStamped, lambda msg: self.people_tracker_callback(msg, "picker01") )
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
                # loop()
                rospy.rostime.wallsleep(0.5)
        except KeyboardInterrupt:
            rospy.logdebug("keyboard interrupt, shutting down")
            rospy.core.signal_shutdown('keyboard interrupt')


    def robot_position_coordinate_callback(self, msg):
        self.bdi.robot_track.append(Object_State(name=id, timestamp=time.time(), x=msg.position.x, y=msg.position.y, width=1.35584, length=1.5))


    def robot_position_node_callback(self, msg):
        rospy.loginfo(msg)
        # self.bdi.world_state.db.add("is_at({:},{:})".format(self.robot_id, msg))


    def human_intention_callback(self, msg):
        rospy.loginfo(msg)
        # TODO: match detected person to symbol
        self.bdi.world_state.db.add("{:}({:})".format(msg.action, msg.id))


    def people_tracker_callback(self, msg, id):
        #TODO write lambda wrapper for callback to send id as well
        with suppress(KeyError):
            latest_node = self.latest_people_node[id]
        self.bdi.latest_people_positions[id] = Object_State(name=id, timestamp=msg.header.stamp.to_sec(), x=msg.pose.position.x, y=msg.pose.position.y, width=0.6, length=0.4)
        (current_node, closest_node) = self.locator.localise_pose(msg)

        with suppress(UnboundLocalError):
            self.bdi.world_state.db.retract("is_at({:}, {:})".format(id, latest_node))
            self.bdi.world_state.db.retract("is_near({:}, {:})".format(id, latest_node))
        if current_node != "none":
            self.bdi.world_state.db.add("is_at({:}, {:})".format(id, current_node), 1)
        else:
            self.bdi.world_state.db.add("is_near({:}, {:})".format(id, closest_node), 1)



    def move_to(self, goal):
        print("Requesting Navigation to {:}".format(goal))
        navgoal = GotoNodeGoal()
        navgoal.target = goal
        #navgoal.origin = orig

        # Sends the goal to the action server.
        self.robot_control.send_goal(navgoal)#,self.done_cb, self.active_cb, self.feedback_cb)

        # Waits for the server to finish performing the action.
        self.robot_control.wait_for_result()

        # Prints out the result of executing the action
        ps = self.robot_control.get_result()  # A FibonacciResult
        print(ps)


    def cancel_movement(self):
        self.robot_control.cancel_all_goals()


    def save(self):
        self.bdi.save()
