
import os
import time
import threading

from qsrlib.qsrlib import QSRlib, QSRlib_Request_Message
from qsrlib_io.world_trace import Object_State,World_Trace
from opencog.type_constructors import *

import rospy

import actionlib
from rasberry_hri.msg import Action
from topological_navigation.msg import GotoNodeAction, GotoNodeGoal
from bayes_people_tracker.msg import PeopleTracker

from std_srvs.srv import Empty
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped

from bdi_system import BDISystem
from bdi_system import INF
from utils import suppress, wp2sym, sym2wp

from knowledge_base import KnowledgeBase

FREQUENCY = 0.5 # Hz

class Scheduler:


    def __init__(self, robot_id):
        rospy.loginfo("SCH: Initializing Scheduler")
        self.kb = KnowledgeBase()
        self.robot_id = robot_id
        self.latest_robot_node = None
        self.bdi = BDISystem(self.robot_id, self.kb)
        self.bdi.world_state.add_thing(self.robot_id.capitalize(), "robot")
        self.robot_pose_sub = rospy.Subscriber('/{:}/robot_pose'.format(self.robot_id), Pose, self.robot_position_coordinate_callback)
        self.robot_sub = rospy.Subscriber('/{:}/closest_node'.format(self.robot_id), String, self.robot_position_node_callback)
        self.human_action_sub = rospy.Subscriber('/human_actions_fast', Action, self.human_intention_callback)
        self.picker01_sub = rospy.Subscriber("/picker01/posestamped", PoseStamped, lambda msg: self.picker_tracker_callback(msg, "Picker01") )
        self.picker02_sub = rospy.Subscriber("/picker02/posestamped", PoseStamped, lambda msg: self.picker_tracker_callback(msg, "Picker02") )
        #TODO: move to multiple pickers
        #
        # self.people_sub = rospy.Subscriber("/people_tracker/positions", PeopleTracker, lambda msg: self.people_tracker_callback(msg, "Picker02") )


        # self.qsrlib = QSRlib()
        # self.options = sorted(self.qsrlib.qsrs_registry.keys())
        # self.which_qsr = "tpcc"#"tpcc"
        rospy.loginfo("SCH: Initialization finished")


    def spin(self):
        """
        Blocks until ROS node is shutdown. Yields activity to other threads.
        @raise ROSInitException: if node is not in a properly initialized state
        """
        bdi = None
        if not rospy.core.is_initialized():
            raise rospy.exceptions.ROSInitException("client code must call rospy.init_node() first")
        rospy.logdebug("node[%s, %s] entering spin(), pid[%s]", rospy.core.get_caller_id(), rospy.core.get_node_uri(), os.getpid())
        try:
            while not rospy.core.is_shutdown():
                # try:
                #     if not bdi.is_alive():
                #         bdi = threading.Thread(target=self.bdi.loop)
                #         bdi.start()
                #     else:
                #         # rospy.loginfo("SCH: sleeping")
                #         rospy.rostime.wallsleep(0.001)
                # except:
                #     bdi = threading.Thread(target=self.bdi.loop)
                #     bdi.start()
                self.bdi.loop()
                rospy.rostime.wallsleep(1.0/FREQUENCY)
        except KeyboardInterrupt:
            rospy.logdebug("keyboard interrupt, shutting down")
            rospy.core.signal_shutdown('keyboard interrupt')


    def robot_position_coordinate_callback(self, msg):
        # rospy.loginfo("SCH: Robot position coordinate callback")
        start_time = time.time()
        self.bdi.latest_robot_msg = msg
        duration = time.time() - start_time
        if duration > 0.01:
            rospy.loginfo("SCH: handled robot_position_coordinate_callback -- {:.4f}".format(duration))


    def robot_position_node_callback(self, msg):
        # rospy.loginfo("SCH: Robot position node callback")
        start_time = time.time()
        if msg.data != "none":
            self.latest_robot_node = wp2sym(msg.data)
            self.bdi.world_state.update_position(self.robot_id.capitalize(), self.latest_robot_node)
        else:
            self.latest_robot_node = None
        duration = time.time() - start_time
        if duration > 0.01:
            rospy.loginfo("SCH: handled robot_position_callback -- {:.4f}".format(duration))


    def human_intention_callback(self, msg):
        rospy.loginfo("SCH: Perceived human action {}, {}".format(msg.person.capitalize(), msg.action))
        person = msg.person.capitalize()
        self.bdi.world_state.update_action(person, msg.action)


    def picker_tracker_callback(self, msg, id):
        # rospy.loginfo("SCH: Picker position node callback")
        start_time = time.time()
        self.bdi.latest_people_msgs[id] = msg
        duration = time.time() - start_time
        if duration > 0.01:
            rospy.loginfo("SCH: handled picker_tracker_callback -- {:.4f}".format(duration))


    # def people_tracker_callback(self, msg, id):
    #     rospy.loginfo("SCH: Person msg: {}".format(msg))
    #     id = "Picker01"
    #     msg.angles
    #     msg = PoseStamped()
    #     indexes = [idx for idx, val in enumerate(msg.angles) if val > -0.5 and val < 0.5]
    #     min_distance = 100
    #     angle = None
    #     msg2.header = msg.header
    #     msg2.pose = None
    #     for index in indexes:
    #         distance = msg.distances[index]
    #         if distance < min_distance:
    #             min_distance = distance
    #             msg2.pose = msg.poses[index]
    #     if msg2.pose is not None:
    #         self.picker_tracker_callback(msg2)

        # world = World_Trace()
        # things = []
        # qsrlib_request_message = QSRlib_Request_Message(which_qsr="tpcc", input_data=world)
        # # request your
        #
        # position = Object_State(name=id, timestamp=msg2.header.stamp.to_sec(), x=msg2.pose.position.x, y=msg2.pose.position.y, width=0.6, length=0.4)
        # things.append(position)
        #
        # things.append(self.bdi.robot_track[-1])
        #
        # world.add_object_state_series(things)
        # qsrlib_response_message = self.qsrlib.request_qsrs(req_msg=qsrlib_request_message)
        # for t in qsrlib_response_message.qsrs.get_sorted_timestamps():
        #     foo = str(t) + ": "
        #     for k, v in zip(qsrlib_response_message.qsrs.trace[t].qsrs.keys(),
        #                     qsrlib_response_message.qsrs.trace[t].qsrs.values()):
        #         foo += str(k) + ":" + str(v.qsr) + "; "
        #     rospy.loginfo(foo)
        # world_qsr.trace[4].qsrs[self.robot_id.capitalize()+','+id].qsr['tpcc']
        # direction = ""
        # if direction in ["dsf","csf"]:
        #     self.bdi.latest_people_msgs[id] = msg
        # self.bdi.latest_people_msgs[id] = msg

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
