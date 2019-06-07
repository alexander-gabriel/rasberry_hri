
import rospy
import actionlib
import tf
from geometry_msgs.msg import Twist, PoseStamped
from strands_navigation_msgs.msg import TopologicalMap
from topological_navigation.msg import GotoNodeAction, GotoNodeGoal

from commandInterface import CommandInterface


class CommandTopological(CommandInterface):

    def __init__(self, current_node):
        self.topological_map = rospy.wait_for_message("topological_map", TopologicalMap)
        rospy.loginfo("Got Topological Map!")
        self.tf_listener = tf.TransformListener()
        self.current_node = current_node
        self.action_client = actionlib.SimpleActionClient('topological_navigation', GotoNodeAction)
        rospy.loginfo("Waiting for Action Server /topological_navigation")
        self.action_client.wait_for_server()
        self.current_edges = dict()
        self.find_edges()


    def find_edges(self):
        for node in self.topological_map.nodes:
            if node.name == self.current_node:
                rospy.loginfo("Node Find in topomap")
                for e in node.edges:
                    rospy.loginfo("edge to {:}".format(e.node))
                    self.current_edges[self.is_node_in_front(e.node)] = e.node


    def is_node_in_front(self, n):
        for node in self.topological_map.nodes:
            if node.name == n:
                p1 = PoseStamped()
                p1.header.frame_id = "map"
                p1.pose = node.pose
                t = rospy.Time(0)
                self.tf_listener.waitForTransform("base_link", "map", t, rospy.Duration(1.0))
                self.tf_listener.lookupTransform("base_link", "map", t)
                p1.header.stamp = t
                x = self.tf_listener.transformPose("base_link", p1)
                if x.pose.position.x > 0:
                    return "forward"
                return "backward"
        return "not_found"


    def command_velocity(self, node_name):
        navgoal = GotoNodeGoal()
        navgoal.target = node_name
        navgoal.no_orientation = False
        self.action_client.send_goal(navgoal)
        self.action_client.wait_for_result()
        rospy.loginfo("Is node %s reached? %r", node_name,  self.action_client.get_result())


    def move_forward(self):
        if not self.current_edges.has_key("forward"):
            rospy.logerr("Node not found")
            return
        self.command_velocity(self.current_edges["forward"])


    def move_backward(self):
        if not self.current_edges.has_key("backward"):
            rospy.logerr("Node not found")
            return

        self.command_velocity(self.current_edges["backward"])
