#!/usr/bin/python
import rospy
import tf

from std_msgs.msg import String, Bool
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseActionGoal
from commandTopological import CommandTopological
from commandMoveBase import CommandMoveBase

from rasberry_han.safe_actions import DynamicReconfigurePolicy
from rasberry_hri.msg import Command


class CommandsExecuter:
    def __init__(self):
        print "call something"
        self.in_polytunnel = rospy.get_param("~start_in_polytunnel", False)
        self.human_proximity_syncronization = rospy.get_param("~human_proximity_syncronization", False)
        self.monitor_dyn_reconfig = None
        self.current_node = None
        rospy.Subscriber("/lcas/hri/robot_control", Command, self.execute_action, queue_size=1)
        rospy.Subscriber("in_polytunnel", Bool, self.in_polytunnel_cb)
        rospy.Subscriber("current_node", String, self.current_node_cb)
        rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, self.move_base_fb)
        rospy.spin()


    def current_node_cb(self, msg):
        self.current_node = msg.data


    def execute_action(self, msg):
        goal = String()
        goal.data = msg.command
        rospy.loginfo("Receiving command  {:}".format(goal.data))
        if not self.in_polytunnel:
            self.actor = CommandMoveBase()
        else:
            self.actor = CommandTopological(self.current_node)

        if "hail" in goal.data and self.human_proximity_syncronization:
            self.monitor_dyn_reconfig = DynamicReconfigurePolicy()
            self.monitor_dyn_reconfig.execute()

        elif "move" in goal.data:
            if "forward" in goal.data:
                self.actor.move_forward()
            elif "backward" in goal.data:
                self.actor.move_backward()
            elif "left" in goal.data:
                self.actor.move_left()
            elif "right" in goal.data:
                self.actor.move_right()
            else:
                rospy.logerr("Move Command {:} not found".format(goal.data))
        elif "rotate" in goal.data:
            if "left" in goal.data:
                self.actor.move_left()
            elif "right" in goal.data:
                self.actor.move_right()
            else:
                rospy.logerr("Rotation Command {:} not found".format(goal.data))
        elif "block" == goal.data:
            rospy.logwarn("block multiplexer")
            self.publish_topic_with_msg("lock_all", Bool, Bool(data=True))
        elif "unblock" == goal.data:
            rospy.logwarn("unblock multiplexer")
            self.publish_topic_with_msg("lock_all", Bool, Bool(data=False))
        elif "cancel"  in goal.data:
            self.publish_topic("topological_navigation/cancel", GoalID)
            self.publish_topic("move_base/cancel", GoalID)
        else:
            rospy.logerr("Move Command {:} not found".format(goal.data))


    def move_base_fb(self,msg):
        if self.human_proximity_syncronization and self.monitor_dyn_reconfig is not None:
            self.monitor_dyn_reconfig.stop()
            self.monitor_dyn_reconfig = None
            rospy.logerr("Move Command feedback")


    def in_polytunnel_cb(self, msg):
        self.in_polytunnel = msg.data


    def publish_topic(self, topic_name, msg_class):
        rospy.logwarn("Publishing to {:}".format(topic_name))
        publisher = rospy.Publisher(topic_name, msg_class, queue_size=1)
        rospy.sleep(0.5)
        publisher.publish(msg_class())
        rospy.sleep(0.5)
        publisher.unregister()


    def publish_topic_with_msg(self, topic_name, msg_class, msg):
        rospy.logwarn("Publishing2 {:}".format(topic_name))
        publisher = rospy.Publisher(topic_name, msg_class, queue_size=1)
        rospy.sleep(0.5)
        publisher.publish(msg)
        rospy.sleep(0.5)
        publisher.unregister()
