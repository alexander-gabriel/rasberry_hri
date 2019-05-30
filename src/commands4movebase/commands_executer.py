#!/usr/bin/python
import rospy
import tf
import actionlib
from std_msgs.msg import String, Bool
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class CommandsExecuter:
    def __init__(self):
        print "call something"
        self.commanded_distance = rospy.get_param("~commanded_distance", 1.0)
        self.commanded_rotation = rospy.get_param("~commanded_rotation", 1.0)
        self.local_frame = rospy.get_param("~local_frame", "base_link")
        self.move_base_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.move_base_client.wait_for_server()
        rospy.Subscriber("command_movebase", String, self.execute_action)
        rospy.spin()

    def publish_topic(self, topic_name, msg_class):
        rospy.logwarn("Publishing to %s " % topic_name)
        publisher = rospy.Publisher(topic_name, msg_class, queue_size=1)
        publisher.publish(msg_class())
        publisher.unregister()
        rospy.sleep(0.5)

    def publish_topic_with_msg(self, topic_name, msg_class, msg):
        rospy.logwarn("Publishing2 to %s " % topic_name)
        publisher = rospy.Publisher(topic_name, msg_class, queue_size=1)
        rospy.sleep(0.5)
        publisher.publish(msg)
        rospy.sleep(0.5)
        publisher.unregister()

    def execute_action(self,goal):
        rospy.loginfo("Receiving command  %s" % goal.data)
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = self.local_frame
        move_base_goal.target_pose.pose.orientation.w = 1.0

        if "cancel"  in goal.data:
            self.publish_topic("topological_navigation/cancel", GoalID)
            self.publish_topic("move_base/cancel", GoalID)
            return

        if "move" in goal.data:
            if "forward" in goal.data:
                move_base_goal.target_pose.pose.position.x = self.commanded_distance
            elif "backward" in goal.data:
                move_base_goal.target_pose.pose.position.x = -self.commanded_distance
            elif "left" in goal.data:
                move_base_goal.target_pose.pose.position.y = self.commanded_distance
            elif "right" in goal.data:
                move_base_goal.target_pose.pose.position.y = -self.commanded_distance
            else:
                rospy.logerr("Move Command %s not found" % goal.data)
                return

            self.move_base_client.send_goal(move_base_goal)
            self.move_base_client.wait_for_result()
            return

        if "rotate" in goal.data:
            if "left" in goal.data:
                quat = tf.transformations.quaternion_from_euler(0.0, 0.0, self.commanded_rotation)
            elif "right" in goal.data:
                quat = tf.transformations.quaternion_from_euler(0.0, 0.0, -self.commanded_rotation)
            else:
                rospy.logerr("Rotation Command %s not found" % goal.data)
                return
            move_base_goal.target_pose.pose.orientation.x = quat[0]
            move_base_goal.target_pose.pose.orientation.y = quat[1]
            move_base_goal.target_pose.pose.orientation.z = quat[2]
            move_base_goal.target_pose.pose.orientation.w = quat[3]

            self.move_base_client.send_goal(move_base_goal)
            self.move_base_client.wait_for_result()

            return

        if "block" == goal.data:
            rospy.logwarn("block multiplexer")
            self.publish_topic_with_msg("lock_all", Bool, Bool(data=True))
            return

        if "unblock" == goal.data:
            rospy.logwarn("unblock multiplexer")
            self.publish_topic_with_msg("lock_all", Bool, Bool(data=False))
            return
