
import rospy

from commandInterface import CommandInterface

class CommandCoordinator(CommandInterface):

    def __init__(self):
        self.local_frame = rospy.get_param("~local_frame", "base_link")
        self.commanded_distance = rospy.get_param("~commanded_distance", 1.0)
        self.commanded_rotation = rospy.get_param("~commanded_rotation", 1.0)
        self.move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base_client.wait_for_server()


    def get_goal(self):
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = self.local_frame
        move_base_goal.target_pose.pose.orientation.w = 1.0
        return move_base_goal


    def send_goal(self, move_base_goal):
        self.move_base_client.send_goal(move_base_goal)
        self.move_base_client.wait_for_result()


    def quat2goal(self, quat):
        move_base_goal = self.get_goal()
        move_base_goal.target_pose.pose.orientation.x = quat[0]
        move_base_goal.target_pose.pose.orientation.y = quat[1]
        move_base_goal.target_pose.pose.orientation.z = quat[2]
        move_base_goal.target_pose.pose.orientation.w = quat[3]
        return move_base_goal


    def move_forward(self):
        move_base_goal = self.get_goal()
        move_base_goal.target_pose.pose.position.x = self.commanded_distance
        self.send_goal(move_base_goal)


    def move_backward(self):
        move_base_goal = self.get_goal()
        move_base_goal.target_pose.pose.position.x = -self.commanded_distance
        self.send_goal(move_base_goal)


    def move_right(self):
        move_base_goal = self.get_goal()
        move_base_goal.target_pose.pose.position.y = -self.commanded_distance
        self.send_goal(move_base_goal)


    def move_left(self):
        move_base_goal = self.get_goal()
        move_base_goal.target_pose.pose.position.y = self.commanded_distance
        self.send_goal(move_base_goal)


    def turn_right(self):
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, -self.commanded_rotation)
        move_base_goal = self.quat2goal(quat)
        self.send_goal(move_base_goal)


    def turn_left(self):
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, self.commanded_rotation)
        move_base_goal = self.quat2goal(quat)
        self.send_goal(move_base_goal)
